import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time

class ExplorationHandler(Node):
    def __init__(self):
        super().__init__('exploration_handler')

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_sent = False
        self.exploring = True
        self.object_found = False
        self.current_index = 0
        self._goal_handle = None

        self.waypoints = [
            (2.3, -9.0, 90.0),
            (2.8, -8.5, 90.0),
            (3.2, -9.8, 0.0),
            (1.6, -10.3, 180.0),
            (1.2, -9.0, 270.0),
        ]

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.last_detection_time = 0.0
        self.detection_cooldown = 3.0

        self.create_subscription(String, '/object_found', self.yolo_callback, 10)
        self.create_timer(1.0, self.main_loop)

        # ✅ Resume exploration service
        self.resume_srv = self.create_service(Trigger, 'resume_exploration', self.resume_exploration_callback)

        self.get_logger().info("🧭 ExplorationHandler initialized and ready.")

    def main_loop(self):
        if not self.exploring or self.goal_sent:
            return
        self.send_next_goal()

    def send_next_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("⏳ Waiting for Nav2 action server...")
            return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ All waypoints completed.")
            self.exploring = False
            return

        x, y, yaw_deg = self.waypoints[self.current_index]
        yaw_rad = math.radians(yaw_deg)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"🚀 Sending waypoint {self.current_index + 1} → ({x:.2f}, {y:.2f})")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True

    def resume_exploration_callback(self, request, response):
        self.get_logger().info("🔄 Resume request received. Resuming exploration...")
        self.exploring = True
        self.goal_sent = False
        self.send_next_goal()
        response.success = True
        response.message = "Exploration resumed"
        return response

    def yolo_callback(self, msg):
        now = time.time()
        if now - self.last_detection_time < self.detection_cooldown:
            self.get_logger().warn("⏱️ Ignoring repeated detection (cooldown active).")
            return

        self.last_detection_time = now

        if self.object_found:
            return

        self.get_logger().warn("🛑 Object found — switching to object navigation mode.")
        self.object_found = True
        self.exploring = False

        if self.goal_sent and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.get_logger().info("🛑 Cancelled current navigation goal.")
            self.goal_sent = False

        try:
            label, cx, cy, coord_str = msg.data.split(':')
            x, y, z = map(float, coord_str.split(','))

            object_pose = PoseStamped()
            object_pose.header.frame_id = 'map'
            object_pose.header.stamp = self.get_clock().now().to_msg()
            object_pose.pose.position.x = x
            object_pose.pose.position.y = y
            object_pose.pose.position.z = 0.0
            object_pose.pose.orientation.w = 1.0

            self.pose_pub.publish(object_pose)
            self.get_logger().info(f"📤 Published object pose → ({x:.2f}, {y:.2f}) on /detected_object_pose")
            self.get_logger().info("⏸ Exploration paused. Awaiting next action (e.g., pick).")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse object detection: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal rejected.")
            self.goal_sent = False
            return

        self._goal_handle = goal_handle
        self.get_logger().info("📍 Goal accepted. Waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("🏁 Goal reached successfully.")

        if self.object_found:
            self.get_logger().info("⏸ Exploration paused. Awaiting next action (e.g., pick).")
            self.goal_sent = False
            return

        self.current_index += 1
        self.goal_sent = False

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
