import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import subprocess
import time
import math
import threading

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter("drop_x", 2.8)
        self.declare_parameter("drop_y", -8.5)
        self.declare_parameter("drop_yaw", 90.0)

        self.drop_pose = self.build_pose(
            self.get_parameter("drop_x").value,
            self.get_parameter("drop_y").value,
            self.get_parameter("drop_yaw").value
        )

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_object_pose',
            self.pose_callback,
            qos
        )
        self.get_logger().info("📡 Subscribed to /detected_object_pose with QoS BEST_EFFORT")

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.resume_client = self.create_client(Trigger, 'resume_exploration')

        self.object_detected = False
        self.processing = False
        self.lock = threading.Lock()

        self.get_logger().info("🧠 Mission Manager ready.")

    def pose_callback(self, pose_msg):
        with self.lock:
            if self.object_detected or self.processing:
                return
            self.object_detected = True
            self.processing = True

        self.get_logger().info(
            f"🎯 Received object pose → Navigating to ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})"
        )

        self.navigate_to_pose(pose_msg, self.handle_pick)

    def handle_pick(self):
        self.get_logger().info("🦾 Starting pick sequence...")
        self.run_script("pick_node")
        self.get_logger().info("📦 Pick complete. Navigating to drop zone...")
        self.navigate_to_pose(self.drop_pose, self.handle_drop)

    def handle_drop(self):
        self.get_logger().info("🪣 Starting drop sequence...")
        self.run_script("drop_node")
        self.get_logger().info("✅ Mission complete. Requesting exploration resume...")

        while not self.resume_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("⏳ Waiting for resume_exploration service...")

        future = self.resume_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info("✅ Exploration resumed successfully.")
        else:
            self.get_logger().error("❌ Resume failed: " + future.result().message)

        self.object_detected = False
        self.processing = False

    def build_pose(self, x, y, yaw_deg):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def navigate_to_pose(self, pose, on_done):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 action server not available.")
            self.processing = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info("🚀 Sending navigation goal...")
        send_future = self.nav_client.send_goal_async(goal)

        def goal_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("❌ Navigation goal rejected.")
                self.processing = False
                return
            self.get_logger().info("📍 Goal accepted, waiting...")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda _: on_done())

        send_future.add_done_callback(goal_callback)

    def run_script(self, node_name):
        try:
            subprocess.run(['ros2', 'run', 'nav_handler', node_name], check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Failed to run {node_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
