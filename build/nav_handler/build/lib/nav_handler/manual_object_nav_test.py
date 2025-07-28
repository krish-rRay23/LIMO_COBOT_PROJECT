import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class ObjectNavTester(Node):
    def __init__(self):
        super().__init__('object_nav_tester')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        self.robot_x = None
        self.robot_y = None
        self.target_received = False

        self.get_logger().info("🧪 Waiting for /target_pose from YOLO and /amcl_pose from robot...")

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def pose_callback(self, msg):
        if self.target_received or self.robot_x is None or self.robot_y is None:
            return

        self.target_received = True

        # === Raw object position ===
        obj_x = msg.pose.position.x
        obj_y = msg.pose.position.y

        # === Calculate direction robot should face ===
        dx = obj_x - self.robot_x
        dy = obj_y - self.robot_y
        yaw_rad = math.atan2(dy, dx)

        # === Offset the goal 35cm away from the object ===
        offset = 0.35
        goal_x = obj_x - math.cos(yaw_rad) * offset
        goal_y = obj_y - math.sin(yaw_rad) * offset

        # === Convert yaw to quaternion ===
        quat_z = math.sin(yaw_rad / 2.0)
        quat_w = math.cos(yaw_rad / 2.0)

        # === Log everything ===
        self.get_logger().info(f"📍 Target: ({obj_x:.2f}, {obj_y:.2f})")
        self.get_logger().info(f"🤖 Robot: ({self.robot_x:.2f}, {self.robot_y:.2f})")
        self.get_logger().info(f"🎯 Sending goal to: ({goal_x:.2f}, {goal_y:.2f}) facing yaw={math.degrees(yaw_rad):.1f}°")

        # === Build navigation goal ===
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.z = quat_z
        goal.pose.pose.orientation.w = quat_w

        def done_cb(future):
            result = future.result().result
            self.get_logger().info("✅ Navigation complete!")
            self.get_logger().info(f"Result: {result}")
            if rclpy.ok():
                rclpy.shutdown()

        def send():
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.handle_goal_response(f, done_cb))

        def wait_for_server():
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("📤 Sending goal to Nav2...")
                send()
            else:
                self.get_logger().warn("⏳ Waiting for Nav2 server...")
                self.create_timer(1.0, wait_for_server)

        wait_for_server()

    def handle_goal_response(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal was rejected!")
            if rclpy.ok():
                rclpy.shutdown()
            return

        self.get_logger().info("✅ Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(done_cb)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavTester()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
