import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import subprocess


class ObjectHandler(Node):
    def __init__(self):
        super().__init__('object_handler')
        self.navigator = BasicNavigator()

        self.goal_sent = False
        self.object_pose = None
        self.initial_pose = None
        self.yolo_sub = self.create_subscription(String, '/object_found', self.object_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/amcl_pose', self.pose_callback, 10)

        self.get_logger().info("🧠 ObjectHandler is active. Waiting for detection...")

    def pose_callback(self, msg):
        if not self.initial_pose:
            self.initial_pose = msg
            self.get_logger().info("📍 Saved initial pose.")
            self.pose_sub.destroy()

    def object_callback(self, msg):
        if self.goal_sent:
            return  # Already navigating

        try:
            label, cx, cy, coords = msg.data.split(':')
            x, y, z = map(float, coords.split(','))
            self.object_pose = (x, y)
            self.send_goal(x, y)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse object msg: {e}")

    def send_goal(self, x, y):
        self.goal_sent = True
        self.get_logger().info(f"🚀 Sending nav goal to ({x:.2f}, {y:.2f})")

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.navigator.go_to_pose(goal)

        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("✅ Goal reached. Playing sound...")
            self.play_sound()
            self.wait_for_arm()
            self.return_to_base()
        else:
            self.get_logger().error("⚠️ Failed to reach goal.")
        self.goal_sent = False

    def play_sound(self):
        try:
            subprocess.call("espeak 'Object reached' --stdout | aplay -D plughw:1,3", shell=True)
        except:
            self.get_logger().warn("🔇 Failed to play sound.")

    def wait_for_arm(self):
        self.get_logger().info("🦾 Waiting for arm (placeholder 10s)...")
        time.sleep(10)  # Replace this with /arm_done check later

    def return_to_base(self):
        if not self.initial_pose:
            self.get_logger().error("❌ Initial pose not saved. Cannot return.")
            return
        self.get_logger().info("🔁 Returning to base...")
        self.navigator.go_to_pose(self.initial_pose)

        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("🏁 Returned to base.")
            self.play_sound()
        else:
            self.get_logger().error("⚠️ Failed to return to base.")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
