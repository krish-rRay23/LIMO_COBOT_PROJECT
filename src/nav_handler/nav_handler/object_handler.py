import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import time
import subprocess
from action_msgs.msg import GoalStatus

class ObjectHandler(Node):
    def __init__(self):
        super().__init__('object_handler')

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_sent = False
        self.object_pose = None
        self.initial_pose = None
        self.active_goal_handle = None

        self.declare_parameter("m5_ip", "192.168.137.75")
        self.m5_ip = self.get_parameter("m5_ip").value


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

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Navigation goal was rejected.")
            self.goal_sent = False
            return

        self.get_logger().info("📡 Navigation goal accepted.")
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✅ Goal reached. Executing pick...")
            self.play_sound("Object reached")
            self.run_arm_pick()
            self.return_to_base()
        else:
            self.get_logger().error("⚠️ Failed to reach goal.")

        self.goal_sent = False

    def return_to_base(self):
        if not self.initial_pose:
            self.get_logger().error("❌ Initial pose not saved. Cannot return.")
            return

        self.get_logger().info("🔁 Returning to base...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.initial_pose
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.return_goal_response)

    def return_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Return-to-base goal was rejected.")
            return

        self.get_logger().info("📡 Return-to-base goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.return_result_callback)

    def return_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🏁 Returned to base. Dropping object...")
            self.run_arm_drop()
            self.play_sound("Returned to base")
        else:
            self.get_logger().error("⚠️ Failed to return to base.")

    def run_arm_pick(self):
        try:
            self.get_logger().info("🦾 Running arm pick_node...")
            subprocess.call(f"ros2 run mycobot_arm pick_node --ros-args -p m5_ip:={self.m5_ip}", shell=True)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Pick script failed: {e}")

    def run_arm_drop(self):
        try:
            self.get_logger().info("🧺 Running arm drop_node...")
            subprocess.call(f"ros2 run mycobot_arm drop_node --ros-args -p m5_ip:={self.m5_ip}", shell=True)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Drop script failed: {e}")

    def play_sound(self, text="Object reached"):
        try:
            subprocess.call(f"espeak '{text}' --stdout | aplay -D plughw:1,3", shell=True)
        except:
            self.get_logger().warn("🔇 Failed to play sound.")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()