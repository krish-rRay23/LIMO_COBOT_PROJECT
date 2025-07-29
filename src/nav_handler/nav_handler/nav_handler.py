import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import subprocess
import math
import shlex
import os
import time

WAYPOINTS = [
    (-0.78, -0.23, 7.9),
    (-0.49,  0.00, -33.3),
    (-0.22, -0.15, -55.0),
    ( 0.15, -0.62, -29.2),
    ( 1.31, -0.65, -9.5),
    ( 2.20, -0.61, 2.0),
    ( 3.71, -0.53, -3.5),
]

BASE_X = 1.9525
BASE_Y = -9.8651
BASE_YAW = 108.25

class NavigationHandler(Node):
    def __init__(self):
        super().__init__('nav_handler')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.target_sub = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.object_sub = self.create_subscription(String, '/object_found', self.object_detected_callback, 10)

        self.current_wp = 0
        self.mode = "exploring"
        self.latest_pose = None
        self.active_goal_handle = None
        self.m5_ip = os.environ.get("M5_IP", "192.168.137.75")

        self.timeout_timer = None
        self.nav_attempted = False
        self.mission_locked = False

        self.say("Navigation handler initialized.")
        self.get_logger().info(f"✅ NavHandler ready. Using M5 IP: {self.m5_ip}")

        self.timer = self.create_timer(2.0, self.main_loop)

    def say(self, text):
        try:
            subprocess.call(f"espeak -s 135 -a 200 {shlex.quote(text)} --stdout | aplay -D plughw:1,3", shell=True)
        except Exception as e:
            self.get_logger().warn(f"Voice error: {e}")

    def main_loop(self):
        if self.mode != "exploring" or self.mission_locked:
            return

        if self.current_wp >= len(WAYPOINTS):
            self.say("All waypoints complete.")
            self.get_logger().info("🎉 Finished all waypoints.")
            self.mode = "done"
            return

        x, y, yaw = WAYPOINTS[self.current_wp]
        self.say(f"Navigating to waypoint {self.current_wp + 1}")
        self.get_logger().info(f"🚀 Going to WP {self.current_wp + 1}: ({x}, {y}, {yaw}°)")
        self.mode = "navigating"
        self.navigate_to_pose(x, y, yaw, self.on_waypoint_done)

    def on_waypoint_done(self):
        if self.mode == "navigating":
            self.say("Waypoint reached.")
            self.current_wp += 1
            self.mode = "exploring"

    def object_detected_callback(self, msg):
        if self.mission_locked:
            self.get_logger().warn("⚠️ Object already detected and mission locked. Ignoring duplicate detection.")
            return

        if self.mode not in ["exploring", "navigating"]:
            return

        self.say("Object detected. Cancelling goal.")
        self.get_logger().warn("🛑 Object detected! Cancelling current goal and entering 10s mission timeout...")

        self.cancel_current_goal()
        self.mode = "detection_timeout"
        self.nav_attempted = False
        self.mission_locked = True

        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.timeout_timer = self.create_timer(10.0, self.force_next_stage)

        if self.latest_pose:
            self.start_object_approach(self.latest_pose)

    def target_pose_callback(self, pose):
        self.latest_pose = pose
        if self.mode == "detection_timeout" and not self.nav_attempted:
            self.start_object_approach(pose)

    def start_object_approach(self, pose):
        if self.mode != "detection_timeout":
            return

        self.say("Navigating to object.")
        x, y = pose.pose.position.x, pose.pose.position.y
        self.get_logger().info(f"🎯 Target at ({x:.2f}, {y:.2f})")
        self.mode = "approaching_object"
        self.nav_attempted = True
        self.navigate_to_pose(x, y, 0.0, self.on_reach_object)

    def force_next_stage(self):
        self.get_logger().warn("⏳ 10s timeout reached. Proceeding regardless.")
        self.on_reach_object()

    def on_reach_object(self):
        if self.timeout_timer:
            self.timeout_timer.cancel()

        if self.mode in ["picking", "returning", "dropping", "done"]:
            self.get_logger().info("⚠️ on_reach_object() called, but mission is already in progress or done. Ignoring.")
            return

        self.mode = "picking"
        self.say("At object. Starting pick.")
        self.get_logger().info("🤖 Executing pick_node...")

        try:
            subprocess.run([
                "ros2", "run", "mycobot_arm", "pick_node",
                "--ros-args", "-p", f"m5_ip:={self.m5_ip}"
            ], check=True)
            self.say("Pick complete. Returning to base.")
            self.mode = "returning"
            self.navigate_to_pose(BASE_X, BASE_Y, BASE_YAW, self.on_return_to_base)
        except Exception as e:
            self.get_logger().error(f"❌ Pick failed: {e}")
            self.say("Pick failed.")
            self.mode = "done"

    def on_return_to_base(self):
        if self.mode != "returning":
            return
        self.mode = "dropping"
        self.say("At base. Starting drop.")
        self.get_logger().info("🪣 Executing drop_node...")

        try:
            subprocess.run([
                "ros2", "run", "mycobot_arm", "drop_node",
                "--ros-args", "-p", f"m5_ip:={self.m5_ip}"
            ], check=True)
            self.say("Drop complete. Mission done.")
        except Exception as e:
            self.get_logger().error(f"❌ Drop failed: {e}")
            self.say("Drop failed.")

        self.mode = "done"
        self.timer.cancel()

    def navigate_to_pose(self, x, y, yaw_deg, on_done=None):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        def send_goal():
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.handle_goal_response(f, on_done))

        def wait_for_server():
            if self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("📤 Sending goal...")
                send_goal()
            else:
                self.get_logger().warn("⏳ Waiting for Nav2 server...")
                self.create_timer(1.0, wait_for_server)

        wait_for_server()

    def handle_goal_response(self, future, on_done):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected.")
            return
        self.active_goal_handle = goal_handle
        self.get_logger().info("✅ Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r: self.on_navigation_complete(r, on_done))

    def cancel_current_goal(self):
        if self.active_goal_handle:
            self.active_goal_handle.cancel_goal()
            self.get_logger().info("🛑 Goal canceled.")
            self.active_goal_handle = None

    def on_navigation_complete(self, future, on_done):
        self.say("Navigation complete.")
        result = future.result().result
        self.get_logger().info(f"🏁 Navigation result: {result}")
        if on_done:
            on_done()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
