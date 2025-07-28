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
        self.target_pose = None
        self.m5_ip = os.environ.get("M5_IP", "192.168.137.75")
        self.active_goal_handle = None
        self.received_target_once = False
        self.awaiting_since = None
        self.timeout_timer = None
        self.object_navigation_start = None

        self.say("Navigation handler initialized.")
        self.get_logger().info(f"✅ NavHandler ready. Using M5 IP: {self.m5_ip}")

        self.timer = self.create_timer(2.0, self.main_loop)

    def say(self, text):
        try:
            subprocess.call(
                f"espeak -s 135 -a 200 {shlex.quote(text)} --stdout | aplay -D plughw:1,3",
                shell=True
            )
        except Exception as e:
            self.get_logger().warn(f"Voice error: {e}")

    def main_loop(self):
        if self.mode != "exploring":
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
        if self.mode in ["exploring", "navigating"]:
            self.say("Object detected. Stopping.")
            self.get_logger().warn("🛑 Object detected! Canceling current goal and awaiting target_pose...")
            self.mode = "awaiting_target"
            self.received_target_once = False
            self.awaiting_since = self.get_clock().now().nanoseconds
            self.cancel_current_goal()

            if self.timeout_timer:
                self.timeout_timer.cancel()
            self.timeout_timer = self.create_timer(10.0, self.target_pose_timeout)

    def target_pose_timeout(self):
        if self.mode == "approaching_object":
            elapsed = time.time() - self.object_navigation_start if self.object_navigation_start else 999
            if elapsed >= 10:
                self.get_logger().warn("⏳ Timeout while navigating to object. Skipping to pick sequence.")
                self.on_reach_object()
        elif self.mode == "awaiting_target" and not self.received_target_once:
            self.get_logger().warn("⏳ Timeout: No target_pose received. Skipping to pick sequence.")
            self.on_reach_object()

    def target_pose_callback(self, pose):
        now = self.get_clock().now().nanoseconds
        if self.received_target_once:
            self.get_logger().info("⚠️ Ignoring duplicate /target_pose.")
            return

        if self.mode != "awaiting_target":
            self.get_logger().info("⚠️ Ignoring /target_pose: not in awaiting_target mode.")
            return

        if self.awaiting_since and pose.header.stamp.sec * 1e9 + pose.header.stamp.nanosec < self.awaiting_since:
            self.get_logger().info("⚠️ Ignoring stale /target_pose published before awaiting_target started.")
            return

        self.received_target_once = True
        if self.timeout_timer:
            self.timeout_timer.cancel()
        self.target_pose = pose
        self.say("Navigating to detected object.")
        self.get_logger().info(f"🎯 Target at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.mode = "approaching_object"
        self.object_navigation_start = time.time()
        self.navigate_to_pose(pose.pose.position.x, pose.pose.position.y, 0.0, self.on_reach_object)

    def on_reach_object(self):
        if self.mode not in ["approaching_object", "awaiting_target"]:
            return
        self.mode = "picking"
        self.say("Reached object. Starting pick.")
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
            self.get_logger().error(f"❌ Pick node failed: {e}")
            self.say("Failed to pick object.")
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
            self.say("Drop complete. Mission over.")
            self.get_logger().info("✅ Drop complete. Mission done.")
        except Exception as e:
            self.get_logger().error(f"❌ Drop node failed: {e}")
            self.say("Failed to drop object.")

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
                self.get_logger().info("📤 Sending navigation goal...")
                send_goal()
            else:
                self.get_logger().warn("⏳ Waiting for Nav2 server...")
                self.create_timer(1.0, wait_for_server)

        wait_for_server()

    def handle_goal_response(self, future, on_done):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal was rejected.")
            return
        self.get_logger().info("✅ Goal accepted.")
        self.active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r: self.on_navigation_complete(r, on_done))

    def cancel_current_goal(self):
        if self.active_goal_handle:
            self.active_goal_handle.cancel_goal()
            self.get_logger().info("🛑 Active goal canceled.")
            self.active_goal_handle = None

    def on_navigation_complete(self, future, on_done):
        self.say("Navigation complete.")
        result = future.result().result
        self.get_logger().info(f"🏁 Navigation done. Result: {result}")
        if on_done:
            self.get_logger().info("▶️ Calling post-navigation callback.")
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
