import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import subprocess
import threading
import time
import enum
import math
import os

# === USER PARAMETERS (edit if needed) ===
WAYPOINTS = [
    (0.37247, -0.10103, -13.26),
    (1.05778, -0.28532, -13.34),
    (1.71948, -0.40092, -11.97),
    (2.63431, -0.54106, 7.45),
    (3.70974, -0.48306, -12.09),
    (3.96764, -0.49506, -44.46)
]
BASE_POSE = (-0.07185, -0.08918, 7.95)
NAV_TIMEOUT = 10.0     # seconds
POSE_WAIT = 2.0        # seconds to wait for /target_pose after detection
PICK_TIMEOUT = 10.0    # seconds
DROP_TIMEOUT = 10.0    # seconds
M5_IP = os.environ.get("M5_IP", "192.168.137.75")
PICK_NODE = ("mycobot_arm", "pick_node")
DROP_NODE = ("mycobot_arm", "drop_node")
# ========================================

class MissionState(enum.Enum):
    EXPLORING = 0
    DETECTED = 1
    NAVIGATING_TO_OBJECT = 2
    PICKING = 3
    RETURNING_TO_BASE = 4
    DROPPING = 5
    COMPLETE = 6
    FAILED = 7

def say(text):
    def run():
        try:
            subprocess.call(f"espeak -s 135 -a 200 '{text}' --stdout | aplay -D plughw:1,3", shell=True)
        except Exception as e:
            print(f"[AUDIO ERROR] {e}")
    threading.Thread(target=run, daemon=True).start()

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.state = MissionState.EXPLORING
        self.current_wp = 0
        self.target_pose = None
        self.target_pose_stamp = None
        self.nav_goal_handle = None
        self.timer_handles = []

        # ROS2 setup
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_cb, 10)
        self.create_subscription(String, '/object_found', self.object_found_cb, 10)

        self.get_logger().info("🚀 Mission Manager ready.")
        say("Mission Manager started.")
        self.logstate()

        self.exploration_timer = self.create_timer(2.0, self.explore_waypoints)

    # ==== Mission Logic ====

    def logstate(self, msg=None):
        msg = msg or ""
        self.get_logger().info(f"[STATE] {self.state.name}{' - ' + msg if msg else ''}")

    def explore_waypoints(self):
        if self.state != MissionState.EXPLORING:
            return
        if self.current_wp >= len(WAYPOINTS):
            self.get_logger().info("✅ All waypoints done.")
            self.state = MissionState.COMPLETE
            say("All waypoints completed.")
            self.shutdown("Mission finished: All waypoints explored.")
            return
        x, y, yaw = WAYPOINTS[self.current_wp]
        self.get_logger().info(f"🧭 Navigating to WP {self.current_wp + 1}: ({x:.2f}, {y:.2f}, {yaw:.1f}°)")
        say(f"Navigating to waypoint {self.current_wp + 1}.")
        self.state = MissionState.EXPLORING
        self.send_nav_goal(x, y, yaw, self.on_waypoint_complete, timeout=NAV_TIMEOUT)

    def on_waypoint_complete(self, success, timed_out):
        if self.state != MissionState.EXPLORING:
            return
        if success:
            self.get_logger().info(f"✅ Reached WP {self.current_wp + 1}")
            say("Waypoint reached.")
        else:
            self.get_logger().warn(f"⚠️ Failed to reach WP {self.current_wp + 1} (timeout or nav error). Skipping.")
            say("Could not reach waypoint, skipping.")
        self.current_wp += 1
        self.state = MissionState.EXPLORING

    def object_found_cb(self, msg):
        if self.state != MissionState.EXPLORING:
            self.get_logger().info("[object_found] Ignored: Not in EXPLORING state.")
            return
        self.get_logger().warn("🛑 Object detected! Cancelling exploration and locking mission.")
        say("Object detected. Cancelling exploration.")
        self.state = MissionState.DETECTED
        self.cancel_nav_goal()
        # Wait up to POSE_WAIT sec for a valid target_pose to arrive (not busy wait)
        self.target_pose = None
        self.target_pose_stamp = None
        self.start_pose_wait_timer()

    def start_pose_wait_timer(self):
        start_time = self.get_clock().now()
        def check_pose():
            if self.target_pose and (self.get_clock().now() - self.target_pose_stamp).nanoseconds/1e9 < POSE_WAIT:
                self.get_logger().info("[POSE] Fresh target pose received, proceeding to navigate.")
                self.state = MissionState.NAVIGATING_TO_OBJECT
                self.navigate_to_object()
            elif (self.get_clock().now() - start_time).nanoseconds/1e9 > POSE_WAIT:
                self.get_logger().warn("❌ No valid target_pose received within window, skipping object.")
                say("No target pose received. Mission ending.")
                self.state = MissionState.FAILED
                self.shutdown("No target_pose received after detection.")
            else:
                # Reschedule check
                self.create_timer(0.2, check_pose)
        self.create_timer(0.2, check_pose)

    def target_pose_cb(self, msg):
        # Always store the most recent pose with its timestamp
        self.target_pose = msg
        self.target_pose_stamp = self.get_clock().now()

    def navigate_to_object(self):
        if not self.target_pose:
            self.get_logger().error("No target_pose to navigate to.")
            self.state = MissionState.FAILED
            self.shutdown("No target_pose at navigation.")
            return
        pose = self.target_pose.pose
        self.get_logger().info(f"🎯 Navigating to object at ({pose.position.x:.2f}, {pose.position.y:.2f})")
        say("Navigating to object.")
        self.send_nav_goal(pose.position.x, pose.position.y, self.yaw_from_quat(pose.orientation), self.on_nav_object_complete, timeout=NAV_TIMEOUT)

    def on_nav_object_complete(self, success, timed_out):
        if timed_out:
            self.get_logger().warn("⏳ Navigation to object timed out. Proceeding to pick anyway.")
            say("Navigation timeout, proceeding to pick.")
        elif not success:
            self.get_logger().warn("❌ Navigation to object failed. Proceeding to pick anyway.")
            say("Navigation failed, proceeding to pick.")
        else:
            self.get_logger().info("✅ Reached object. Starting pick.")
            say("Object reached. Starting pick.")
        self.state = MissionState.PICKING
        self.run_pick_node()

    def run_pick_node(self):
        self.get_logger().info("🤖 Running pick node.")
        self.run_subprocess(PICK_NODE, PICK_TIMEOUT, self.on_pick_done)

    def on_pick_done(self, success):
        if not success:
            self.get_logger().error("❌ Pick node failed or timed out.")
            say("Pick failed. Mission ending.")
            self.state = MissionState.FAILED
            self.shutdown("Pick failed.")
            return
        self.get_logger().info("✅ Pick complete. Returning to base.")
        say("Pick complete. Returning to base.")
        self.state = MissionState.RETURNING_TO_BASE
        x, y, yaw = BASE_POSE
        self.send_nav_goal(x, y, yaw, self.on_base_returned, timeout=NAV_TIMEOUT)

    def on_base_returned(self, success, timed_out):
        if not success or timed_out:
            self.get_logger().warn("⚠️ Return to base failed or timed out. Proceeding to drop anyway.")
            say("Return to base failed. Proceeding to drop.")
        else:
            self.get_logger().info("✅ Returned to base. Starting drop.")
            say("At base. Starting drop.")
        self.state = MissionState.DROPPING
        self.run_subprocess(DROP_NODE, DROP_TIMEOUT, self.on_drop_done)

    def on_drop_done(self, success):
        if not success:
            self.get_logger().error("❌ Drop node failed or timed out.")
            say("Drop failed. Mission ending.")
            self.state = MissionState.FAILED
            self.shutdown("Drop failed.")
            return
        self.get_logger().info("✅ Drop complete. Mission finished!")
        say("Drop complete. Mission accomplished.")
        self.state = MissionState.COMPLETE
        self.shutdown("Mission complete.")

    def shutdown(self, reason=""):
        self.get_logger().info(f"[MISSION] Shutdown: {reason}")
        say("Mission manager shutting down.")
        for timer in self.timer_handles:
            timer.cancel()
        rclpy.shutdown()

    # ==== Core Helpers ====

    def send_nav_goal(self, x, y, yaw_deg, done_cb, timeout=NAV_TIMEOUT):
        self.cancel_nav_goal()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Wait for server, then send goal
        def send():
            if self.nav_client.wait_for_server(timeout_sec=2.0):
                send_goal_future = self.nav_client.send_goal_async(goal)
                send_goal_future.add_done_callback(lambda f: self.on_goal_response(f, done_cb, timeout))
            else:
                self.get_logger().warn("⏳ Nav2 server not available, retrying in 2s...")
                self.create_timer(2.0, send)
        send()

    def on_goal_response(self, future, done_cb, timeout):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Nav goal rejected.")
            done_cb(False, False)
            return
        self.nav_goal_handle = goal_handle
        nav_done = threading.Event()
        timed_out = [False]
        # Start watchdog timer
        def nav_timeout():
            if not nav_done.is_set():
                self.get_logger().warn(f"⏰ Nav goal timeout after {timeout}s! Cancelling goal.")
                timed_out[0] = True
                self.cancel_nav_goal()
                done_cb(False, True)
        watchdog = threading.Timer(timeout, nav_timeout)
        watchdog.start()
        # Handle result
        def result_cb(fut):
            if not nav_done.is_set():
                nav_done.set()
                watchdog.cancel()
                result = fut.result().result
                self.get_logger().info(f"🏁 Nav2 result: {result}")
                done_cb(True, False)
        self.nav_goal_handle.get_result_async().add_done_callback(result_cb)

    def cancel_nav_goal(self):
        if self.nav_goal_handle:
            try:
                self.nav_goal_handle.cancel_goal()
                self.get_logger().info("🛑 Cancelled active nav goal.")
            except Exception as e:
                self.get_logger().warn(f"Cancel nav goal failed: {e}")
            self.nav_goal_handle = None

    def run_subprocess(self, node_tuple, timeout_sec, done_cb):
        pkg, exe = node_tuple
        def _run():
            try:
                proc = subprocess.Popen([
                    "ros2", "run", pkg, exe, "--ros-args", "-p", f"m5_ip:={M5_IP}"
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    outs, errs = proc.communicate(timeout=timeout_sec)
                    if proc.returncode == 0:
                        done_cb(True)
                    else:
                        self.get_logger().error(f"[{exe}] Error: {errs.decode().strip()}")
                        done_cb(False)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    self.get_logger().error(f"[{exe}] Timeout after {timeout_sec}s.")
                    done_cb(False)
            except Exception as e:
                self.get_logger().error(f"Failed to launch {exe}: {e}")
                done_cb(False)
        threading.Thread(target=_run, daemon=True).start()

    @staticmethod
    def yaw_from_quat(q):
        # Converts quaternion to yaw angle in degrees
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
