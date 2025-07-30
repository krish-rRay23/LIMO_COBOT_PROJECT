import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
import math
import subprocess
import shlex
import os

WAYPOINTS = [
    (0.37247, -0.10103, -13.26),
    (1.05778, -0.28532, -13.34),
    (1.71948, -0.40092, -11.97),
    (2.63431, -0.54106, 7.45),
    (3.70974, -0.48306, -12.09),
    (3.96764, -0.49506, -44.46)
]

BASE_POSE = (-0.07185, -0.08918, 7.95)

PICK_NODE = ("mycobot_arm", "pick_node")
DROP_NODE = ("mycobot_arm", "drop_node")
M5_IP = os.environ.get("M5_IP", "192.168.137.75")
PICK_TIMEOUT = 20
DROP_TIMEOUT = 20

SCAN_DEGREES = 270
SCAN_SPEED = 0.5

class State:
    IDLE = 'idle'
    NAVIGATING = 'navigating'
    SCANNING = 'scanning'
    PICKING = 'picking'
    RETURNING = 'returning'
    DROPPING = 'dropping'
    DONE = 'done'

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = State.IDLE
        self.current_wp = 0
        self.timer = self.create_timer(1.0, self.main_loop)

        self.scan_timer = None
        self.scan_start_time = None
        self.scan_duration = None
        self.scan_twist = Twist()

        self.say("Exploration initialized.")
        self.get_logger().info("🟢 Exploration node ready.")

    def say(self, text):
        try:
            subprocess.call(f"espeak -s 135 -a 200 {shlex.quote(text)} --stdout | aplay -D plughw:1,3", shell=True)
        except Exception as e:
            self.get_logger().warn(f"TTS failed: {e}")

    def main_loop(self):
        if self.state == State.IDLE:
            if self.current_wp < len(WAYPOINTS):
                self.navigate_to(WAYPOINTS[self.current_wp])
                self.state = State.NAVIGATING
            else:
                self.say("Waypoints done. Starting pick operation.")
                self.get_logger().info("Waypoints done. Running pick node at last position.")
                self.state = State.PICKING
                self.run_arm_node(PICK_NODE, PICK_TIMEOUT, self.on_pick_complete)

        elif self.state == State.NAVIGATING:
            pass  # Wait for async callback

        elif self.state == State.SCANNING:
            pass  # Scan handled by timer

        elif self.state == State.PICKING:
            pass  # Wait for pick complete

        elif self.state == State.RETURNING:
            pass  # Wait for nav complete

        elif self.state == State.DROPPING:
            pass  # Wait for drop complete

        elif self.state == State.DONE:
            self.say("Mission complete.")
            self.get_logger().info("🏁 Mission done.")
            self.timer.cancel()

    def navigate_to(self, target):
        x, y, yaw = target
        self.get_logger().info(f"🚀 Navigating to: ({x:.2f}, {y:.2f}, {yaw:.1f}°)")
        self.say("Navigating")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw_rad = math.radians(yaw)
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        def send_goal():
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(self.handle_goal_response)

        if self.nav_client.wait_for_server(timeout_sec=2.0):
            send_goal()
        else:
            self.get_logger().warn("❌ Nav2 server not ready!")

    def handle_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal rejected.")
            self.state = State.IDLE
            return

        self.get_logger().info("✅ Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_complete)

    def on_goal_complete(self, future):
        self.get_logger().info("🏁 Goal reached.")
        self.say("Waypoint reached")

        if self.state == State.RETURNING:
            self.say("Back to base. Starting drop.")
            self.get_logger().info("At base, running drop node.")
            self.state = State.DROPPING
            self.run_arm_node(DROP_NODE, DROP_TIMEOUT, self.on_drop_complete)
        elif self.state == State.NAVIGATING:
            self.start_rotation()

    def start_rotation(self, speed=SCAN_SPEED, angle_deg=SCAN_DEGREES):
        self.get_logger().info(f"🔄 Starting scan: {angle_deg}° at {speed} rad/s")
        self.scan_duration = math.radians(angle_deg) / speed
        self.scan_start_time = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()
        twist.angular.z = speed
        self.scan_twist = twist
        self.scan_timer = self.create_timer(0.1, self.scan_loop)
        self.state = State.SCANNING

    def scan_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.scan_start_time
        if elapsed < self.scan_duration:
            self.cmd_vel_pub.publish(self.scan_twist)
        else:
            self.cmd_vel_pub.publish(Twist())  # stop
            self.scan_timer.cancel()
            self.scan_timer = None
            self.get_logger().info("✅ Scan complete.")
            self.say("Scan complete")
            self.current_wp += 1
            self.state = State.IDLE

    def run_arm_node(self, node_tuple, timeout, done_cb):
        pkg, exe = node_tuple
        def _run():
            try:
                proc = subprocess.Popen([
                    "ros2", "run", pkg, exe, "--ros-args", "-p", f"m5_ip:={M5_IP}"
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    outs, errs = proc.communicate(timeout=timeout)
                    if proc.returncode == 0:
                        self.get_logger().info(f"[{exe}] Succeeded.")
                        done_cb(True)
                    else:
                        self.get_logger().error(f"[{exe}] Error: {errs.decode().strip()}")
                        done_cb(False)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    self.get_logger().error(f"[{exe}] Timeout after {timeout}s.")
                    done_cb(False)
            except Exception as e:
                self.get_logger().error(f"Failed to launch {exe}: {e}")
                done_cb(False)
        import threading
        threading.Thread(target=_run, daemon=True).start()

    def on_pick_complete(self, success):
        if not success:
            self.say("Pick failed. Mission ends.")
            self.get_logger().error("❌ Pick failed.")
            self.state = State.DONE
            return
        self.say("Pick done. Returning to base.")
        self.get_logger().info("Pick complete, returning to base pose.")
        self.state = State.RETURNING
        self.navigate_to(BASE_POSE)

    def on_drop_complete(self, success):
        if not success:
            self.say("Drop failed. Mission ends.")
            self.get_logger().error("❌ Drop failed.")
        else:
            self.say("Drop done. Mission complete.")
            self.get_logger().info("Drop complete. Mission done.")
        self.state = State.DONE

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
