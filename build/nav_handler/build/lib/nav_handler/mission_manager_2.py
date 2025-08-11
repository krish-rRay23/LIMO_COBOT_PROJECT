import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import os
import copy

LOGFILE = 'navigate_to_object_debug.log'
APPROACH_OFFSETS = [0.25, 0.20, 0.15, 0.10, 0.07, 0.05]  # in meters (try closer and closer)

def logboth(node, msg):
    node.get_logger().info(msg)
    with open(LOGFILE, 'a') as f:
        f.write(msg + '\n')

def distance(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.hypot(dx, dy)

def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

class NavigateToObject(Node):
    def __init__(self):
        super().__init__('navigate_to_object')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_pose = None
        self.latest_amcl_pose = None
        self.goal_seq = 0
        self.object_pose = None  # the real object, for dynamic approach calculation

        # Clean log on startup
        if os.path.exists(LOGFILE):
            os.remove(LOGFILE)

        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.create_timer(2.0, self.status_timer)

        logboth(self, "🟢 Adaptive NavigateToObject node ready.")

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg.pose.pose

    def status_timer(self):
        if self.latest_amcl_pose:
            logboth(self, f"[AMCL] Robot pose: x={self.latest_amcl_pose.position.x:.3f}, y={self.latest_amcl_pose.position.y:.3f}")

    def target_pose_callback(self, msg):
        # Save approach pose and extract object pose from the string in msg.header.frame_id (HACK: see below)
        self.goal_seq += 1
        self.goal_pose = msg
        # Assume the header.frame_id contains object_x,object_y in CSV format if available
        try:
            obj_xy = [float(s) for s in msg.header.frame_id.replace("map:", "").split(",")]
            if len(obj_xy) == 2:
                self.object_pose = (obj_xy[0], obj_xy[1])
                logboth(self, f"🔎 Object pose: x={obj_xy[0]:.3f}, y={obj_xy[1]:.3f}")
        except Exception:
            self.object_pose = None

        logboth(self, f"🚀 Received /target_pose: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, yaw=not_shown")
        self.attempts = 0
        self.try_approach_next()

    def try_approach_next(self):
        # Adaptive: Try with decreasing offset, until succeed or min offset reached
        if self.attempts >= len(APPROACH_OFFSETS):
            logboth(self, f"[❌] All approach distances failed. Robot not able to reach target precisely.")
            return

        # If we have the object position, calculate new approach goal
        if self.object_pose:
            obj_x, obj_y = self.object_pose
            approach_dist = APPROACH_OFFSETS[self.attempts]
            # Vector from robot to object
            robot_x = self.latest_amcl_pose.position.x if self.latest_amcl_pose else 0
            robot_y = self.latest_amcl_pose.position.y if self.latest_amcl_pose else 0
            dx = obj_x - robot_x
            dy = obj_y - robot_y
            d = math.hypot(dx, dy)
            d = max(d, 1e-4)
            goal_x = obj_x - dx/d * approach_dist
            goal_y = obj_y - dy/d * approach_dist
            # Use previous goal's orientation
            approach_pose = copy.deepcopy(self.goal_pose)
            approach_pose.pose.position.x = goal_x
            approach_pose.pose.position.y = goal_y
            logboth(self, f"[ATTEMPT] Approach offset: {approach_dist:.2f}m, Goal: ({goal_x:.3f},{goal_y:.3f})")
        else:
            approach_pose = self.goal_pose
            logboth(self, f"[ATTEMPT] (no object pose info)")

        self.navigate_to_object(approach_pose)

    def navigate_to_object(self, pose_msg):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            logboth(self, "[❌] Nav2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_msg

        logboth(self, f"📤 Sending NavigateToPose goal {self.goal_seq}.{self.attempts+1}: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}")
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            logboth(self, "[❌] Goal was rejected by Nav2!")
            return
        logboth(self, "[✅] Goal accepted by Nav2.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist_rem = fb.distance_remaining if hasattr(fb, 'distance_remaining') else None
        logboth(self, f"[🔄] Feedback: distance_remaining={dist_rem}")

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        status_msg = {
            4: "Goal was aborted (FAILED)",
            3: "Goal was succeeded (SUCCESS)",
            2: "Goal was canceled"
        }.get(status, f"Goal ended with unknown status {status}")
        logboth(self, f"[🏁] Result: {status_msg}")

        if self.latest_amcl_pose and self.goal_pose:
            d = distance(self.latest_amcl_pose.position, self.goal_pose.pose.position)
            logboth(self, f"[RESULT] Final distance to goal: {d:.3f}m")

            # Success if within 7cm of goal, else try next offset
            if status == 3 and d <= 0.07:
                logboth(self, "[✅] Robot is within 7cm of target. Stopping attempts.")
                return
            else:
                logboth(self, f"[ℹ️] Not close enough (d={d:.3f}m). Retrying with closer approach offset.")
                self.attempts += 1
                self.try_approach_next()
        else:
            logboth(self, "[⚠️] No AMCL or goal pose for distance check.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
