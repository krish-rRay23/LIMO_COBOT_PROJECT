import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import math

# Experimentally determined: real_distance / published_distance
SCALE_VECTOR = 2.96    # (e.g., 83cm / 28cm)
APPROACH_DIST = 0.30   # Robot will stop this far before object (meters)

def compute_yaw_to_target(from_pose, to_pose):
    dx = to_pose.position.x - from_pose.position.x
    dy = to_pose.position.y - from_pose.position.y
    return math.atan2(dy, dx)

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def euclidean_distance_2d(p1, p2):
    dx = p1.position.x - p2.position.x
    dy = p1.position.y - p2.position.y
    return math.hypot(dx, dy)

def pose_to_yaw_deg(pose):
    q = pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)

class GotoObjectScaledDebug(Node):
    def __init__(self):
        super().__init__('goto_object_scaled_debug')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_amcl_pose = None
        self.has_goal = False

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        self.get_logger().info("🟢 GotoObjectScaledDebug Node started.")
        self.get_logger().info(f"   - Using SCALE_VECTOR: {SCALE_VECTOR:.2f} (adjust if needed)")
        self.get_logger().info(f"   - Will stop {APPROACH_DIST*100:.0f}cm before object, facing it.")

    def amcl_pose_callback(self, msg):
        self.current_amcl_pose = msg.pose.pose
        self.get_logger().info(
            f"[AMCL] Robot pose: x={self.current_amcl_pose.position.x:.3f}, y={self.current_amcl_pose.position.y:.3f}, yaw={pose_to_yaw_deg(self.current_amcl_pose):.1f}°"
        )

    def target_pose_callback(self, msg):
        if self.has_goal:
            return  # Only use the first detection
        if self.current_amcl_pose is None:
            self.get_logger().warn("[WARN] No AMCL pose received yet, can't navigate.")
            return

        orig = msg.pose
        rob = self.current_amcl_pose

        # Vector from robot to original YOLO target
        dx = orig.position.x - rob.position.x
        dy = orig.position.y - rob.position.y
        raw_dist = math.hypot(dx, dy)

        # Apply experimental scaling correction
        dx_scaled = dx * SCALE_VECTOR
        dy_scaled = dy * SCALE_VECTOR
        scaled_x = rob.position.x + dx_scaled
        scaled_y = rob.position.y + dy_scaled
        scaled_dist = math.hypot(dx_scaled, dy_scaled)

        # Approach point: stop APPROACH_DIST before the object
        if scaled_dist > APPROACH_DIST:
            scale = (scaled_dist - APPROACH_DIST) / scaled_dist
            ax = rob.position.x + dx_scaled * scale
            ay = rob.position.y + dy_scaled * scale
        else:
            ax, ay = scaled_x, scaled_y

        yaw = math.atan2(dy_scaled, dx_scaled)

        approach_pose = PoseStamped()
        approach_pose.header = msg.header
        approach_pose.pose.position.x = ax
        approach_pose.pose.position.y = ay
        approach_pose.pose.position.z = orig.position.z
        approach_pose.pose.orientation = yaw_to_quaternion(yaw)

        # LOG EVERYTHING
        self.get_logger().info(f"[DETECT] Raw /target_pose: x={orig.position.x:.3f}, y={orig.position.y:.3f}")
        self.get_logger().info(f"[VEC] Robot→Target vector: dx={dx:.3f}, dy={dy:.3f} | Raw dist={raw_dist:.2f}m")
        self.get_logger().info(f"[SCALED] Scaled goal: x={scaled_x:.3f}, y={scaled_y:.3f} | Scaled dist={scaled_dist:.2f}m")
        self.get_logger().info(f"[APPROACH] Approach goal: x={ax:.3f}, y={ay:.3f}, yaw={math.degrees(yaw):.1f}°")
        self.get_logger().info(f"[INFO] Will stop {APPROACH_DIST*100:.0f}cm before object.")

        self.send_nav_goal(approach_pose)
        self.has_goal = True

    def send_nav_goal(self, pose_stamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        if self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("[NAV] Nav2 server ready, sending goal...")
            send_goal_future = self.nav_client.send_goal_async(goal)
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().warn("[NAV] Nav2 server not available!")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("[NAV] Goal rejected by Nav2 server.")
            return
        self.get_logger().info("[NAV] Goal accepted! Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("[NAV] Arrived at the approach point! Nav2 result: %s" % result)
        self.get_logger().info("[END] Done, shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GotoObjectScaledDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
