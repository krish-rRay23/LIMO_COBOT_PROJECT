import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
import math
import os
import time
import threading

# Configuration
LOGFILE = 'navigate_to_object_debug.log'
SUCCESS_DISTANCE_THRESHOLD = 0.15  # meters - considered successful if within this distance
MAX_APPROACH_ATTEMPTS = 5
APPROACH_OFFSETS = [0.30, 0.25, 0.20, 0.15, 0.12, 0.10]  # Progressive approach distances
NAVIGATION_TIMEOUT = 60.0  # seconds
GOAL_TOLERANCE = 0.08  # meters - Nav2 goal tolerance

def timestamp():
    """Get current timestamp string"""
    return time.strftime("%H:%M:%S")

def log_message(node, message, also_print=True):
    """Log message to file and optionally print"""
    timestamped_msg = f"[{timestamp()}] {message}"
    
    # Log to file
    try:
        with open(LOGFILE, 'a', encoding='utf-8') as f:
            f.write(timestamped_msg + '\n')
    except Exception as e:
        if also_print:
            print(f"Failed to write to log: {e}")
    
    # Log to ROS
    if node:
        node.get_logger().info(timestamped_msg)
    elif also_print:
        print(timestamped_msg)

def distance_2d(p1, p2):
    """Calculate 2D distance between two points"""
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.hypot(dx, dy)

def yaw_from_quat(q):
    """Convert quaternion to yaw angle in radians"""
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

class RobustNavigationTester(Node):
    def __init__(self):
        super().__init__('robust_navigation_tester')
        
        # Clear log file on startup
        if os.path.exists(LOGFILE):
            os.remove(LOGFILE)
        
        log_message(self, "🚀 Robust Navigation Tester Starting Up")
        
        # State variables
        self.current_goal = None
        self.latest_amcl_pose = None
        self.navigation_start_time = None
        self.attempt_count = 0
        self.goal_sequence = 0
        self.is_navigating = False
        self.last_detection_info = None
        self.detection_active = False
        
        # Navigation client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        
        # Publisher for emergency stop
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(
            PoseStamped, '/target_pose', 
            self.target_pose_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', 
            self.amcl_pose_callback, 10
        )
        self.create_subscription(
            String, '/yolo/detection_info', 
            self.detection_info_callback, 10
        )
        self.create_subscription(
            Bool, '/yolo/detection_active', 
            self.detection_status_callback, 10
        )
        
        # Status timer
        self.create_timer(2.0, self.status_update)
        
        log_message(self, "✅ Robust Navigation Tester initialized successfully")
        log_message(self, f"📊 Configuration: Max attempts: {MAX_APPROACH_ATTEMPTS}, Success threshold: {SUCCESS_DISTANCE_THRESHOLD}m")

    def amcl_pose_callback(self, msg):
        """Update robot pose from AMCL"""
        self.latest_amcl_pose = msg.pose.pose

    def detection_info_callback(self, msg):
        """Store latest detection information"""
        self.last_detection_info = msg.data

    def detection_status_callback(self, msg):
        """Update detection system status"""
        self.detection_active = msg.data

    def status_update(self):
        """Periodic status updates"""
        if not self.latest_amcl_pose:
            log_message(self, "⚠️ Waiting for AMCL pose...")
            return
        
        robot_x = self.latest_amcl_pose.position.x
        robot_y = self.latest_amcl_pose.position.y
        robot_yaw = yaw_from_quat(self.latest_amcl_pose.orientation)
        
        status_parts = [
            f"Robot: ({robot_x:.3f}, {robot_y:.3f}, {math.degrees(robot_yaw):.1f}°)",
            f"Detection: {'🟢' if self.detection_active else '🔴'}",
            f"Navigation: {'🟢' if self.is_navigating else '⚪'}"
        ]
        
        if self.current_goal:
            goal_x = self.current_goal.pose.position.x
            goal_y = self.current_goal.pose.position.y
            distance_to_goal = distance_2d(self.latest_amcl_pose.position, self.current_goal.pose.position)
            status_parts.append(f"Goal: ({goal_x:.3f}, {goal_y:.3f}) dist: {distance_to_goal:.3f}m")
        
        if self.last_detection_info:
            status_parts.append(f"Last detection: {self.last_detection_info}")
        
        log_message(self, " | ".join(status_parts))

    def target_pose_callback(self, msg):
        """Handle new target pose from object detector"""
        if self.is_navigating:
            log_message(self, "⚠️ Already navigating, ignoring new target pose")
            return
        
        self.goal_sequence += 1
        self.attempt_count = 0
        
        log_message(self, f"🎯 NEW TARGET RECEIVED (Sequence #{self.goal_sequence})")
        log_message(self, f"   Target: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})")
        log_message(self, f"   Frame: {msg.header.frame_id}")
        
        if self.latest_amcl_pose:
            initial_distance = distance_2d(self.latest_amcl_pose.position, msg.pose.position)
            log_message(self, f"   Initial distance: {initial_distance:.3f}m")
        
        self.current_goal = msg
        self.attempt_navigation_to_target()

    def attempt_navigation_to_target(self):
        """Attempt navigation with progressive approach distances"""
        if not self.current_goal or not self.latest_amcl_pose:
            log_message(self, "❌ Cannot navigate: missing goal or robot pose")
            return
        
        if self.attempt_count >= MAX_APPROACH_ATTEMPTS:
            log_message(self, f"❌ NAVIGATION FAILED after {MAX_APPROACH_ATTEMPTS} attempts")
            self.reset_navigation_state()
            return
        
        self.attempt_count += 1
        approach_distance = APPROACH_OFFSETS[min(self.attempt_count - 1, len(APPROACH_OFFSETS) - 1)]
        
        # Calculate modified approach position
        goal_pose = self.calculate_approach_pose(self.current_goal, approach_distance)
        
        log_message(self, f"🚀 NAVIGATION ATTEMPT #{self.attempt_count}/{MAX_APPROACH_ATTEMPTS}")
        log_message(self, f"   Approach distance: {approach_distance:.2f}m")
        log_message(self, f"   Target: ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})")
        
        self.execute_navigation(goal_pose)

    def calculate_approach_pose(self, original_goal, approach_distance):
        """Calculate approach pose with specified distance"""
        # Extract original object position from header if available
        try:
            if ":" in original_goal.header.frame_id:
                coords_str = original_goal.header.frame_id.split(":", 1)[1]
                obj_x, obj_y = map(float, coords_str.split(","))
            else:
                # Use original goal position as object position
                obj_x = original_goal.pose.position.x
                obj_y = original_goal.pose.position.y
        except:
            obj_x = original_goal.pose.position.x
            obj_y = original_goal.pose.position.y
        
        # Calculate vector from robot to object
        robot_x = self.latest_amcl_pose.position.x
        robot_y = self.latest_amcl_pose.position.y
        
        dx = obj_x - robot_x
        dy = obj_y - robot_y
        distance = math.hypot(dx, dy)
        
        if distance < 1e-3:
            return original_goal
        
        # Normalize and calculate approach position
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        approach_x = obj_x - dx_norm * approach_distance
        approach_y = obj_y - dy_norm * approach_distance
        
        # Create new goal
        approach_goal = PoseStamped()
        approach_goal.header = original_goal.header
        approach_goal.pose.position.x = approach_x
        approach_goal.pose.position.y = approach_y
        approach_goal.pose.position.z = 0.0
        approach_goal.pose.orientation = original_goal.pose.orientation
        
        return approach_goal

    def execute_navigation(self, goal_pose):
        """Execute navigation to the specified goal"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            log_message(self, "❌ Navigation action server not available!")
            self.reset_navigation_state()
            return
        
        # Create navigation goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose
        
        # Send goal
        log_message(self, "📤 Sending navigation goal to Nav2...")
        self.navigation_start_time = time.time()
        self.is_navigating = True
        
        send_goal_future = self.nav_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            log_message(self, "❌ Navigation goal REJECTED by Nav2")
            self.reset_navigation_state()
            return
        
        log_message(self, "✅ Navigation goal ACCEPTED by Nav2")
        self.current_goal_handle = goal_handle
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        
        if hasattr(feedback, 'distance_remaining'):
            distance_remaining = feedback.distance_remaining
            elapsed_time = time.time() - self.navigation_start_time if self.navigation_start_time else 0
            
            log_message(self, f"📍 Navigation feedback: {distance_remaining:.2f}m remaining, {elapsed_time:.1f}s elapsed")
            
            # Check for timeout
            if elapsed_time > NAVIGATION_TIMEOUT:
                log_message(self, "⏰ Navigation TIMEOUT - cancelling goal")
                self.cancel_current_goal()

    def navigation_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result()
        status = result.status
        
        elapsed_time = time.time() - self.navigation_start_time if self.navigation_start_time else 0
        
        status_messages = {
            4: "ABORTED",
            3: "SUCCEEDED", 
            2: "CANCELED",
            1: "EXECUTING"
        }
        status_msg = status_messages.get(status, f"UNKNOWN({status})")
        
        log_message(self, f"🏁 Navigation result: {status_msg} after {elapsed_time:.1f}s")
        
        if not self.latest_amcl_pose or not self.current_goal:
            log_message(self, "⚠️ Cannot evaluate result: missing pose data")
            self.reset_navigation_state()
            return
        
        # Check final distance to goal
        final_distance = distance_2d(
            self.latest_amcl_pose.position, 
            self.current_goal.pose.position
        )
        
        log_message(self, f"📏 Final distance to goal: {final_distance:.3f}m")
        
        # Evaluate success
        if status == 3 and final_distance <= SUCCESS_DISTANCE_THRESHOLD:
            log_message(self, f"🎉 NAVIGATION SUCCESS! Robot within {final_distance:.3f}m of target")
            log_message(self, f"📊 Total attempts: {self.attempt_count}, Total time: {elapsed_time:.1f}s")
            self.reset_navigation_state()
        
        elif final_distance <= SUCCESS_DISTANCE_THRESHOLD:
            log_message(self, f"🎉 NAVIGATION SUCCESS! Robot close enough despite status {status_msg}")
            self.reset_navigation_state()
        
        else:
            log_message(self, f"❌ Navigation incomplete: {final_distance:.3f}m > {SUCCESS_DISTANCE_THRESHOLD:.3f}m threshold")
            
            # Try again with closer approach
            self.is_navigating = False
            threading.Timer(2.0, self.attempt_navigation_to_target).start()

    def cancel_current_goal(self):
        """Cancel the current navigation goal"""
        if self.current_goal_handle:
            log_message(self, "🛑 Cancelling current navigation goal")
            self.current_goal_handle.cancel_goal_async()
        
        # Emergency stop
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        self.reset_navigation_state()

    def reset_navigation_state(self):
        """Reset navigation state variables"""
        self.is_navigating = False
        self.current_goal_handle = None
        self.navigation_start_time = None
        log_message(self, "🔄 Navigation state reset")

def main(args=None):
    rclpy.init(args=args)
    node = RobustNavigationTester()
    
    try:
        log_message(node, "🟢 Starting navigation tester - waiting for object detections...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_message(node, "🛑 Keyboard interrupt - shutting down")
    finally:
        log_message(node, "👋 Navigation tester shutting down")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
