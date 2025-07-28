import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math
import threading

class ExplorationHandler(Node):
    def __init__(self):
        super().__init__('exploration_handler')

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.object_sub = self.create_subscription(Bool, '/object_found', self.object_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.resume_srv = self.create_service(Trigger, 'resume_exploration', self.handle_resume)

        flat_default = [2.3, -9.0, 90.0, 2.8, -8.5, 90.0]
        self.declare_parameter('waypoints_flat', flat_default)
        flat_list = self.get_parameter('waypoints_flat').value
        self.waypoints = [flat_list[i:i+3] for i in range(0, len(flat_list), 3)]

        self.current_index = 0
        self.goal_active = False
        self.state_lock = threading.Lock()
        self.state = 'exploring'

        self.timer = self.create_timer(1.0, self.run_loop)
        self.get_logger().info('🔍 ExplorationHandler ready.')

    def object_callback(self, msg):
        if not msg.data:
            return

        with self.state_lock:
            if self.state != 'exploring':
                self.get_logger().info('⚠️ Already handling object.')
                return

            self.state = 'handling_object'
            self.goal_active = False

        self.get_logger().info('📍 Object detected. Publishing pose and pausing exploration.')

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = -0.34  # Replace with actual detection position later
        pose.pose.position.y = 0.18
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

    def handle_resume(self, request, response):
        with self.state_lock:
            self.state = 'exploring'
            self.goal_active = False
            self.current_index += 1

        response.success = True
        response.message = 'Exploration resumed.'
        return response

    def run_loop(self):
        with self.state_lock:
            if self.state != 'exploring' or self.goal_active:
                return

        if self.current_index >= len(self.waypoints):
            self.get_logger().info('✅ Finished all waypoints.')
            self.timer.cancel()
            return

        self.send_goal(self.waypoints[self.current_index])

    def send_goal(self, waypoint):
        x, y, yaw = waypoint
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
        pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        self.goal_active = True

        self.get_logger().info(f'🚀 Navigating to waypoint: ({x:.2f}, {y:.2f})')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal was rejected.')
            self.goal_active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().status
        if result == 0:
            self.get_logger().info('🏁 Waypoint reached.')
        else:
            self.get_logger().warn(f'⚠️ Goal failed with status: {result}')

        with self.state_lock:
            if self.state == 'handling_object':
                self.get_logger().info('⏸ Waiting for mission_manager to finish object handling.')
            else:
                self.current_index += 1
            self.goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
