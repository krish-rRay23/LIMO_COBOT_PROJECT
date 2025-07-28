import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import subprocess

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pose_sub = self.create_subscription(PoseStamped, '/detected_object_pose', self.pose_callback, 10)

        self.resume_client = self.create_client(Trigger, 'resume_exploration')

        self.object_pose = None
        self.state = 'idle'

        self.get_logger().info('🧠 Mission Manager online.')

    def pose_callback(self, msg):
        if self.state != 'idle':
            return

        self.get_logger().info('📥 Received object pose. Navigating...')
        self.object_pose = msg
        self.state = 'navigating_to_object'
        self.navigate_to_pose(self.object_pose, self.after_navigating_to_object)

    def after_navigating_to_object(self):
        self.get_logger().info('🤖 At object. Running pick_node...')
        try:
            subprocess.run(['ros2', 'run', 'mycobot_arm', 'pick_node'], check=True, timeout=20)
            self.after_pick()
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'❌ pick_node failed: {e}')
            self.state = 'idle'
        except subprocess.TimeoutExpired:
            self.get_logger().error('⏰ pick_node timed out.')
            self.state = 'idle'

    def after_pick(self):
        self.get_logger().info('🏠 Pick complete. Returning to base...')
        base_pose = PoseStamped()
        base_pose.header.frame_id = 'map'
        base_pose.header.stamp = self.get_clock().now().to_msg()
        base_pose.pose.position.x = 1.95
        base_pose.pose.position.y = -9.86
        base_pose.pose.orientation.w = 1.0

        self.state = 'returning_to_base'
        self.navigate_to_pose(base_pose, self.after_returning_to_base)

    def after_returning_to_base(self):
        self.get_logger().info('📦 At base. Running drop_node...')
        try:
            subprocess.run(['ros2', 'run', 'mycobot_arm', 'drop_node'], check=True, timeout=20)
            self.after_drop()
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'❌ drop_node failed: {e}')
            self.state = 'idle'
        except subprocess.TimeoutExpired:
            self.get_logger().error('⏰ drop_node timed out.')
            self.state = 'idle'

    def after_drop(self):
        self.get_logger().info('🔁 Drop complete. Resuming exploration...')
        if not self.resume_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('❌ resume_exploration service not available.')
            self.state = 'idle'
            return

        future = self.resume_client.call_async(Trigger.Request())
        future.add_done_callback(self.after_resume)

    def after_resume(self, fut):
        if fut.result().success:
            self.get_logger().info('✅ Exploration resumed.')
        else:
            self.get_logger().error(f'❌ Failed to resume: {fut.result().message}')
        self.state = 'idle'

    def navigate_to_pose(self, pose, on_done):
        if not self.goal_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 server unavailable.')
            self.state = 'idle'
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f'🚀 Sending goal to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})...')
        future = self.goal_client.send_goal_async(goal)

        def goal_cb(fut):
            handle = fut.result()
            if not handle.accepted:
                self.get_logger().error('❌ Goal rejected.')
                self.state = 'idle'
                return

            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda r: on_done())

        future.add_done_callback(goal_cb)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
