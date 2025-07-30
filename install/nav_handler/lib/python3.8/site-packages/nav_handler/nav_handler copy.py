import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time


def yaw_to_quaternion(yaw_rad):
    """Convert yaw in radians to quaternion (x, y, z, w)"""
    qz = math.sin(yaw_rad / 2.0)
    qw = math.cos(yaw_rad / 2.0)
    return (0.0, 0.0, qz, qw)


class PoseNavReturn(Node):
    def __init__(self):
        super().__init__('pose_nav_return')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.init_pose = (-0.781, -0.228, 7.9)            # Initial pose to set
        self.goals = [
            (0.15, -0.62, -29.2),                         # First goal
            self.init_pose                                # Return to initial
        ]
        self.goal_index = 0
        self.pose_set = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if not self.pose_set:
            self.publish_initial_pose()
            self.pose_set = True
            self.get_logger().info("✅ Initial pose set. Waiting before sending goal...")
            time.sleep(3.0)
            self.send_next_goal()

    def publish_initial_pose(self):
        x, y, yaw_deg = self.init_pose
        yaw_rad = math.radians(yaw_deg)
        _, _, qz, qw = yaw_to_quaternion(yaw_rad)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.pose_pub.publish(msg)
        self.get_logger().info(f"📍 Initial pose published at ({x}, {y}, {yaw_deg}°)")

    def send_next_goal(self):
        if self.goal_index >= len(self.goals):
            self.get_logger().info("🏁 All navigation goals completed.")
            rclpy.shutdown()
            return

        x, y, yaw_deg = self.goals[self.goal_index]
        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"🚀 Sending goal {self.goal_index + 1}: ({x}, {y}, {yaw_deg}°)")
        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected.')
            return

        self.get_logger().info('✅ Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎯 Goal {self.goal_index + 1} reached.')
        self.goal_index += 1
        time.sleep(2.0)
        self.send_next_goal()


def main():
    rclpy.init()
    node = PoseNavReturn()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
