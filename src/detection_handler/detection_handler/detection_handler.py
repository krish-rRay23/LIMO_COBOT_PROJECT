#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class ObjectExplorer(Node):
    def __init__(self):
        super().__init__('object_explorer')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.detection_sub = self.create_subscription(
            String,
            '/detections',  # replace with your YOLO detection topic
            self.detected_callback,
            10
        )
        self.declare_parameter('search_area', [-2.0, 2.0, -2.0, 2.0])  # [xmin, xmax, ymin, ymax]
        self.timer = self.create_timer(10.0, self.send_random_goal)
        self.stop_search = False

    def detected_callback(self, msg):
        if "cube" in msg.data:
            self.get_logger().info("🎯 Object detected! Stopping exploration.")
            self.stop_search = True
            self.timer.cancel()

    def send_random_goal(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available.')
            return

        if self.stop_search:
            return

        bounds = self.get_parameter('search_area').value
        x = random.uniform(bounds[0], bounds[1])
        y = random.uniform(bounds[2], bounds[3])

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0  # facing forward

        self.get_logger().info(f"🚀 Sending goal: x={x:.2f}, y={y:.2f}")
        self.client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

