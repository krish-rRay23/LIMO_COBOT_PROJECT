#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading


class ObjectExplorer(Node):
    def __init__(self):
        super().__init__('object_explorer')
        self.get_logger().info("🧭 ObjectExplorer initialized.")

        self.declare_parameter('search_area', [-2.0, 2.0, -2.0, 2.0])  # [xmin, xmax, ymin, ymax]
        self.declare_parameter('goal_interval_sec', 10.0)

        self.stop_search = False
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.detection_sub = self.create_subscription(
            String,
            '/object_found',  # Now listens to actual YOLOv8 3D output
            self.detected_callback,
            10
        )

        self.goal_timer = self.create_timer(
            self.get_parameter('goal_interval_sec').value,
            self.send_random_goal
        )

        self.get_logger().info("📡 Explorer waiting for Nav2 server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("✅ Nav2 server ready. Exploration can begin.")

    def detected_callback(self, msg: String):
        if self.stop_search:
            return

        if msg.data.startswith("cube"):
            self.get_logger().info(f"🎯 Object detected: {msg.data}")
            self.stop_search = True
            self.goal_timer.cancel()

            # Optionally: extract and log 3D position
            try:
                label, px, py, coords = msg.data.split(":")
                x, y, z = map(float, coords.split(","))
                self.get_logger().info(f"🧠 Object 3D Position → x={x:.2f}, y={y:.2f}, z={z:.2f}")
            except:
                self.get_logger().warn("⚠️ Could not parse detection string.")

    def send_random_goal(self):
        if self.stop_search:
            return

        bounds = self.get_parameter('search_area').value
        x = round(random.uniform(bounds[0], bounds[1]), 2)
        y = round(random.uniform(bounds[2], bounds[3]), 2)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Simple facing forward

        self.get_logger().info(f"🚀 Sending exploration goal: ({x}, {y})")
        self.nav_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
