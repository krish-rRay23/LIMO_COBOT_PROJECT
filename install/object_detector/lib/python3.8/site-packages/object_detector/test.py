import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped

class ObjectStopTester(Node):
    def __init__(self):
        super().__init__('object_stop_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.object_detected_callback, 10)
        self.timer = self.create_timer(0.1, self.send_movement)
        self.moving = True
        self.get_logger().info("🚀 Robot will move until object detected...")

    def send_movement(self):
        if self.moving:
            cmd = Twist()
            cmd.linear.x = 0.2  # Adjust speed as needed
            self.cmd_pub.publish(cmd)

    def object_detected_callback(self, msg):
        if self.moving:
            self.get_logger().info("🛑 Object detected. Stopping robot.")
            self.moving = False
            stop = Twist()
            self.cmd_pub.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectStopTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
