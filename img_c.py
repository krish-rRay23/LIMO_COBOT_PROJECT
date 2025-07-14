import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.bridge = CvBridge()
        self.count = 0
        self.folder = "/home/agilex/krish_ws/my_dataset/images"
        os.makedirs(self.folder, exist_ok=True)
        self.get_logger().info("Press 's' to save image, 'q' to quit.")

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Feed", frame)
        key = cv2.waitKey(1)

        if key == ord('s'):
            filename = f"{self.folder}/img{self.count}.jpg"
            cv2.imwrite(filename, frame)
            self.get_logger().info(f"Saved {filename}")
            self.count += 1

        elif key == ord('q'):
            self.get_logger().info("Quitting image capture.")
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ImageCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

