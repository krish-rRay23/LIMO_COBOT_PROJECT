import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()
        self.model = YOLO("/home/agilex/krish_ws/runs/detect/mycobot_final2/weights/best.pt")

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.result_pub = self.create_publisher(String, '/object_found', 10)

        self.latest_depth_image = None
        self.intrinsics_received = False
        self.fx = self.fy = self.cx = self.cy = None

        self.get_logger().info("✅ YOLOv8 detector node initialized.")

    def camera_info_callback(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.intrinsics_received = True
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None
        self.get_logger().info("🎯 Camera intrinsics captured.")
    
    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def get_3d_point_from_depth_image(self, depth_img, cx, cy):
        try:
            points = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < depth_img.shape[1] and 0 <= y < depth_img.shape[0]:
                        z = depth_img[y, x] / 1000.0  # mm to m
                        if z > 0:
                            X = (x - self.cx) * z / self.fx
                            Y = (y - self.cy) * z / self.fy
                            points.append((X, Y, z))
            if not points:
                return None
            x_avg = sum(p[0] for p in points) / len(points)
            y_avg = sum(p[1] for p in points) / len(points)
            z_avg = sum(p[2] for p in points) / len(points)
            return (x_avg, y_avg, z_avg)
        except:
            return None

    def image_callback(self, msg):
        if not self.intrinsics_received or self.latest_depth_image is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth_image, "16UC1")

        results = self.model.predict(source=color_img, conf=0.5, verbose=False)

        if results and results[0].boxes:
            for box in results[0].boxes:
                cls_id = int(box.cls)
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                point_3d = self.get_3d_point_from_depth_image(depth_img, cx, cy)
                if point_3d:
                    x, y, z = point_3d
                    msg_data = f"{cls_name}:{cx}:{cy}:{x:.2f},{y:.2f},{z:.2f}"
                    self.result_pub.publish(String(data=msg_data))
                    self.get_logger().info(f"[📍 DETECTED] {cls_name} → 2D({cx},{cy}) → 3D({x:.2f},{y:.2f},{z:.2f})")

        # Always show image with or without detections
        annotated = results[0].plot()
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
