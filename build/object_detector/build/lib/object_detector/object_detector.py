import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import subprocess
import threading
import time
import tf2_ros
import tf2_geometry_msgs

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
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.latest_depth_image = None
        self.intrinsics_received = False
        self.fx = self.fy = self.cx = self.cy = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("✅ YOLOv8 detector node initialized.")

        self.last_spoken_time = 0
        self.audio_cooldown = 5
        self.audio_lock = threading.Lock()

    def camera_info_callback(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.intrinsics_received = True
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None
        self.get_logger().info("🎯 Camera intrinsics captured.")

    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def speak_object_detected(self):
        def speak():
            with self.audio_lock:
                subprocess.call("espeak 'Object detected' --stdout | aplay -D plughw:1,3", shell=True)

        now = time.time()
        if now - self.last_spoken_time > self.audio_cooldown:
            self.last_spoken_time = now
            threading.Thread(target=speak, daemon=True).start()

    def get_3d_point_from_depth_image(self, depth_img, cx, cy):
        try:
            points = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < depth_img.shape[1] and 0 <= y < depth_img.shape[0]:
                        z = depth_img[y, x] / 1000.0
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

    def transform_to_map(self, point):
        shifted = PointStamped()
        shifted.header.stamp = self.get_clock().now().to_msg()
        shifted.header.frame_id = "camera_link"
        shifted.point.x = point[0]
        shifted.point.y = point[1]
        shifted.point.z = point[2] + 0.3

        try:
            tf_result = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
            self.get_logger().info(f"[TF OK] Using transform map ← camera_link")
            shifted.header.stamp = tf_result.header.stamp  # ✅ align stamp to available TF
            tf_point = self.tf_buffer.transform(shifted, "map", timeout=rclpy.duration.Duration(seconds=1.0))
            return tf_point
        except Exception as e:
            self.get_logger().warn(f"[❌ TF FAIL] {e}")
            return None

    def image_callback(self, msg):
        if not self.intrinsics_received or self.latest_depth_image is None:
            return

        color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(self.latest_depth_image, "16UC1")
        results = self.model.predict(source=color_img, conf=0.5, verbose=False)

        if results and results[0].boxes:
            self.speak_object_detected()
            for box in results[0].boxes:
                cls_id = int(box.cls)
                cls_name = self.model.names[cls_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                point_3d = self.get_3d_point_from_depth_image(depth_img, cx, cy)
                if point_3d:
                    x, y, z = point_3d

                    # Publish legacy string message
                    msg_data = f"{cls_name}:{cx}:{cy}:{x:.2f},{y:.2f},{z:.2f}"
                    self.result_pub.publish(String(data=msg_data))
                    self.get_logger().info(f"[📍 DETECTED] {cls_name} → 2D({cx},{cy}) → 3D({x:.2f},{y:.2f},{z:.2f})")

                    # Try to publish PoseStamped in map frame
                    tf_point = self.transform_to_map(point_3d)
                    if tf_point:
                        pose_msg = PoseStamped()
                        pose_msg.header = tf_point.header
                        pose_msg.pose.position = tf_point.point
                        pose_msg.pose.orientation.w = 1.0  # No rotation
                        self.pose_pub.publish(pose_msg)
                        self.get_logger().info(f"[🌍 POSE] Published /target_pose → map({tf_point.point.x:.2f}, {tf_point.point.y:.2f}, {tf_point.point.z:.2f})")

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
