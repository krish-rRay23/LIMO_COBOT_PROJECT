import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Quaternion
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import subprocess
import threading, time
import math
import tf2_ros
import tf2_geometry_msgs

# ======= USER SETTINGS =======
DEPTH_TOPIC = '/camera/depth/image_raw'
MIN_DEPTH_METERS = 0.15
MAX_DEPTH_METERS = 4.0
APPROACH_DIST = 0.30      # Approach distance (meters, stop this far before object)
TF_USE_NOW = True         # Use current time for TF lookup to avoid extrapolation errors
# =============================

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.bridge = CvBridge()
        self.model = YOLO("/home/agilex/krish_ws/runs/detect/mycobot_final2/weights/best.pt")

        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.result_pub = self.create_publisher(String, '/object_found', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.latest_depth_image = None
        self.intrinsics_received = False
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_amcl_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"✅ YOLOv8 detector node initialized. Using depth topic: {DEPTH_TOPIC}")

        self.last_spoken_time = 0
        self.audio_cooldown = 5
        self.audio_lock = threading.Lock()

    def camera_info_callback(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.get_logger().info(f"[INTRINSICS] fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
        self.intrinsics_received = True
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None
        self.get_logger().info("🎯 Camera intrinsics captured.")

    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg.pose.pose

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
            # Gather 3x3 region to smooth noisy depth
            points = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    x, y = cx + dx, cy + dy
                    if 0 <= x < depth_img.shape[1] and 0 <= y < depth_img.shape[0]:
                        z = depth_img[y, x] / 1000.0  # mm to meters
                        if z > MIN_DEPTH_METERS and z < MAX_DEPTH_METERS:
                            X = (x - self.cx) * z / self.fx
                            Y = (y - self.cy) * z / self.fy
                            points.append((X, Y, z))
            if not points:
                return None
            x_avg = sum(p[0] for p in points) / len(points)
            y_avg = sum(p[1] for p in points) / len(points)
            z_avg = sum(p[2] for p in points) / len(points)
            return (x_avg, y_avg, z_avg)
        except Exception as e:
            self.get_logger().warn(f"Depth-to-3D error: {e}")
            return None

    def transform_to_map(self, point, image_msg):
        shifted = PointStamped()
        shifted.header.stamp = rclpy.time.Time().to_msg()
        shifted.header.frame_id = "camera_link"
        shifted.point.x = point[0]
        shifted.point.y = point[1]
        shifted.point.z = point[2]
        try:
            tf_point = self.tf_buffer.transform(shifted, "map", timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(
                f"[TF OK] cam:({point[0]:.2f},{point[1]:.2f},{point[2]:.2f}) → map:({tf_point.point.x:.2f},{tf_point.point.y:.2f},{tf_point.point.z:.2f})"
            )
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
                depth_value = depth_img[cy, cx]
                self.get_logger().info(f"[DEBUG] Pixel ({cx},{cy}) depth={depth_value}mm ({depth_value/1000.0:.2f}m)")

                point_3d = self.get_3d_point_from_depth_image(depth_img, cx, cy)
                if not point_3d:
                    self.get_logger().warn(f"[SKIP] No valid depth at ({cx},{cy}); skipping detection.")
                    continue

                x, y, z = point_3d
                self.get_logger().info(f"[DEBUG] 3D cam: ({x:.2f}, {y:.2f}, {z:.2f})")

                tf_point = self.transform_to_map(point_3d, msg)
                if not tf_point:
                    continue

                if not self.latest_amcl_pose:
                    self.get_logger().warn(f"[SKIP] No AMCL pose available, can't compute approach.")
                    continue

                # --- Compute approach point ---
                dx = tf_point.point.x - self.latest_amcl_pose.position.x
                dy = tf_point.point.y - self.latest_amcl_pose.position.y
                d = math.hypot(dx, dy)
                if d < 1e-6:
                    d = 1e-6

                approach_x = tf_point.point.x - (dx / d) * APPROACH_DIST
                approach_y = tf_point.point.y - (dy / d) * APPROACH_DIST

                # --- Compute orientation: from approach point to object ---
                goal_yaw = math.atan2(tf_point.point.y - approach_y, tf_point.point.x - approach_x)
                q = Quaternion()
                q.z = math.sin(goal_yaw / 2.0)
                q.w = math.cos(goal_yaw / 2.0)

                # --- Build and publish target pose ---
                pose_msg = PoseStamped()
                pose_msg.header = tf_point.header
                pose_msg.pose.position.x = approach_x
                pose_msg.pose.position.y = approach_y
                pose_msg.pose.position.z = tf_point.point.z
                pose_msg.pose.orientation = q

                self.pose_pub.publish(pose_msg)
                self.get_logger().info(
                    f"[🌍 POSE] Published /target_pose → map({approach_x:.2f}, {approach_y:.2f}, {tf_point.point.z:.2f}), yaw={math.degrees(goal_yaw):.1f}°"
                )

                # Publish detection info for legacy
                msg_data = f"{cls_name}:{cx}:{cy}:{x:.2f},{y:.2f},{z:.2f}"
                self.result_pub.publish(String(data=msg_data))
                self.get_logger().info(f"[📍 DETECTED] {cls_name} 2D({cx},{cy}) 3D({x:.2f},{y:.2f},{z:.2f})")

        if results:
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
