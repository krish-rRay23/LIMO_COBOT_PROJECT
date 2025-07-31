import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion, PoseWithCovarianceStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import subprocess
import threading
import time
import math
import tf2_ros
import tf2_geometry_msgs
from message_filters import ApproximateTimeSynchronizer, Subscriber

# ======= USER SETTINGS =======
DEPTH_TOPIC = '/camera/depth/image_raw'
MIN_DEPTH_METERS = 0.15
MAX_DEPTH_METERS = 4.0
APPROACH_DIST = 0.30  # How far to stop before object
# =============================

def yaw_from_quat(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class YoloNavDetector(Node):
    def __init__(self):
        super().__init__('yolo_nav_detector')

        self.bridge = CvBridge()
        self.model = YOLO("/home/agilex/krish_ws/runs/detect/mycobot_final2/weights/best.pt")

        # message_filters subscribers
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, DEPTH_TOPIC)
        self.info_sub = Subscriber(self, CameraInfo, '/camera/color/camera_info')

        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1  # Adjust if needed
        )
        self.ts.registerCallback(self.synced_callback)

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)

        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated', 10)
        self.result_pub = self.create_publisher(String, '/object_found', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.latest_amcl_pose = None

        self.fx = self.fy = self.cx = self.cy = None
        self.intrinsics_received = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.audio_lock = threading.Lock()
        self.last_spoken_time = 0
        self.audio_cooldown = 5

        self.get_logger().info("✅ YOLO detector node initialized with synchronized inputs.")

    def amcl_pose_callback(self, msg):
        self.latest_amcl_pose = msg.pose.pose

    def speak_once(self, text):
        def _speak():
            with self.audio_lock:
                subprocess.call(f"espeak '{text}' --stdout | aplay -D plughw:1,3", shell=True)
        now = time.time()
        if now - self.last_spoken_time > self.audio_cooldown:
            self.last_spoken_time = now
            threading.Thread(target=_speak, daemon=True).start()

    def get_3d_point(self, depth_img, cx, cy):
        points = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                x, y = cx + dx, cy + dy
                if 0 <= x < depth_img.shape[1] and 0 <= y < depth_img.shape[0]:
                    z = depth_img[y, x] / 1000.0  # mm to m
                    if MIN_DEPTH_METERS < z < MAX_DEPTH_METERS:
                        X = (x - self.cx) * z / self.fx
                        Y = (y - self.cy) * z / self.fy
                        points.append((X, Y, z))
        if not points:
            return None
        return tuple(np.median(points, axis=0))

    def transform_to_map(self, point, stamp, src_frame):
        p = PointStamped()
        p.header.stamp = stamp
        p.header.frame_id = src_frame
        p.point.x, p.point.y, p.point.z = point

        try:
            return self.tf_buffer.transform(p, "map", timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e1:
            self.get_logger().warn(f"[⚠️ TF TIME WARN] {e1} — falling back to 'now'")
            # Try again with current time (safe fallback)
            p.header.stamp = rclpy.time.Time().to_msg()
            try:
                return self.tf_buffer.transform(p, "map", timeout=rclpy.duration.Duration(seconds=0.5))
            except Exception as e2:
                self.get_logger().error(f"[❌ TF FAIL] Even 'now' failed: {e2}")
                return None

    def synced_callback(self, rgb_msg, depth_msg, cam_info_msg):
        self.fx, self.fy = cam_info_msg.k[0], cam_info_msg.k[4]
        self.cx, self.cy = cam_info_msg.k[2], cam_info_msg.k[5]
        self.intrinsics_received = True

        if not self.latest_amcl_pose:
            return

        color_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        results = self.model.predict(source=color_img, conf=0.5, verbose=False)

        if not results or not results[0].boxes:
            return

        boxes = sorted(results[0].boxes, key=lambda b: b.xyxy[0][3]-b.xyxy[0][1])
        best_box = boxes[0]
        cls_id = int(best_box.cls)
        cls_name = self.model.names[cls_id]
        x1, y1, x2, y2 = map(int, best_box.xyxy[0].tolist())
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        self.get_logger().info(f"[DEBUG] Pixel ({cx},{cy}) depth={depth_img[cy, cx]}mm")

        point_3d = self.get_3d_point(depth_img, cx, cy)
        if not point_3d:
            self.get_logger().warn(f"[SKIP] No valid depth at ({cx},{cy})")
            return

        x_cam, y_cam, z_cam = point_3d
        self.get_logger().info(f"[DEBUG] 3D cam: ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f})")

        # Use exact frame from camera header (and trim whitespace for safety!)
        src_frame = depth_msg.header.frame_id.strip()
        tf_point = self.transform_to_map(point_3d, depth_msg.header.stamp, src_frame)
        if not tf_point:
            return

        # Compute approach position
        CAMERA_TO_BASE_LINK = 0.12  # meters, adjust if your real offset is different
        amcl_x = self.latest_amcl_pose.position.x
        amcl_y = self.latest_amcl_pose.position.y
        obj_x, obj_y = tf_point.point.x, tf_point.point.y
        
        dx = obj_x - amcl_x
        dy = obj_y - amcl_y
        d = math.hypot(dx, dy)
        d = max(d, 1e-3)
        
        # Approach exactly at the object: center stops at (object - camera_to_base_link)
        approach_x = obj_x - (dx / d) * CAMERA_TO_BASE_LINK
        approach_y = obj_y - (dy / d) * CAMERA_TO_BASE_LINK
        

        goal_yaw = math.atan2(obj_y - approach_y, obj_x - approach_x)
        q = quat_from_yaw(goal_yaw)

        robot_yaw = yaw_from_quat(self.latest_amcl_pose.orientation)
        self.get_logger().info(
            f"[AMCL] Robot: x={amcl_x:.2f}, y={amcl_y:.2f}, yaw={math.degrees(robot_yaw):.1f}°"
        )
        self.get_logger().info(
            f"[OBJECT] map: x={obj_x:.2f}, y={obj_y:.2f}, z={tf_point.point.z:.2f}"
        )
        self.get_logger().info(
            f"[APPROACH] map: x={approach_x:.2f}, y={approach_y:.2f}, yaw={math.degrees(goal_yaw):.1f}°"
        )
        robot_pose_str = f"AMCL: x={amcl_x:.3f}, y={amcl_y:.3f}, yaw={math.degrees(robot_yaw):.2f}°"
        object_pose_str = f"OBJ: x={obj_x:.3f}, y={obj_y:.3f}, z={tf_point.point.z:.3f}"
        approach_pose_str = f"APPROACH: x={approach_x:.3f}, y={approach_y:.3f}, yaw={math.degrees(goal_yaw):.2f}°"
        dist_robot_object = math.hypot(obj_x - amcl_x, obj_y - amcl_y)
        dist_robot_approach = math.hypot(approach_x - amcl_x, approach_y - amcl_y)
        self.get_logger().info(
            f"[DETECT LOG]\n  {robot_pose_str}\n  {object_pose_str}\n  {approach_pose_str}\n  "
            f"dist(robot→object)={dist_robot_object:.3f}m, dist(robot→approach)={dist_robot_approach:.3f}m"
        )
        self.get_logger().info(
            f"[DEBUG] amcl_pose=({amcl_x:.3f},{amcl_y:.3f}) "
            f"object_map=({obj_x:.3f},{obj_y:.3f}) "
            f"approach=({approach_x:.3f},{approach_y:.3f}) "
            f"dist_to_object={math.hypot(obj_x-amcl_x,obj_y-amcl_y):.3f}m "
            f"dist_to_approach={math.hypot(approach_x-amcl_x,approach_y-amcl_y):.3f}m "
            f"depth_pixel={depth_img[cy,cx]}mm"
        )

        # Publish navigation goal
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = f"map:{obj_x:.3f},{obj_y:.3f}"
        pose_msg.header.stamp = rgb_msg.header.stamp
        pose_msg.pose.position.x = approach_x
        pose_msg.pose.position.y = approach_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = q
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"[🌍 POSE] Published /target_pose → map({approach_x:.2f}, {approach_y:.2f}), yaw={math.degrees(goal_yaw):.1f}°"
        )

        self.result_pub.publish(String(data=f"{cls_name}:{cx}:{cy}:{x_cam:.2f},{y_cam:.2f},{z_cam:.2f}"))
        self.get_logger().info(f"[📍 DETECTED] {cls_name} 2D({cx},{cy}) 3D({x_cam:.2f},{y_cam:.2f},{z_cam:.2f})")
        self.speak_once("Object detected.")

        annotated = results[0].plot()
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloNavDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
