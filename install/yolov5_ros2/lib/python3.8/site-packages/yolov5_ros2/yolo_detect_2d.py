from math import frexp
from traceback import print_tb
from torch import imag
from yolov5 import YOLOv5
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor

from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose, Detection2D
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

from yolov5_ros2.cv_tool import px2xy

package_share_directory = get_package_share_directory('yolov5_ros2')
# package_share_directory = "/home/mouse/code/github/yolov5_test/src/yolov5_ros2"


class YoloV5Ros2(Node):
    def __init__(self):
        super().__init__('yolov5_ros2')
        self.declare_parameter("device", "cuda", ParameterDescriptor(
            name="device", description="calculate_device default:cpu optional:cuda:0"))

        self.declare_parameter("model", f"{package_share_directory}/config/yolov5m.pt", ParameterDescriptor(
            name="model", description=f"default: {package_share_directory}/config/yolov5m.pt"))

        self.declare_parameter("image_topic", "/camera/color/image_raw", ParameterDescriptor(
            name="image_topic", description=f"default: /camera/color/image_raw"))

        self.declare_parameter("camera_info_topic", "/camera/color/camera_info", ParameterDescriptor(
            name="camera_info_topic", description=f"default: /camera/color/camera_info"))

        # 默认从camera_info中读取参数,如果可以从话题接收到参数则覆盖文件中的参数
        self.declare_parameter("camera_info_file", f"{package_share_directory}/config/camera_info.yaml", ParameterDescriptor(
            name="camera_info", description=f"{package_share_directory}/config/camera_info.yaml"))

        # 默认显示识别结果
        self.declare_parameter("show_result", True, ParameterDescriptor(
            name="show_result", description=f"default: True"))
        
        # 添加置信度阈值参数
        self.declare_parameter("confidence_threshold", 0.25, ParameterDescriptor(
            name="confidence_threshold", description=f"default: 0.25"))

        # 1.load model
        model = self.get_parameter('model').value
        device = self.get_parameter('device').value
        confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.yolov5 = YOLOv5(model_path=model, device=device)
        # Set confidence threshold
        self.yolov5.model.conf = confidence_threshold
        self.get_logger().info(f"YOLOv5 model loaded with confidence threshold: {confidence_threshold}")

        # 2.create publisher
        self.yolo_result_pub = self.create_publisher(
            Detection2DArray, "yolo_result", 10)
        self.result_msg = Detection2DArray()

        # 3.create sub image (if 3d, sub depth, if 2d load camera info)
        image_topic = self.get_parameter('image_topic').value
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, rclpy.qos.qos_profile_sensor_data)

        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 1)

        # get camera info - Initialize with default values
        self.camera_info = {'k': [], 'd': [], 'p': [], 'r': [], 'roi': None}
        self.camera_info_received = False
        
        # Try to load camera info from file as fallback
        try:
            with open(self.get_parameter('camera_info_file').value) as f:
                file_camera_info = yaml.full_load(f.read())
                if file_camera_info:
                    self.camera_info.update(file_camera_info)
                    self.get_logger().info(f"Loaded camera info from file: {self.camera_info['k']}, {self.camera_info['d']}")
        except Exception as e:
            self.get_logger().warn(f"Could not load camera info file: {e}")

        # 4.convert cv2 (cvbridge)
        self.bridge = CvBridge()

        self.show_result = self.get_parameter('show_result').value

    def camera_info_callback(self, msg: CameraInfo):
        """
        通过回调函数获取到相机的参数信息
        """
        if not self.camera_info_received:
            self.camera_info['k'] = list(msg.k)
            self.camera_info['p'] = list(msg.p)
            self.camera_info['d'] = list(msg.d)
            self.camera_info['r'] = list(msg.r)
            self.camera_info['roi'] = msg.roi
            self.camera_info_received = True
            
            self.get_logger().info(f"Received camera info: K={self.camera_info['k']}, D={self.camera_info['d']}")
            self.get_logger().info("Camera info received, will ignore subsequent camera info messages")

    def image_callback(self, msg: Image):
        # Skip processing if camera info is not available
        if not self.camera_info.get('k') or not self.camera_info.get('d'):
            self.get_logger().warn("Camera info not available, skipping image processing")
            return

        # 5.detect pub result
        image = self.bridge.imgmsg_to_cv2(msg)
        
        # Add some debug info about the image
        self.get_logger().info(f"Processing image: {image.shape}, dtype: {image.dtype}")
        
        detect_result = self.yolov5.predict(image)
        
        self.result_msg.detections.clear()
        self.result_msg.header.frame_id = "camera"
        self.result_msg.header.stamp = self.get_clock().now().to_msg()

        # Check if detections exist and log more details
        if not detect_result.pred or len(detect_result.pred) == 0:
            self.get_logger().info("No prediction results from YOLOv5")
            return
            
        predictions = detect_result.pred[0]
        if len(predictions) == 0:
            self.get_logger().info("No detections found (empty predictions)")
            # Still show the image even if no detections
            if self.show_result:
                cv2.imshow('YOLOv5 Detection Results', image)
                cv2.waitKey(1)
            return

        # parse results
        boxes = predictions[:, :4]  # x1, y1, x2, y2
        scores = predictions[:, 4]
        categories = predictions[:, 5]

        self.get_logger().info(f"Found {len(categories)} detections with scores: {scores.tolist()}")

        for index in range(len(categories)):
            name = detect_result.names[int(categories[index])]
            detection2d = Detection2D()
            detection2d.id = name
            # detection2d.bbox
            x1, y1, x2, y2 = boxes[index]
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            center_x = (x1+x2)/2.0
            center_y = (y1+y2)/2.0
            detection2d.bbox.center.position.x = center_x
            detection2d.bbox.center.position.y = center_y
            detection2d.bbox.size_x = float(x2-x1)
            detection2d.bbox.size_y = float(y2-y1)

            obj_pose = ObjectHypothesisWithPose()
            obj_pose.hypothesis.class_id = name
            obj_pose.hypothesis.score = float(scores[index])

            # px2xy - only if camera info is properly loaded
            try:
                world_x, world_y = px2xy(
                    [center_x, center_y], self.camera_info["k"], self.camera_info["d"], 1)
                obj_pose.pose.pose.position.x = world_x
                obj_pose.pose.pose.position.y = world_y
            except Exception as e:
                self.get_logger().warn(f"Error in px2xy conversion: {e}")
                obj_pose.pose.pose.position.x = 0.0
                obj_pose.pose.pose.position.y = 0.0
            
            # obj_pose.pose.pose.position.z = 1.0  #2D相机则显示,归一化后的结果,用户用时自行乘上深度z获取正确xy
            detection2d.results.append(obj_pose)
            self.result_msg.detections.append(detection2d)

            # draw
            if self.show_result:
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, f"{name}: {scores[index]:.2f}", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # if view or pub
        if self.show_result:
            cv2.imshow('YOLOv5 Detection Results', image)
            cv2.waitKey(1)

        if len(categories) > 0:
            self.yolo_result_pub.publish(self.result_msg)
            self.get_logger().info(f"Published {len(categories)} detections")


def main():
    rclpy.init()
    node = YoloV5Ros2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLOv5 node")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
