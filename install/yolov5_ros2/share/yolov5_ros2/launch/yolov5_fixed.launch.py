from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov5_ros2',
            executable='yolo_detect_2d',
            name='yolov5_ros2',
            output='screen',
            parameters=[
                {'image_topic': '/camera/color/image_raw'},
                {'camera_info_topic': '/camera/color/camera_info'},
                {'device': 'cpu'},
                {'confidence_threshold': 0.01},  # Lower confidence threshold
                {'show_result': True}
            ]
        )
    ])
