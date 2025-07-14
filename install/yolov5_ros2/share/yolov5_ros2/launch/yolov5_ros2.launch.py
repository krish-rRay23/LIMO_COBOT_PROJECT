from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov5_ros2',
            executable='yolov5_ros2',
            name='yolov5_ros2',
            output='screen',
            parameters=[{'image_topic': '/color/image_raw'}]  # Change this to your camera topic
        )
    ])

