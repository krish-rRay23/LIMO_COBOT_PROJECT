from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

ip_suffix = input("🔌 Enter MyCobot IP suffix (e.g., 79 for 192.168.137.79): ").strip()
m5_ip = f"192.168.137.{ip_suffix}"

def generate_launch_description():
    return LaunchDescription([

        # 1. 🟢 Start LIMO base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("limo_bringup"),
                    "launch",
                    "limo_start.launch.py"
                ])
            ])
        ),

        # 2. 📷 Start Orbbec DaBai camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("orbbec_camera"),
                    "launch",
                    "dabai.launch.py"
                ])
            ])
        ),

        # 3. 🧭 Start Nav2 with map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("limo_bringup"),
                    "launch",
                    "limo_nav2.launch.py"
                ])
            ]),
            launch_arguments=[
                ("map", "/home/agilex/krish_ws/maps/room_map.yaml")
            ]
        ),

        # 4. 🧠 YOLOv8 detector
        Node(
            package="object_detector",
            executable="yolo_detector",
            name="yolo_detector",
            output="screen"
        ),
        Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="image_viewer",
            output="screen"
        ),
        # 5. 🤖 Object handler (corrected package)
        Node(
            package="nav_handler",
            executable="object_handler",
            name="object_handler",
            parameters=[{"m5_ip": m5_ip}],
            output="screen"
        ),
    ])