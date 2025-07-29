from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import socket, threading, time, psutil

def get_subnet():
    for iface_info in psutil.net_if_addrs().values():
        for addr in iface_info:
            if addr.family == socket.AF_INET and not addr.address.startswith("127."):
                return ".".join(addr.address.split('.')[:3]) + '.'
    return "192.168.137."

def scan_ip(subnet, port=9000, timeout=3, fallback="192.168.137.75"):
    found_ip = None
    stop = threading.Event()
    lock = threading.Lock()

    def try_ip(ip):
        nonlocal found_ip
        try:
            with socket.create_connection((ip, port), timeout=0.2):
                with lock:
                    if not found_ip:
                        found_ip = ip
                        stop.set()
                        print(f"✅ Found MyCobot at {ip}")
        except:
            pass

    threads = [threading.Thread(target=try_ip, args=(f"{subnet}{i}",)) for i in range(1, 255)]
    for t in threads:
        t.start()

    start = time.time()
    while not stop.is_set() and time.time() - start < timeout:
        time.sleep(0.05)

    for t in threads:
        t.join(timeout=0.05)

    return found_ip or fallback

def generate_launch_description():
    subnet = get_subnet()
    detected_ip = scan_ip(subnet)

    limo_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("limo_bringup"),
                "launch",
                "limo_start.launch.py"
            ])
        ])
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "dabai.launch.py"
            ])
        ])
    )

    nav2 = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'limo_bringup', 'limo_nav2.launch.py',
            'map:=/home/agilex/krish_ws/maps/map11.yaml'
        ],
        output='screen'
    )

    pose_setter = Node(
        package="nav_handler",
        executable="pose_setter",
        name="pose_setter",
        output="screen"
    )

    yolo_node = Node(
        package="object_detector",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen"
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_static_tf",
        arguments=["0.12", "0", "0.21", "0", "0", "0", "base_link", "camera_link"],
        output="screen"
    )


    nav_handler = Node(
        package="nav_handler",
        executable="nav_handler",
        name="nav_handler",
        output="screen",
        additional_env={"M5_IP": detected_ip}  
    )

    rqt_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
        output='screen'
    )

    return LaunchDescription([
        SetLaunchConfiguration("m5_ip", detected_ip),
        DeclareLaunchArgument("m5_ip", default_value=detected_ip),

        limo_start,
        static_tf,
        camera_launch,

        TimerAction(period=3.0, actions=[yolo_node]),        
        TimerAction(period=5.0, actions=[nav2]),
        TimerAction(period=7.0, actions=[pose_setter]),
        TimerAction(period=15.0, actions=[nav_handler]),
        TimerAction(period=19.0, actions=[rqt_view])
    ])
