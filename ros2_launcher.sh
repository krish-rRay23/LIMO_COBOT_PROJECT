#!/bin/bash
cd ~/krish_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch my_launch_pkg full_system.launch.py

