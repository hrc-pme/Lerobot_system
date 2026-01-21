#!/bin/bash
set -e

# Setup Environment
source /opt/ros/humble/setup.bash
source ~/Lerobot_system/ros2_ws/install/setup.bash

# Launch cameras
#ros2 launch koch_control rs_far_launch.py &
ros2 launch koch_control rs_first_launch.py &
ros2 launch koch_control rs_top_launch.py