#!/bin/bash
set -e

# Setup Environment
source /opt/ros/humble/setup.bash
source ~/Lerobot_system/ros2_ws/install/setup.bash

# follower
ros2 run koch_control koch_follower_control --ros-args -p config_file:=/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/two_follower.yaml