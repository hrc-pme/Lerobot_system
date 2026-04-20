#!/bin/bash
# 確保 ROS 2 和工作區已經 source
source /opt/ros/humble/setup.bash
source /home/hrc/Lerobot_system/ros2_ws/install/setup.bash

# 自動產生 URDF 到 /tmp 目錄供轉換器使用
xacro /home/hrc/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=left_ > /tmp/koch_left.urdf
xacro /home/hrc/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:=right_ > /tmp/koch_right.urdf

echo "Koch URDFs generated at /tmp/koch_left.urdf and /tmp/koch_right.urdf"
