#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/hrc/Lerobot_system/ros2_ws/install/setup.bash

# 【新增】啟動 TF 發布器，將 Koch 的 Base 連結到 OpenArm 的 Base 上面！
# 依照你的要求，讓 Koch 的兩隻手相聚 40cm (一個 y=0.2, 一個 y=-0.2)
# 同時，OpenArm 似乎預設是往前伸，我們加入 90 度的 Yaw 旋轉 (1.5708 rad) 來對齊兩者的座標系朝向
echo "Starting Static TF Publishers..."
# ----------------------
# 放一個統一的 world 座標：Koch面向前方 (Yaw=0)
# OpenArm 若發現向右伸反倒往左走，代表X/Y定義呈180度反轉，我們這裡將 Yaw 改為 3.14159 (180度)
# ----------------------
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id openarm_body_link0 > /dev/null 2>&1 &
P_TF0=$!
ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0.18 --z 0.33 --yaw -1.5708 --pitch 0 --roll 0 --frame-id world --child-frame-id left_base_link > /dev/null 2>&1 &
P_TF1=$!
ros2 run tf2_ros static_transform_publisher --x 0.5 --y -0.18 --z 0.33 --yaw -1.5708 --pitch 0 --roll 0 --frame-id world --child-frame-id right_base_link > /dev/null 2>&1 &
P_TF2=$!

# 捕捉 Ctrl+C 訊號，確保徹底關閉所有背景發佈器
trap 'echo "Stopping processes..."; kill $P1 $P2 $P3 $P_TF0 $P_TF1 $P_TF2; exit 0' SIGINT SIGTERM

echo "Starting OpenArm robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /home/hrc/Lerobot_system/repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf)" > /dev/null 2>&1 &
P1=$!

echo "Starting Koch Left robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/hrc/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:='left_')" -r /robot_description:=/robot_description_left -r /joint_states:=/joint_states > /dev/null 2>&1 &
P2=$!

echo "Starting Koch Right robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/hrc/Lerobot_system/ros2_ws/src/koch_simulation/urdf/low_cost_robot.xacro prefix:='right_')" -r /robot_description:=/robot_description_right -r /joint_states:=/joint_states > /dev/null 2>&1 &
P3=$!


echo "Starting RViz2..."
rviz2

# 當 RViz 正常關閉時，關閉所有背景發佈器
echo "Stopping processes..."
kill $P1 $P2 $P3 $P_TF0 $P_TF1 $P_TF2 2>/dev/null
wait
