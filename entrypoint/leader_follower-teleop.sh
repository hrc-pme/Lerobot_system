#!/bin/bash
set -e

# Setup Environment
source /opt/ros/humble/setup.bash
source ~/Lerobot_system/ros2_ws/install/setup.bash

# Ensure PYTHONPATH includes venv for LeRobot
export PYTHONPATH=/opt/venv/lib/python3.10/site-packages:$PYTHONPATH

# Default Config (Single Right Arm)
DEFAULT_CONFIG="/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/two_leader_follower.yaml"
# Default Pairs (for Bridge)
DEFAULT_PAIRS="['right_leader:right_follower']"

echo "=================================================="
echo "   Koch Robot Teleoperation (Unified)"
echo "=================================================="
echo "1. Single Right Arm (default)"
echo "2. Single Left Arm"
echo "3. Dual Arms (Both Left & Right)"
echo "=================================================="
read -p "Select Mode (1-3): " mode

CONFIG_FILE=$DEFAULT_CONFIG
PAIRS=$DEFAULT_PAIRS

if [ "$mode" == "2" ]; then
    echo "🔵 Selected: Single Left Arm"
    # 假設您之後會建立這個 yaml，如果還沒，請確保 config 資料夾有對應檔案
    # CONFIG_FILE="/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/single_left_leader_follower.yaml" 
    # 目前先用現有的兩個檔案示範邏輯，您可能需要新增 left 的 yaml
    # 因為目前只有 two_leader_follower.yaml 包含雙手
    echo "⚠️  注意: 請確保 config 目錄下有單手左臂的 YAML 設定"
    PAIRS="['left_leader:left_follower']"

elif [ "$mode" == "3" ]; then
    echo "🟣 Selected: Dual Arms"
    CONFIG_FILE="/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/two_leader_follower.yaml"
    PAIRS="['right_leader:right_follower', 'left_leader:left_follower']"
else
    echo "🟢 Selected: Single Right Arm"
    CONFIG_FILE="/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/single_leader_follower.yaml"
    PAIRS="['right_leader:right_follower']"
fi

echo "📂 Using Config: $CONFIG_FILE"

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "🛑 Stopping..."
    if [ -n "$PID_BRIDGE" ]; then kill $PID_BRIDGE 2>/dev/null; fi
    if [ -n "$PID_DRIVER" ]; then kill $PID_DRIVER 2>/dev/null; fi
    exit 0
}
trap cleanup SIGINT

# 1. Start the Unified Driver with selected Config
echo "🚀 Starting Hardware Driver..."
ros2 run koch_control koch_leader_follower_control --ros-args -p config_file:="$CONFIG_FILE" &
PID_DRIVER=$!

# Wait for driver initialization
sleep 5

# 2. Start the Teleop Bridge with selected Pairs
echo "🔗 Starting Teleop Bridge..."
ros2 run koch_control koch_teleop_bridge --ros-args -p pairs:="$PAIRS" &
PID_BRIDGE=$!

echo "✅ System Ready!"
wait $PID_DRIVER $PID_BRIDGE


