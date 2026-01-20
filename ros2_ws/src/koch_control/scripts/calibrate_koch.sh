#!/bin/bash
# Koch 機器人校正腳本 - 使用 Python 校正工具
# 這個腳本是一個簡單的啟動器，調用功能更完整的 Python 校正程式

echo "Koch 機器人校正工具 (Launcher)"
echo "調用 Python 工具進行 LeRobot 原生校正"
echo "=========================================================="

# ==========================================================
# 🔧 配置區域: 如果您要使用其他的 YAML 配置文件，請修改這裡
# ==========================================================
# 預設配置文件路徑
CONFIG_FILE="/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/single_leader_follower.yaml"
# ==========================================================

# 定義文件路徑
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PYTHON_SCRIPT="$SCRIPT_DIR/../koch_control/koch_calibration.py"

echo "使用配置文件: $CONFIG_FILE"
echo ""

# 檢查 Python 腳本是否存在
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "❌ 錯誤: 找不到 Python 校正腳本: $PYTHON_SCRIPT"
    exit 1
fi

# 執行 Python 腳本並傳遞配置文件路徑
python3 "$PYTHON_SCRIPT" --config "$CONFIG_FILE"

# 檢查執行結果
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================================="
    echo "✅ 校正程序正常結束"
    echo "校正檔位置: /home/hrc/Lerobot_system/calibration/koch/"
else
    echo ""
    echo "❌ 校正程序發生錯誤或被中斷"
fi