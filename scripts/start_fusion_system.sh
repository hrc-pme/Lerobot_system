#!/bin/bash
# 快速啟動腳本 - PointCloud Fusion System

echo "========================================="
echo "  PointCloud Fusion System - 快速啟動"
echo "========================================="

# 設定顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 進入工作空間
cd /home/hrc/Lerobot_system/ros2_ws

echo -e "${YELLOW}[1/3] 編譯 package...${NC}"
colcon build --packages-select koch_tactile_ros2
if [ $? -ne 0 ]; then
    echo -e "${RED}編譯失敗！${NC}"
    exit 1
fi

echo -e "${GREEN}編譯成功！${NC}"

echo -e "${YELLOW}[2/3] Source 環境...${NC}"
source install/setup.bash

echo -e "${YELLOW}[3/3] 啟動系統...${NC}"
echo ""
echo "選擇啟動模式："
echo "  1) 完整系統 (RealSense + Wowskin + Fusion)"
echo "  2) 只啟動融合節點 (假設 RealSense 和 Wowskin 已運行)"
echo "  3) 測試訂閱器"
echo ""
read -p "請選擇 (1-3): " choice

case $choice in
    1)
        echo -e "${GREEN}啟動完整系統...${NC}"
        ros2 launch koch_tactile_ros2 complete_system.launch.py
        ;;
    2)
        echo -e "${GREEN}啟動融合節點...${NC}"
        ros2 launch koch_tactile_ros2 pointcloud_fusion.launch.py
        ;;
    3)
        echo -e "${GREEN}啟動測試訂閱器...${NC}"
        ros2 run koch_tactile_ros2 test_fusion_subscriber
        ;;
    *)
        echo -e "${RED}無效選擇！${NC}"
        exit 1
        ;;
esac
