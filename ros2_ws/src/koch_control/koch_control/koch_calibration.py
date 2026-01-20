#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch 機器人校正工具 (Koch Robot Calibration Tool)
使用 LeRobot 原生校正方式，並支援自定義儲存路徑
"""
import yaml
import sys
import argparse
from pathlib import Path

sys.path.append('/home/hrc/Lerobot_system/repos/lerobot/src')

from lerobot.robots.koch_follower import KochFollowerConfig, KochFollower
from lerobot.teleoperators.koch_leader import KochLeaderConfig, KochLeader

# 預設 YAML 路徑 (使用絕對路徑以避免安裝後路徑錯誤)
DEFAULT_CONFIG = Path("/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/two_leader_follower.yaml")
# 校正檔案存放根目錄
CALIB_ROOT = Path("/home/hrc/Lerobot_system/calibration")

def run_calibration(role, arm_data):
    """執行單一手臂的校正流程"""
    name = arm_data['name']
    port = arm_data['port']
    
    # 判斷左右手 (side: right/left)
    side = "right" if "right" in name else "left" if "left" in name else "unknown"
    
    # 設定校正 ID 與 目錄 (格式: koch_{side}_{role}_calibration)
    calib_id = f"koch_{side}_{role}_calibration"
    calib_dir = CALIB_ROOT / "koch" / role
    
    print(f"\n=== 校正 {role.title()} ({side}) ===")
    print(f"裝置名稱: {name}")
    print(f"序列埠: {port}")
    
    if not Path(port).exists():
        print(f"❌ 錯誤: 找不到裝置 {port}")
        return False

    robot = None
    try:
        # 初始化配置與機器人物件
        ConfigClass = KochFollowerConfig if role == "follower" else KochLeaderConfig
        RobotClass = KochFollower if role == "follower" else KochLeader
        
        cfg = ConfigClass(port=port, id=calib_id, calibration_dir=calib_dir)
        robot = RobotClass(cfg)
            
        print("1. 連接中...")
        robot.connect(calibrate=False)
        print("   ✅ 連接成功")
        
        print(f"2. 開始校正 (儲存至 {calib_dir / (calib_id + '.json')})...")
        print("   👉 請依指示操作: 移至中間按 ENTER -> 遍歷全範圍")
        robot.calibrate()
        
        print(f"✅ {role.title()} 校正完成!")
        return True
        
    except KeyboardInterrupt:
        print("\n⚠️ 用戶中斷校正")
        return False
    except Exception as e:
        print(f"❌ 校正失敗: {e}")
        return False
    finally:
        if robot:
            try: robot.disconnect()
            except: pass

def select_arm(arms_list, role):
    """讓使用者選擇要校正哪一隻手臂"""
    if not arms_list:
        print(f"⚠️  設定檔中沒有找到 {role} 手臂")
        return None
    
    if len(arms_list) == 1:
        return arms_list[0]
        
    print(f"\n偵測到多隻 {role} 手臂，請選擇:")
    for i, arm in enumerate(arms_list):
        print(f"{i+1}. {arm['name']} ({arm['port']})")
    
    while True:
        try:
            choice = input(f"請輸入編號 (1-{len(arms_list)}): ").strip()
            idx = int(choice) - 1
            if 0 <= idx < len(arms_list):
                return arms_list[idx]
        except ValueError:
            pass
        print("無效輸入，請重新選擇")

def main():
    parser = argparse.ArgumentParser(description="Koch 校正工具")
    parser.add_argument("--config", type=str, default=str(DEFAULT_CONFIG), help="YAML 配置檔路徑")
    args = parser.parse_args()
    
    print(f"載入配置: {args.config}")
    try:
        with open(args.config) as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"❌ 無法讀取配置: {e}")
        sys.exit(1)

    while True:
        print("\n請選擇校正功能:")
        print("1. 校正 Follower")
        print("2. 校正 Leader")
        print("3. 校正兩者 (Follower -> Leader)")
        print("4. 離開")
        
        try:
            choice = input("輸入選項 (1-4): ").strip()
        except KeyboardInterrupt:
            break

        if choice == "1":
            arm = select_arm(config.get('follower_arms', []), "follower")
            if arm: run_calibration("follower", arm)
        elif choice == "2":
            arm = select_arm(config.get('leader_arms', []), "leader")
            if arm: run_calibration("leader", arm)
        elif choice == "3":
            f_arm = select_arm(config.get('follower_arms', []), "follower")
            if f_arm:
                if run_calibration("follower", f_arm):
                    input("\n按 Enter 繼續校正 Leader...")
                    l_arm = select_arm(config.get('leader_arms', []), "leader")
                    if l_arm: run_calibration("leader", l_arm)
        elif choice == "4":
            print("👋 程式結束")
            break
        else:
            print("無效選項")

if __name__ == "__main__":
    main()