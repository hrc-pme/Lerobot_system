#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch Follower 控制節點 (Simplified)
- 連接 Follower 手臂
- 讀取校正檔 (koch_{side}_follower_calibration.json)
- 發佈 JointState 到 /<arm_name>/joint_states
- 訂閱 /<arm_name>/joint_states_control (Joy/Controller cmd) 寫入 Goal_Position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import sys
sys.path.append('/home/hrc/Lerobot_system/repos/lerobot/src')

from lerobot.motors.dynamixel import DynamixelMotorsBus
from lerobot.motors.motors_bus import Motor, MotorNormMode, MotorCalibration
import numpy as np
import yaml
import json
import os
from pathlib import Path

class KochFollower(Node):
    def __init__(self):
        super().__init__('follower_control_node')
        
        # 參數設置
        self.declare_parameter('config_file', '/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/single_leader_follower.yaml')
        self.declare_parameter('calibration_dir', '/home/hrc/Lerobot_system/calibration')
        self.declare_parameter('publish_rate', 50.0)

        config_path = self.get_parameter('config_file').value
        calib_root = self.get_parameter('calibration_dir').value
        pub_rate = self.get_parameter('publish_rate').value

        # 讀取配置
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.arms = {}
        self.pubs = {}
        self.subs = {}
        self.srvs = {}

        # 初始化 Follower Arms
        for arm_cfg in config.get('follower_arms', []):
            name = arm_cfg['name']
            port = arm_cfg['port']
            
            # 初始化 Motors
            motors = {}
            for m_name, spec in arm_cfg['motors'].items():
                motors[m_name] = Motor(id=spec[0], model=spec[1], norm_mode=MotorNormMode.DEGREES)

            # 載入校正
            side = "right" if "right" in name else "left"
            calib_file = Path(calib_root) / "koch" / "follower" / f"koch_{side}_follower_calibration.json"
            
            calibration = {}
            if calib_file.exists():
                try:
                    with open(calib_file) as f:
                        data = json.load(f)
                        # 處理兩種可能的 JSON 格式 (LeRobot 原生 vs 自定義)
                        for m_key, m_val in data.items():
                            if m_key == "motors": 
                                # 遞迴或重新遍歷
                                for sub_k, sub_v in m_val.items():
                                    calibration[sub_k] = self._make_calib_obj(sub_v)
                                break
                            else:
                                calibration[m_key] = self._make_calib_obj(m_val)
                    self.get_logger().info(f"✅ Loaded calibration from {calib_file}")
                    self.get_logger().info(f"   Calibration keys: {list(calibration.keys())}")
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to load calibration: {e}")
                    calibration = {}
            else:
                self.get_logger().warn(f"⚠️  Calibration not found: {calib_file}")
                self.get_logger().warn(f"   Robot will use default settings (may not work correctly!)")
                # 即使沒有校正文件，也要創建空的校正對象以避免錯誤
                calibration = None

            # 建立 Bus
            # 重要: DynamixelMotorsBus 需要正確的 calibration 參數才能正常工作
            # calibration 用於 normalize/unnormalize 位置數據
            self.get_logger().info(f"🔧 Creating DynamixelMotorsBus for {name}")
            self.get_logger().info(f"   Port: {port}")
            self.get_logger().info(f"   Motors: {list(motors.keys())}")
            self.get_logger().info(f"   Calibration: {'✅ Loaded' if calibration else '❌ None'}")
            
            if calibration:
                self.get_logger().info(f"   Calibration motors: {list(calibration.keys())}")
            
            bus = DynamixelMotorsBus(port=port, motors=motors, calibration=calibration)
            try:
                bus.connect()
                self.get_logger().info(f"✅ Connected to {port}")
                
                # 檢查校正是否正確載入
                if bus.calibration:
                    self.get_logger().info(f"✅ Bus has calibration for: {list(bus.calibration.keys())}")
                else:
                    self.get_logger().error(f"❌ Bus calibration is None!")
                
                bus.enable_torque() # Follower 需保持位置
                self.get_logger().info(f"✅ Torque enabled for {name}")
                self.arms[name] = bus
                
                # ROS 介面
                ## Publisher: /<name>/joint_states
                self.pubs[name] = self.create_publisher(JointState, f'/{name}/joint_states', 10)
                
                ## Subscriber: /<name>/joint_states_control
                self.subs[name] = self.create_subscription(
                    JointState, 
                    f'/{name}/joint_states_control', 
                    lambda msg, n=name: self._control_cb(msg, n), 
                    10
                )

                ## Service: /<name>_reset_to_home (回原點)
                self.srvs[name] = self.create_service(
                     Trigger, 
                     f'/{name}_reset_to_home',
                     lambda req, res, n=name: self._home_cb(req, res, n)
                )

                self.get_logger().info(f"Follower {name} ready on {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect {name}: {e}")

        # 定時發佈
        self.create_timer(1.0/pub_rate, self._publish)

    def _make_calib_obj(self, data):
        # MotorCalibration 是 @dataclass，需要所有參數
        return MotorCalibration(
            id=int(data.get('id', 0)),
            drive_mode=int(data.get('drive_mode', 0)),
            homing_offset=int(data.get('homing_offset', 0)),
            range_min=int(data.get('range_min', 0)),
            range_max=int(data.get('range_max', 4095))
        )

    def _publish(self):
        for name, bus in self.arms.items():
            try:
                readings = bus.sync_read("Present_Position")
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                for joint_name, pos_deg in readings.items():
                    msg.name.append(f"{name}_{joint_name}")
                    msg.position.append(np.radians(pos_deg))
                self.pubs[name].publish(msg)
            except Exception:
                pass

    def _control_cb(self, msg, arm_name):
        """
        接收控制命令 (弧度) -> 轉為度數 -> 寫入馬達
        """
        try:
            bus = self.arms[arm_name]
            target_deg = {}
            motor_names = list(bus.motors.keys())

            # 嘗試名稱對齊
            if len(msg.name) > 0:
                for joint_name, pos_rad in zip(msg.name, msg.position):
                    # 處理不同命名慣例: "right_follower_shoulder_pan" vs "shoulder_pan"
                    short_name = joint_name.replace(f"{arm_name}_", "")
                    if short_name in bus.motors:
                        target_deg[short_name] = np.degrees(pos_rad)
            else:
                # 若無名稱則用順序對齊
                for i, pos_rad in enumerate(msg.position):
                    if i < len(motor_names):
                        target_deg[motor_names[i]] = np.degrees(pos_rad)
            
            if target_deg:
                bus.sync_write("Goal_Position", target_deg)
        except Exception as e:
            self.get_logger().warn(f"Control error {arm_name}: {e}")

    def _home_cb(self, req, res, arm_name):
        """回歸原點 (所有關節回歸校正中的零位 -> 即 0 度)"""
        try:
            bus = self.arms[arm_name]
            # 因使用 MotorNormMode.DEGREES 且已載入 homing_offset，
            # 寫入 0 度即代表回到物理校正原點
            targets = {name: 0.0 for name in bus.motors.keys()}
            bus.sync_write("Goal_Position", targets)
            res.success = True
            res.message = "Homed"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def destroy_node(self):
        for bus in self.arms.values():
            try: 
                bus.disable_torque() # 安全起見，結束時放鬆
                bus.disconnect()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KochFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()