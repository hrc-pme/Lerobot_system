#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch Leader-Follower 整合驅動節點 (Unified Driver)
- 同時啟動 Leader 和 Follower 介面在同一個 Node 中
- 不包含 Leader -> Follower 的自動控制邏輯 (需外部或另一個腳本做 topic relay)
- Leader: 讀取狀態, Torque OFF, 發佈 /<arm>/joint_states
- Follower: 讀取狀態, Torque ON, 發佈 /<arm>/joint_states; 訂閱 /<arm>/joint_states_control 寫入位置
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

class KochLeaderFollowerNode(Node):
    def __init__(self):
        super().__init__('leader_follower_unified_node')
        
        # 參數設置
        self.declare_parameter('config_file', '/home/hrc/Lerobot_system/ros2_ws/src/koch_control/config/single_leader_follower.yaml')
        self.declare_parameter('calibration_dir', '/home/hrc/Lerobot_system/calibration')
        self.declare_parameter('publish_rate', 50.0)

        config_path = self.get_parameter('config_file').value
        self.calib_root = self.get_parameter('calibration_dir').value
        pub_rate = self.get_parameter('publish_rate').value

        # 讀取配置
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.buses = {}    # {arm_name: method_bus}
        self.pubs = {}     # {arm_name: publisher}
        self.subs = {}     # {arm_name: subscriber} (Only Follower)
        self.srvs = {}     # {arm_name: service}    (Only Follower)

        # 初始化 Leader Arms
        for arm_cfg in self.config.get('leader_arms', []):
            self._init_arm(arm_cfg, "leader")

        # 初始化 Follower Arms
        for arm_cfg in self.config.get('follower_arms', []):
            self._init_arm(arm_cfg, "follower")

        # 定時發佈所有狀態
        self.timer = self.create_timer(1.0/pub_rate, self._publish_all)
        self.get_logger().info("Unified Node Ready. Leaders are passive, Followers are active.")

    def _init_arm(self, arm_cfg, role):
        name = arm_cfg['name']
        port = arm_cfg['port']
        side = "right" if "right" in name else "left"
        
        # 1. 準備 Motors
        motors = {}
        for m_name, spec in arm_cfg['motors'].items():
            motors[m_name] = Motor(id=spec[0], model=spec[1], norm_mode=MotorNormMode.DEGREES)

        # 2. 準備 Calibration
        # 路徑: .../calibration/koch/{role}/koch_{side}_{role}_calibration.json
        calib_file = Path(self.calib_root) / "koch" / role / f"koch_{side}_{role}_calibration.json"
        
        calibration = {}
        if calib_file.exists():
            try:
                with open(calib_file) as f:
                    data = json.load(f)
                    # 處理兩種可能的 JSON 格式
                    root = data.get('motors', data) # 如果是新格式有 'motors' key
                    # 如果 root 裡面還有 's' 結尾的鍵值...
                    if 'motors' in data: # LeRobot new schema
                        for k, v in data['motors'].items():
                             calibration[k] = self._make_calib_obj(v)
                    else: # Flat dict
                        for k, v in data.items():
                             calibration[k] = self._make_calib_obj(v)
                self.get_logger().info(f"[{role}] {name}: Loaded calibration {calib_file}")
            except Exception as e:
                self.get_logger().warn(f"[{role}] {name}: Failed to load calibration: {e}")
        else:
            self.get_logger().warn(f"[{role}] {name}: Calibration not found at {calib_file}")

        # 3. 建立並連接 Bus
        bus = DynamixelMotorsBus(port=port, motors=motors, calibration=calibration)
        try:
            bus.connect()
            if role == "leader":
                bus.disable_torque() # Leader 被動
            else:
                bus.enable_torque()  # Follower 主動維持位置

            self.buses[name] = bus
            
            # 4. ROS 介面
            # Publisher
            self.pubs[name] = self.create_publisher(JointState, f'/{name}/joint_states', 10)
            
            # Follower 專屬介面
            if role == "follower":
                self.subs[name] = self.create_subscription(
                    JointState, 
                    f'/{name}/joint_states_control', 
                    lambda msg, n=name: self._follower_control_cb(msg, n), 
                    10
                )
                self.srvs[name] = self.create_service(
                     Trigger, 
                     f'/{name}_reset_to_home',
                     lambda req, res, n=name: self._follower_home_cb(req, res, n)
                )

            status = "Torque OFF" if role == "leader" else "Torque ON"
            self.get_logger().info(f"Initialized {role} arm '{name}' on {port} ({status})")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize {name}: {e}")

    def _make_calib_obj(self, data):
        return MotorCalibration(
            id=data.get('id', 0),
            drive_mode=data.get('drive_mode', 0),
            homing_offset=data.get('homing_offset', 0),
            range_min=data.get('range_min', 0),
            range_max=data.get('range_max', 4096)
        )

    def _publish_all(self):
        for name, bus in self.buses.items():
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

    def _follower_control_cb(self, msg, arm_name):
        """ Follower 接收控制指令 """
        try:
            bus = self.buses[arm_name]
            target_deg = {}
            motor_names = list(bus.motors.keys())

            if len(msg.name) > 0:
                for joint_name, pos_rad in zip(msg.name, msg.position):
                    short_name = joint_name.replace(f"{arm_name}_", "")
                    if short_name in bus.motors:
                        target_deg[short_name] = np.degrees(pos_rad)
            else:
                for i, pos_rad in enumerate(msg.position):
                    if i < len(motor_names):
                        target_deg[motor_names[i]] = np.degrees(pos_rad)
            
            if target_deg:
                bus.sync_write("Goal_Position", target_deg)
        except Exception as e:
            self.get_logger().warn(f"Control error {arm_name}: {e}")

    def _follower_home_cb(self, req, res, arm_name):
        try:
            bus = self.buses[arm_name]
            # 歸零 (因 NormMode=DEGREES, 0 即為校正點)
            targets = {name: 0.0 for name in bus.motors.keys()}
            bus.sync_write("Goal_Position", targets)
            res.success = True
            res.message = "Homed"
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def destroy_node(self):
        for bus in self.buses.values():
            try: 
                bus.disable_torque()
                bus.disconnect()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KochLeaderFollowerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()