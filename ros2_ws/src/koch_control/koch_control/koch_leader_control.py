#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Koch Leader 控制節點 (Simplified)
- 連接 Leader 手臂
- 讀取校正檔 (koch_{side}_leader_calibration.json)
- 發佈 JointState 到 /<arm_name>/joint_states
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

class KochLeader(Node):
    def __init__(self):
        super().__init__('leader_control_node')
        
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

        # 初始化 Leader Arms
        for arm_cfg in config.get('leader_arms', []):
            name = arm_cfg['name']
            port = arm_cfg['port']
            
            # 初始化 Motors
            motors = {}
            for m_name, spec in arm_cfg['motors'].items():
                motors[m_name] = Motor(id=spec[0], model=spec[1], norm_mode=MotorNormMode.DEGREES)

            # 載入校正
            side = "right" if "right" in name else "left"
            calib_file = Path(calib_root) / "koch" / "leader" / f"koch_{side}_leader_calibration.json"
            
            calibration = {}
            if calib_file.exists():
                with open(calib_file) as f:
                    data = json.load(f)
                    # 處理兩種可能的 JSON 格式 (LeRobot 原生 vs 自定義)
                    # 這裡假設是 koch_calibration.py 剛生成的 LeRobot 格式 (dict[motor_name, dict])
                    for m_key, m_val in data.items():
                        # 檢查是否包含必要的 key，若是 'motors' 包一層則解開
                        if m_key == "motors": 
                            # 遞迴或重新遍歷
                            for sub_k, sub_v in m_val.items():
                                calibration[sub_k] = self._make_calib_obj(sub_v)
                            break
                        else:
                            calibration[m_key] = self._make_calib_obj(m_val)
                self.get_logger().info(f"Loaded calibration: {calib_file}")
            else:
                self.get_logger().warn(f"Calibration not found: {calib_file}")

            # 建立 Bus
            bus = DynamixelMotorsBus(port=port, motors=motors, calibration=calibration)
            try:
                bus.connect()
                bus.disable_torque() # Leader 需可被移動
                self.arms[name] = bus
                self.pubs[name] = self.create_publisher(JointState, f'/{name}/joint_states', 10)
                self.get_logger().info(f"Leader {name} ready on {port}")
            except Exception as e:
                self.get_logger().error(f"Failed to connect {name}: {e}")

        # 定時發佈
        self.create_timer(1.0/pub_rate, self._publish)

    def _make_calib_obj(self, data):
        return MotorCalibration(
            id=data.get('id', 0),
            drive_mode=data.get('drive_mode', 0),
            homing_offset=data.get('homing_offset', 0),
            range_min=data.get('range_min', 0),
            range_max=data.get('range_max', 4096)
        )

    def _publish(self):
        for name, bus in self.arms.items():
            try:
                # 讀取位置 (度)
                readings = bus.sync_read("Present_Position")
                
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                
                # 轉換為弧度並填充消息
                for joint_name, pos_deg in readings.items():
                    msg.name.append(f"{name}_{joint_name}")
                    msg.position.append(np.radians(pos_deg))
                
                self.pubs[name].publish(msg)
            except Exception:
                pass

    def destroy_node(self):
        for bus in self.arms.values():
            try: bus.disconnect()
            except: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KochLeader()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()