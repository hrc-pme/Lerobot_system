#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class OpenArmRealHardwareBridge(Node):
    def __init__(self):
        super().__init__('openarm_real_hardware_bridge')
        
        self.arms = ['left', 'right']

        # 儲存目前的平滑位置與目標位置 (7個關節 + 1個夾爪 = 8)
        self.start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_positions = {arm: list(self.start_pose) for arm in self.arms}
        self.target_positions = {arm: list(self.start_pose) for arm in self.arms}
        
        self.is_connected = {arm: False for arm in self.arms}
        self.last_msg_time = {arm: self.get_clock().now() for arm in self.arms}

        self.alpha = 0.3 # 平滑係數 (可以微調，越大跟隨越快但越抖)
        self.max_vel = 2.5
        self.timeout_sec = 2.0

        # 訂閱模擬器或轉換器算出來的 OpenArm 關節位置指令
        self.create_subscription(JointState, "/openarm/joint_commands", self.openarm_cmd_cb, 10)
        
        self.pubs = {}
        self.gripper_pubs = {}
        for arm in self.arms:
            # 發布給手臂 position controller
            self.pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_forward_position_controller/commands', 10)

            # 發布給夾爪 position controller
            self.gripper_pubs[arm] = self.create_publisher(
                Float64MultiArray, f'/{arm}_gripper_controller/commands', 10)

        # 控制迴圈 (預設 50Hz / 0.02s，與你前一個 VR 控制相同)
        self.last_loop_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("OpenArm Real Hardware Bridge (Position Controller Mode) is running...")

    def openarm_cmd_cb(self, msg: JointState):
        left_target = list(self.target_positions['left'])
        right_target = list(self.target_positions['right'])
        
        left_updated = False
        right_updated = False

        # 將收到的整包 OpenArm 指令拆解並更新目標
        for name, pos in zip(msg.name, msg.position):
            if "left" in name:
                self.is_connected['left'] = True
                self.last_msg_time['left'] = self.get_clock().now()
                if "finger" in name:
                    left_target[7] = float(pos)
                    left_updated = True
                else:
                    for idx in range(1, 8):
                        if f"joint{idx}" in name:
                            left_target[idx-1] = float(pos)
                            left_updated = True
                            break
            elif "right" in name:
                self.is_connected['right'] = True
                self.last_msg_time['right'] = self.get_clock().now()
                if "finger" in name:
                    right_target[7] = float(pos)
                    right_updated = True
                else:
                    for idx in range(1, 8):
                        if f"joint{idx}" in name:
                            right_target[idx-1] = float(pos)
                            right_updated = True
                            break
                            
        if left_updated:
            self.target_positions['left'] = left_target
        if right_updated:
            self.target_positions['right'] = right_target

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds / 1e9
        self.last_loop_time = now

        for arm in self.arms:
            elapsed = (now - self.last_msg_time[arm]).nanoseconds / 1e9
            if self.is_connected[arm] and elapsed > self.timeout_sec:
                self.is_connected[arm] = False

            max_step = self.max_vel * dt

            temp_full_pos = []
            for i in range(8):
                target = self.target_positions[arm][i]
                current = self.current_positions[arm][i]

                # 如果持續接收到更新，就進行平滑 (alpha濾波)
                if self.is_connected[arm]:
                    smoothed = self.alpha * target + (1.0 - self.alpha) * current
                    diff = smoothed - current
                    step = max(-max_step, min(diff, max_step))
                    new_pos = current + step
                else:
                    new_pos = current # 逾時斷線時停在原地

                temp_full_pos.append(new_pos)

            self.current_positions[arm] = temp_full_pos

            # --- 分流發送 ---
            # 1. 發送前 7 軸給手臂 Joystick/Position Controller
            arm_msg = Float64MultiArray(data=temp_full_pos[:7])
            self.pubs[arm].publish(arm_msg)

            # 2. 發送第 8 軸給夾爪
            gripper_msg = Float64MultiArray(data=[temp_full_pos[7]])
            self.gripper_pubs[arm].publish(gripper_msg)

    def stop_robot(self):
        """Ctrl+C 強制停止時的安全保護，停止更新並發送目前位置讓手臂停在原地"""
        self.get_logger().warn("Ctrl+C detected! 鎖定當前位置保護真實手臂...")
        self.is_connected['left'] = False
        self.is_connected['right'] = False

def main(args=None):
    rclpy.init(args=args)
    node = OpenArmRealHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
