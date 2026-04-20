#!/usr/bin/env python3
import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import subprocess

# 將 end-effector 路徑加入，以便引入 kinematics 模組
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, ".."))

try:
    from robust_pino_kinematics import RobotKinematics
except ImportError:
    print("Error: Could not import robust_pino_kinematics. Make sure scripts/end-effector/robust_pino_kinematics.py exists.")
    sys.exit(1)

def ensure_urdf_exists():
    """確保 Koch 的 URDF 檔案存在於 /tmp 目錄下，如果沒有就自動呼叫腳本生成"""
    need_generate = False
    for side in ['left', 'right']:
        if not os.path.exists(f"/tmp/koch_{side}.urdf"):
            need_generate = True
            break
            
    if need_generate:
        print("Koch URDF not found in /tmp. Automatically generating them via xacro...")
        script_path = os.path.join(current_dir, "setup_koch_urdf.sh")
        if os.path.exists(script_path):
            subprocess.run(["bash", script_path], check=True)
        else:
            print(f"Warning: Setup script {script_path} not found. Ensure URDFs are manually created.")

def get_urdf_path(side):
    # 嘗試不同的 URDF 常見路徑
    possible_paths = [
        f"/tmp/koch_{side}.urdf",
        f"koch_{side}.urdf",
        f"{current_dir}/../koch_{side}.urdf"
    ]
    for p in possible_paths:
        if os.path.exists(p):
            return p
    return None

class JointToEEConverter(Node):
    def __init__(self):
        super().__init__('joint_to_ee_converter')
        
        # 確保 URDF 存在 (若在重開機後 /tmp 被清空，程式會在這裡自動幫你跑 xacro 指令)
        ensure_urdf_exists()
        
        # 1. 載入 Kinematics 模型 (FK 計算用)
        self.kin_models = {}
        urdf_l = get_urdf_path("left")
        urdf_r = get_urdf_path("right")
        
        if urdf_l:
            self.get_logger().info(f"Loading Left Arm Model from {urdf_l}")
            self.kin_models['left'] = RobotKinematics(urdf_l)
        else:
            self.get_logger().warning("Left URDF not found.")
            
        if urdf_r:
            self.get_logger().info(f"Loading Right Arm Model from {urdf_r}")
            self.kin_models['right'] = RobotKinematics(urdf_r)
        else:
            self.get_logger().warning("Right URDF not found.")

        # 2. 訂閱者 (監聽 LeRobot Agent 發佈的 Joint 控制指令) 【去程：Action】
        self.create_subscription(JointState, "/left_follower/joint_states_control", lambda msg: self.joint_cb(msg, "left"), 10)
        self.create_subscription(JointState, "/right_follower/joint_states_control", lambda msg: self.joint_cb(msg, "right"), 10)

        # 3. 發佈者 (將計算出來的 End-Effector 座標發佈出去，供機器人底層控制器使用) 【去程：Action】
        self.ee_pub_left = self.create_publisher(PoseStamped, "/left_follower/ee_pose_cmd", 10)
        self.ee_pub_right = self.create_publisher(PoseStamped, "/right_follower/ee_pose_cmd", 10)
        
        self.gripper_pub_left = self.create_publisher(Float32, "/left_follower/gripper_cmd", 10)
        self.gripper_pub_right = self.create_publisher(Float32, "/right_follower/gripper_cmd", 10)

        # ================= NEW: Observation 回推 (Cross-Robot 必備) =================
        # 4. 訂閱者 (監聽新機器人回傳的實際 End-Effector 位置) 【回程：Observation】
        self.create_subscription(PoseStamped, "/left_follower/actual_ee_pose", lambda msg: self.ee_cb(msg, "left"), 10)
        self.create_subscription(PoseStamped, "/right_follower/actual_ee_pose", lambda msg: self.ee_cb(msg, "right"), 10)

        # 5. 發佈者 (將 IK 算出的 Koch 虛擬 Joint 角度發布給 Policy) 【回程：Observation】
        self.virtual_joint_pub_left = self.create_publisher(JointState, "/left_follower/virtual_joint_states", 10)
        self.virtual_joint_pub_right = self.create_publisher(JointState, "/right_follower/virtual_joint_states", 10)

        # 紀錄上一幀的 Joint 角度，作為 IK 計算的初始值 (q_init) 加速收斂並避免姿勢突變
        self.last_q = {
            "left": np.zeros(self.kin_models['left'].dof) if 'left' in self.kin_models else None,
            "right": np.zeros(self.kin_models['right'].dof) if 'right' in self.kin_models else None
        }
        self.last_gripper = {"left": 0.0, "right": 0.0}
        # ============================================================================

        self.get_logger().info("Joint<->EE Bi-directional Converter Node Started. Cross-robot ready!")

    def joint_cb(self, msg, side):
        if side not in self.kin_models:
            return
            
        # 假設傳進來的是 [joint1, joint2, joint3, joint4, joint5, gripper] (總共 6 個)
        if len(msg.position) < 6:
            self.get_logger().warning(f"Received joint state for {side} with less than 6 joints.")
            return

        kin = self.kin_models[side]
        
        # 取出手臂的關節角度 (排除夾爪，通常取前 DOF 個)
        arm_joints = msg.position[:kin.dof] 
        # 假設最後一個值是 gripper 的數值
        gripper_val = msg.position[-1]     
        
        # --- 核心：正向運動學 Forward Kinematics (FK) ---
        try:
            pos, rot = kin.forward_kinematics(arm_joints)
        except Exception as e:
            self.get_logger().error(f"FK calculation failed: {e}")
            return
            
        # --- 打包成 PoseStamped 訊息 ---
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        # 基準座標系，如果是全域可以設定為 map，或是各自手臂的 base
        pose_msg.header.frame_id = f"{side}_base_link" 
        
        # Position (x, y, z)
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        
        # Orientation / Quaternion (x, y, z, w)
        pose_msg.pose.orientation.x = float(rot[0])
        pose_msg.pose.orientation.y = float(rot[1])
        pose_msg.pose.orientation.z = float(rot[2])
        pose_msg.pose.orientation.w = float(rot[3])
        
        # --- 打包 Gripper 訊息 ---
        gripper_msg = Float32()
        gripper_msg.data = float(gripper_val)
        
        # 發佈到對應的 Topic
        if side == "left":
            self.ee_pub_left.publish(pose_msg)
            self.gripper_pub_left.publish(gripper_msg)
        else:
            self.ee_pub_right.publish(pose_msg)
            self.gripper_pub_right.publish(gripper_msg)

    def ee_cb(self, msg, side):
        """【回程】將實際的 EE 座標透過 Koch IK 轉回虛擬 Koch 的 Joint 角度"""
        if side not in self.kin_models or self.last_q[side] is None:
            return

        kin = self.kin_models[side]

        # 讀取傳進來的目標座標與四元數
        target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        target_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        # 執行 IK 計算 (使用上一幀的角度 self.last_q 作為初始猜測值，確保連續性)
        try:
            virtual_q = kin.inverse_kinematics_5dof(
                target_pos=target_pos, 
                target_quat=target_quat, 
                q_init=self.last_q[side]
            )
            self.last_q[side] = virtual_q  # 更新上一幀狀態
        except Exception as e:
            self.get_logger().error(f"IK calculation failed: {e}")
            return

        # 把算出來的虛擬關節角度打包回 JointState
        virtual_joint_msg = JointState()
        virtual_joint_msg.header.stamp = self.get_clock().now().to_msg()
        virtual_joint_msg.header.frame_id = f"virtual_{side}_base"
        
        # 組合關節角度 (這裡將 gripper 加入最後一維，假設 gripper 不變，保持上次數值或由其他 Topic 補上)
        virtual_joint_msg.position = virtual_q.tolist() + [self.last_gripper[side]]

        # 發布出去供 Policy 監聽 Observation
        if side == "left":
            self.virtual_joint_pub_left.publish(virtual_joint_msg)
        else:
            self.virtual_joint_pub_right.publish(virtual_joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointToEEConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
