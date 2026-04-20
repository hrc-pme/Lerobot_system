#!/usr/bin/env python3
import sys
import os
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

# 加入 robust_pino_kinematics 的路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, ".."))

from robust_pino_kinematics import RobotKinematics
from scipy.spatial.transform import Rotation as R

# OpenArm URDF 路徑 (請確認這個路徑正確指向你的 URDF)
OPENARM_URDF_PATH = "/home/hrc/Lerobot_system/repos/openarm_dev/openarm_urdf/openarm_bimanual.urdf"

class OpenArmEEBridge(Node):
    def __init__(self):
        super().__init__('openarm_ee_bridge')
        
        # --- 1. 載入 OpenArm 模型 ---
        self.kin_models = {}
        if os.path.exists(OPENARM_URDF_PATH):
            self.get_logger().info("Loading OpenArm Kinematics Model...")
            # 針對左右手分別初始化，並指定手腕為末端點 (與 Koch URDF 保持相同定義點)
            self.kin_models['left'] = RobotKinematics(OPENARM_URDF_PATH, tip_link="openarm_left_hand")
            self.kin_models['right'] = RobotKinematics(OPENARM_URDF_PATH, tip_link="openarm_right_hand")
            
            # 從模型中動態獲取關節總數 (這通常會是 18，因為 URDF 是雙臂合一的)
            self.nq = self.kin_models['left'].model.nq
            self.get_logger().info(f"OpenArm has {self.nq} degrees of freedom in total.")
        else:
            self.get_logger().error(f"OpenArm URDF not found at {OPENARM_URDF_PATH}")
            sys.exit(1)

        # 紀錄「整個機台 (18 軸)」的狀態，供 IK 當初值，因為 pinocchio 需要吃整包
        # 我們只共用一份 complete_q，無論是左手還是右手的 callback 都會更新並丟這份給 IK
        self.complete_q = np.zeros(self.nq)
        
        # --- 2. 訂閱者 (監聽轉換器發送的目標 Target EE 座標) 【去程】---
        self.create_subscription(PoseStamped, "/left_follower/ee_pose_cmd", lambda msg: self.target_pose_cb(msg, "left"), 10)
        self.create_subscription(PoseStamped, "/right_follower/ee_pose_cmd", lambda msg: self.target_pose_cb(msg, "right"), 10)
        
        # 夾爪訂閱
        self.create_subscription(Float32, "/left_follower/gripper_cmd", lambda msg: self.target_gripper_cb(msg, "left"), 10)
        self.create_subscription(Float32, "/right_follower/gripper_cmd", lambda msg: self.target_gripper_cb(msg, "right"), 10)

        # --- 3. 發布者 (推播給 OpenArm 底層的 Joint 指令) 【去程】---
        self.openarm_joint_pub = self.create_publisher(JointState, "/openarm/joint_commands", 10)

        # --- 4. 訂閱者與發布者 (監聽 OpenArm 實體狀態，打包成 Actual EE) 【回程】---
        # 這邊假設底下會發布真實的 "/openarm/joint_states"
        self.create_subscription(JointState, "/openarm/joint_states", self.actual_joint_cb, 10)
        
        self.actual_ee_pub_left = self.create_publisher(PoseStamped, "/left_follower/actual_ee_pose", 10)
        self.actual_ee_pub_right = self.create_publisher(PoseStamped, "/right_follower/actual_ee_pose", 10)

        self.get_logger().info("OpenArm EE Bridge is Ready!")

    def target_pose_cb(self, msg, side):
        """【去程】接到空間座標 -> 轉成 OpenArm 7軸 Joint指令"""
        
        koch_local_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        target_rot = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        # 1. 轉成 World 共用座標 (Koch 本身 Yaw=0，只有平移)
        z_offset = 0.33
        if side == 'left':
            world_pos = koch_local_pos + np.array([-0.18, 0.3, z_offset])
        else:
            world_pos = koch_local_pos + np.array([0.18, 0.3, z_offset])
        world_rot_R = R.from_quat(target_rot)
        
        # 1. 旋轉轉換矩陣
        # openarm 和 koch 對於 X/Y 平面的定義若有正反顛倒
        # 若發生左右向位移剛好反向，就在這裡加一個旋轉轉回來 (例如 yaw: 180度, X軸鏡像)
        # 你的問題是 "右手伸出去向右時 openarm右手是向左"，這代表 Y軸(或X軸)的座標符號是相反的。
        
        # 由於 Koch Local 轉到 World，再從 World 轉入 OpenArm IK 時
        # 2. 將 World 座標轉成 OpenArm 內部座標
        r_world_to_oa = R.from_euler('xyz', [0, 0, -1.5708]) # "去程" 的旋轉要與 shell 的 "yaw" (1.5708) 剛好相反: 轉 -90 度
        
        # 3. 修正末端點 (End-effector) 兩台機器人 URDF 在手腕座標系定義的差異
        # 從圖片看，Koch 的夾爪是水平開合，OpenArm 是垂直開合，代表兩者的末端座標軸差了 90 度
        # 在這裡做 Local Rotation (右乘)，你可以直接修改這裡的數值來微調夾爪的旋轉！
        # 例如若是 Roll 差 90 度：[1.5708, 0, 0] 或 Yaw: [0, 0, 1.5708]
        r_ee_correction = R.from_euler('xyz', [2.5, 0.0, 1.5708]) # 測試旋轉 90 度
        
        oa_pos = r_world_to_oa.apply(world_pos)
        oa_rot = r_world_to_oa * world_rot_R * r_ee_correction

        try:
            # 呼叫逆向運動學
            q_target = self.kin_models[side].inverse_kinematics_5dof(
                oa_pos, oa_rot.as_quat(), q_init=self.complete_q, max_iter=100
            ) 
            
            # 更新全域狀態
            self.complete_q = q_target

            # 發布出去
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
            
            target_angles = []
            for name in joint_msg.name:
                joint_id = self.kin_models[side].model.getJointId(name)
                idx_q = self.kin_models[side].model.joints[joint_id].idx_q
                target_angles.append(q_target[idx_q])
            
            joint_msg.position = target_angles
            self.openarm_joint_pub.publish(joint_msg)

        except Exception as e:
            self.get_logger().error(f"OpenArm IK Failed: {e}")

    def target_gripper_cb(self, msg, side):
        """處理夾爪控制，做線性映射"""
        koch_val = msg.data
        gripper_openarm_val = np.clip(koch_val * 0.044, 0.0, 0.044) 
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"openarm_{side}_finger_joint1"]
        joint_msg.position = [float(gripper_openarm_val)]
        self.openarm_joint_pub.publish(joint_msg)

    def actual_joint_cb(self, msg):
        """【回程】接聽 OpenArm 真實各軸角度 -> 轉成實際空間座標 -> 丟給 Koch 轉換器"""
        q_current = np.zeros(self.kin_models['left'].model.nq)
        
        for idx, name in enumerate(msg.name):
            if self.kin_models['left'].model.existJointName(name):
                joint_id = self.kin_models['left'].model.getJointId(name)
                idx_q = self.kin_models['left'].model.joints[joint_id].idx_q
                q_current[idx_q] = msg.position[idx]

        for side in ['left', 'right']:
            pos, rot = self.kin_models[side].forward_kinematics(q_current)
            
            # 從 OpenArm 內部座標轉出到 World 座標 (對應上面，這裡要跟 shell 的 Yaw 完全一致):
            r_oa_to_world = R.from_euler('xyz', [0, 0, 1.5708]) # "回程" 要與 shell 的 "yaw" (1.5708) 完全一樣: 轉 90 度
            world_pos = r_oa_to_world.apply(pos)
            world_rot_R = r_oa_to_world * R.from_quat(rot)
            
            # 從 World 座標轉回 Koch Local 座標 (扣除平移):
            z_offset = 0.33  # 必須與上面保持一致
            if side == 'left':
                koch_local_pos = world_pos - np.array([-0.18, 0.3, z_offset])
            else:
                koch_local_pos = world_pos - np.array([0.18, 0.3, z_offset])
            
            q_koch = world_rot_R.as_quat()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = f"{side}_base_link"
            pose_msg.pose.position.x = float(koch_local_pos[0])
            pose_msg.pose.position.y = float(koch_local_pos[1])
            pose_msg.pose.position.z = float(koch_local_pos[2])
            pose_msg.pose.orientation.x = float(q_koch[0])
            pose_msg.pose.orientation.y = float(q_koch[1])
            pose_msg.pose.orientation.z = float(q_koch[2])
            pose_msg.pose.orientation.w = float(q_koch[3])

            if side == 'left':
                self.actual_ee_pub_left.publish(pose_msg)
            else:
                self.actual_ee_pub_right.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenArmEEBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
