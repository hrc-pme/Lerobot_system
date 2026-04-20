#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import sys
import os
import threading

class DatasetReplayBridge(Node):
    def __init__(self, bag_path):
        super().__init__('dataset_replay_bridge')
        
        # 建立與 mock_cross_robot.py 一樣的 Koch 虛擬硬體回放 (把 bag 的控制當成動作轉發給 RViz)
        self.create_subscription(JointState, "/left_follower/raw_joint_states", self.left_cb, 10)
        self.create_subscription(JointState, "/right_follower/raw_joint_states", self.right_cb, 10)
        
        # 發布校正後的 joint state 給 FK (joint_to_ee_converter.py)
        self.koch_left_ctrl_pub = self.create_publisher(JointState, "/left_follower/joint_states_control", 10)
        self.koch_right_ctrl_pub = self.create_publisher(JointState, "/right_follower/joint_states_control", 10)
        
        # OpenArm 虛擬硬體回放 (模擬真實 OpenArm 接到指令後到達，回傳狀態讓模擬的 OpenArm 在 RViz 裡面也能跟著跑)
        self.create_subscription(JointState, "/openarm/joint_commands", self.openarm_cmd_cb, 10)
        
        self.openarm_state_pub = self.create_publisher(JointState, "/openarm/joint_states", 10)
        self.rviz_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        
        self.last_left_msg = None
        self.last_right_msg = None
        
        # dataset的關節名稱對應到 URDF 及 需要的偏移與反轉校正
        # 這邊取自 real_robot_remapper.py 的校正參數：('對應名稱', 方向, 偏移量)
        self.mapping = {
            'shoulder_pan':  ('joint1', -1, 0.0),
            'shoulder_lift': ('joint2', -1, 1.05), 
            'elbow_flex':    ('joint3', 1, 1.57),
            'wrist_flex':    ('joint4', -1, 0.0),
            'wrist_roll':    ('joint5', 1, 0.0),
            'gripper':       ('joint_gripper', 1, 0.5)
        }

        self.get_logger().info(f"Dataset Replay Bridge Started. Will play: {bag_path}")

    def process_joint_msg(self, msg, prefix):
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = []
        new_msg.position = []
        
        for i, name in enumerate(msg.name):
            for key, (target_name, direction, offset) in self.mapping.items():
                if name.endswith(key):
                    new_msg.name.append(f"{prefix}{target_name}")
                    if i < len(msg.position):
                        val = msg.position[i]
                        new_msg.position.append((val * direction) + offset)
                    break
        return new_msg

    def left_cb(self, msg):
        self.last_left_msg = self.process_joint_msg(msg, "left_")
        self.koch_left_ctrl_pub.publish(self.last_left_msg)
        self.publish_koch_rviz()

    def right_cb(self, msg):
        self.last_right_msg = self.process_joint_msg(msg, "right_")
        self.koch_right_ctrl_pub.publish(self.last_right_msg)
        self.publish_koch_rviz()

    def publish_koch_rviz(self):
        # 兩隻手都收到第一筆資料後，就能把這當作組合好的一個人體狀態一起送給 joint_states 以供機器人 RViz 模型更新
        if not self.last_left_msg or not self.last_right_msg:
            return
            
        combined_viz_msg = JointState()
        combined_viz_msg.header.stamp = self.get_clock().now().to_msg()
        combined_viz_msg.name = list(self.last_left_msg.name) + list(self.last_right_msg.name)
        combined_viz_msg.position = list(self.last_left_msg.position) + list(self.last_right_msg.position)
        self.rviz_state_pub.publish(combined_viz_msg)
        # self.get_logger().info("Published Koch to RViz")

    def openarm_cmd_cb(self, msg):
        """假裝 OpenArm 模擬機已經到達指定位址，回傳結果。把運算出來的 OpenArm 控制也一起發布給 RViz 顯示。"""
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = msg.name
        state_msg.position = msg.position
        self.openarm_state_pub.publish(state_msg)
        self.rviz_state_pub.publish(state_msg)


def play_bag(bag_path):
    print(f"\n[Bag Player] Starting ros2 bag play...")
    # 播放 control topics 並重新對應到 raw，讓 python node 攔截做校正
    cmd = (f"ros2 bag play {bag_path} "
           f"--remap /left_follower/joint_states_control:=/left_follower/raw_joint_states "
           f"/right_follower/joint_states_control:=/right_follower/raw_joint_states")
    try:
        proc = subprocess.Popen(cmd, shell=True)
        # 不斷檢查是否提早結束，或是拋出錯誤
        stdout, stderr = proc.communicate()
        print(f"\n[Bag Player] Playback finished! Return code: {proc.returncode}")
    except Exception as e:
        print(f"\n[Bag Player] Failed: {e}")
    finally:
        os._exit(0)


def main(args=sys.argv):
    rclpy.init(args=args)
    
    # 使用者可以藉由命令列參數傳入其他 bag，預設則是你要求的 0005 這個 bag
    bag_path = "/home/hrc/Lerobot_system/Dataset/bags/0320_wipewuthTissue/0005"
    if len(args) > 1:
        bag_path = args[1]
        
    if not os.path.exists(bag_path):
        print(f"Error: Bag path '{bag_path}' does not exist!")
        return

    node = DatasetReplayBridge(bag_path)
    
    # 啟動背後的 ros2 bag play 執行緒
    bag_thread = threading.Thread(target=play_bag, args=(bag_path,), daemon=True)
    bag_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
