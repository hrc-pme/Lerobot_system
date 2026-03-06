#!/usr/bin/env python3
"""
Replay LeRobot Dataset to ROS2
功能：讀取轉換後的 LeRobot 資料集，並將 Action 發佈回 ROS2 Topic，讓您可以觀察機器人是否正確重現動作。
python3 replay_lerobot_dataset.py /home/hrc/Lerobot_system/Dataset/0303_clearwater_yellowtableconverted_dataset 0
"""

import sys
import time
import torch
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Add LeRobot to path
import os
sys.path.append(os.path.expanduser("~/Lerobot_system/repos/lerobot/src"))
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class DatasetReplayer(Node):
    def __init__(self, repo_id, root_path):
        super().__init__('lerobot_replayer')
        
        # 1. 載入資料集
        print(f"載入資料集: {root_path}")
        self.dataset = LeRobotDataset(repo_id=repo_id, root=root_path)
        print(f"資料集載入成功。 FPS: {self.dataset.fps}, Total Episodes: {self.dataset.num_episodes}")

        # 2. 建立 ROS2 Publisher (改為 _control topic 以驅動機器人)
        self.publisher_left = self.create_publisher(JointState, '/left_follower/joint_states_control', 10)
        self.publisher_right = self.create_publisher(JointState, '/right_follower/joint_states_control', 10)

        # 定義關節名稱 (對應 koch_follower_control 的順序)
        self.joint_names = [
            "shoulder_pan", 
            "shoulder_lift", 
            "elbow_flex", 
            "wrist_flex", 
            "wrist_roll", 
            "gripper"
        ]

    def replay(self, episode_index=0):
        if episode_index >= self.dataset.num_episodes:
            print(f"Episode {episode_index} 超出範圍 (總共 {self.dataset.num_episodes})")
            return

        print(f"開始重播 Episode {episode_index} ...")
        
        # 取得 Episode 起始與結束的 frame index
        # Fix for LeRobot v2.0+ structure
        episode_meta = self.dataset.meta.episodes[episode_index]
        start_idx = episode_meta['dataset_from_index']
        end_idx = episode_meta['dataset_to_index']
        
        print(f"Frame 範圍: {start_idx} -> {end_idx}")
        
        for i in range(start_idx, end_idx):
            start_time = time.time()
            
            # 從 Dataset 中讀取一個 Frame (是一個 dict)
            # 包含 'action', 'observation.state', 'observation.images.top' 等
            frame = self.dataset[i]
            
            # 取出 Action (Tensor shape: 12)
            # 這是當初錄製的 "Leader" 動作，現在我們發送給 Follower 重現
            action = frame['action'] 
            
            # 分割左右手 (前6是左手, 後6是右手)
            # 注意：這裡假設您的順序是先左後右，與 config 一致
            left_pos = action[:6].tolist()
            right_pos = action[6:].tolist()
            
            # 發佈左手命令
            msg_left = JointState()
            msg_left.header.stamp = self.get_clock().now().to_msg()
            
            # 使用正確的關節名稱 (必須包含 left_follower_ 前綴)
            msg_left.name = [f"left_follower_{n}" for n in self.joint_names]
            msg_left.position = left_pos
            self.publisher_left.publish(msg_left)
            
            # 發佈右手命令
            msg_right = JointState()
            msg_right.header.stamp = self.get_clock().now().to_msg()
            
            # 使用正確的關節名稱 (必須包含 right_follower_ 前綴)
            msg_right.name = [f"right_follower_{n}" for n in self.joint_names]
            msg_right.position = right_pos
            self.publisher_right.publish(msg_right)
            
            print(f"Frame {i}: Sent Action Left={left_pos[:2]}... Right={right_pos[:2]}...", end='\r')
            
            # 控制 FPS
            elapsed = time.time() - start_time
            sleep_time = max(0, (1.0 / self.dataset.fps) - elapsed)
            time.sleep(sleep_time)

        print("\n重播結束。")

def main(args=None):
    rclpy.init(args=args)
    
    # 指向剛才轉換好的資料集
    repo_id = "koch_bi_test_dataset"
    episode_index = 0
    
    if len(sys.argv) > 1:
        root_path = sys.argv[1]
    else:
        root_path = "/home/hrc/Lerobot_system/Dataset/converted_dataset"
    
    if len(sys.argv) > 2:
        try:
            episode_index = int(sys.argv[2])
        except ValueError:
            print("Usage: python3 replay_lerobot_dataset.py [dataset_path] [episode_index]")
            sys.exit(1)
            
    print(f"Using dataset path: {root_path}")
    print(f"Replaying episode: {episode_index}")
    
    node = DatasetReplayer(repo_id, root_path)
    
    try:
        node.replay(episode_index=episode_index)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
