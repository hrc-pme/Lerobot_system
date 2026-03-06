#!/usr/bin/env python3
"""
ROS2 Bag to LeRobot Dataset Converter
Usage: python3 ros2_to_lerobot.py --config ros2_to_lerobot_config.yaml
"""

import os
import sys
import yaml
import argparse
import numpy as np
import cv2
import rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CompressedImage, JointState
from cv_bridge import CvBridge
import torch
import shutil

# Add LeRobot to path
sys.path.append(os.path.expanduser("~/Lerobot_system/repos/lerobot/src"))
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class BagConverter:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.bridge = CvBridge()
        self.fps = self.config['dataset']['fps']
        self.interval = 1.0 / self.fps
        
        # Prepare output path
        self.root_path = Path(self.config['dataset']['root']).expanduser()
        if self.root_path.exists():
            print(f"Warning: Output directory {self.root_path} exists. Cleaning it up...")
            shutil.rmtree(self.root_path)
            
        # Parse features
        self.features = self._build_features()
        
        # Mapping helpers
        self.topic_map = {}
        self.data_buffers = {}
        self._setup_mappings()
        
    def _build_features(self):
        features = {}
        
        # Cameras
        for topic, cam_conf in self.config['mapping']['cameras'].items():
            features[cam_conf['key']] = {
                "dtype": "video",
                "shape": (3, cam_conf['height'], cam_conf['width']),
                "names": ["channel", "height", "width"],
            }
            
        # State
        state_dim = self.config['robot']['state_dim']
        features[self.config['mapping']['state']['key']] = {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": [f"joint_{i}" for i in range(state_dim)],
        }
        
        # Action
        action_dim = self.config['robot']['action_dim']
        features[self.config['mapping']['action']['key']] = {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": [f"joint_{i}" for i in range(action_dim)],
        }
        
        return features

    def _setup_mappings(self):
        # Cameras
        for topic, conf in self.config['mapping']['cameras'].items():
            self.topic_map[topic] = {'type': 'camera', 'key': conf['key']}
            self.data_buffers[conf['key']] = None
            
        # State
        state_conf = self.config['mapping']['state']
        self.data_buffers[state_conf['key']] = np.zeros(self.config['robot']['state_dim'], dtype=np.float32)
        self.state_fill_status = {t['name']: False for t in state_conf['topics']}
        
        current_idx = 0
        for topic in state_conf['topics']:
            self.topic_map[topic['name']] = {
                'type': 'state', 
                'key': state_conf['key'],
                'start': current_idx,
                'end': current_idx + topic['dim'],
                'dim': topic['dim']
            }
            current_idx += topic['dim']
        
        # Action
        action_conf = self.config['mapping']['action']
        self.data_buffers[action_conf['key']] = np.zeros(self.config['robot']['action_dim'], dtype=np.float32)
        self.action_fill_status = {t['name']: False for t in action_conf['topics']}

        current_idx = 0
        for topic in action_conf['topics']:
            self.topic_map[topic['name']] = {
                'type': 'action', 
                'key': action_conf['key'],
                'start': current_idx,
                'end': current_idx + topic['dim'],
                'dim': topic['dim']
            }
            current_idx += topic['dim']

    def process_image(self, msg, compressed=False):
        if compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
        # Resize if needed (optional, purely defensive)
        # Using configuration height/width
        # target_h = ...
        
        # Convert BGR to RGB (Important for LeRobot/PyTorch)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Convert to CHW is handled by LeRobot usually, but let's check input requirements
        # LeRobotDataset.add_frame expects HWC numpy array for images
        return img

    def process_joint(self, msg, dim):
        # Taking the first N joints from position
        # TODO: Implement joint name mapping if provided in config
        if len(msg.position) < dim:
            return None
        return np.array(msg.position[:dim], dtype=np.float32)

    def run(self):
        # Initialize Dataset
        print(f"Initializing LeRobot Dataset at {self.root_path}...")
        dataset = LeRobotDataset.create(
            repo_id=self.config['dataset']['repo_id'],
            fps=self.fps,
            robot_type=self.config['robot']['type'],
            features=self.features,
            root=self.root_path,
            image_writer_processes=4,
            image_writer_threads=2
        )
        
        base_path = Path(self.config['rosbag']['path']).expanduser()
        
        # Determine if input is a single bag or a directory of bags
        bag_paths = []
        if (base_path / "metadata.yaml").exists():
            bag_paths = [base_path]
        else:
            # Look for subdirectories that contain metadata.yaml
            bag_paths = sorted([p for p in base_path.iterdir() if p.is_dir() and (p / "metadata.yaml").exists()])
            
        if not bag_paths:
            print(f"Error: No valid ROS2 bags found in {base_path}")
            return

        print(f"Found {len(bag_paths)} bags to process.")
        
        total_frames = 0
        
        for episode_idx, bag_path in enumerate(bag_paths):
            print(f"Processing Episode {episode_idx}: {bag_path.name}")
            
            # Reset buffers for new episode
            self._setup_mappings()
            
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
            converter_options = rosbag2_py.ConverterOptions("", "")
            try:
                reader.open(storage_options, converter_options)
            except Exception as e:
                print(f"Skipping {bag_path.name} due to open error: {e}")
                continue
            
            topics_to_filter = list(self.topic_map.keys())
            storage_filter = rosbag2_py.StorageFilter(topics=topics_to_filter)
            reader.set_filter(storage_filter)
            
            last_save_time = 0
            start_time = None
            episode_frames = 0
            has_all_data = False
            
            while reader.has_next():
                topic, data, t_ns = reader.read_next()
                t_sec = t_ns / 1e9
                
                if start_time is None:
                    start_time = t_sec
                    last_save_time = t_sec
                
                # --- Update Buffer ---
                map_info = self.topic_map.get(topic)
                if not map_info:
                    continue
                    
                key = map_info['key']
                
                try:
                    if map_info['type'] == 'camera':
                        if "compressed" in topic:
                            msg = deserialize_message(data, CompressedImage)
                            img = self.process_image(msg, compressed=True)
                        else:
                            msg = deserialize_message(data, Image)
                            img = self.process_image(msg, compressed=False)
                        self.data_buffers[key] = img
                        
                    elif map_info['type'] == 'state':
                        msg = deserialize_message(data, JointState)
                        val = self.process_joint(msg, map_info['dim'])
                        if val is not None:
                            self.data_buffers[key][map_info['start']:map_info['end']] = val
                            self.state_fill_status[topic] = True
                            
                    elif map_info['type'] == 'action':
                        msg = deserialize_message(data, JointState)
                        val = self.process_joint(msg, map_info['dim'])
                        if val is not None:
                            self.data_buffers[key][map_info['start']:map_info['end']] = val
                            self.action_fill_status[topic] = True

                except Exception as e:
                    print(f"[Error] Failed to process message from {topic}: {e}")
                    continue

                # --- Check if we need to save a frame ---
                if (t_sec - last_save_time) >= self.interval:
                    if not has_all_data:
                        cameras_ready = all(v is not None for k, v in self.data_buffers.items() if "image" in k)
                        state_ready = all(self.state_fill_status.values())
                        action_ready = all(self.action_fill_status.values())
                        if cameras_ready and state_ready and action_ready:
                            has_all_data = True
                        else:
                            continue 
                    
                    frame = {
                        "task": self.config['dataset'].get('task', 'Do something')
                    }
                    
                    for k, v in self.data_buffers.items():
                        # We must copy current numpy array to avoid reference issues
                        if isinstance(v, np.ndarray):
                            frame[k] = v.copy()
                        elif isinstance(v, torch.Tensor):
                            frame[k] = v.clone()
                        else:
                            frame[k] = v
                    
                    dataset.add_frame(frame)
                    last_save_time = t_sec
                    episode_frames += 1
                    total_frames += 1

            print(f"  -> Saved {episode_frames} frames.")
            dataset.save_episode()

        print(f"\nProcessing complete. Total frames across all episodes: {total_frames}")
        print("Finalizing dataset (encoding videos)... this may take a while.")
        
        # Finalize (write metadata, etc)
        dataset.finalize()
        print(f"Success! Dataset saved to: {self.root_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="ros2_to_lerobot_config.yaml", help="Path to config file")
    args = parser.parse_args()
    
    converter = BagConverter(args.config)
    converter.run()
