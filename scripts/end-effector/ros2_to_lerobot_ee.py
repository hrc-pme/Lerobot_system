#!/usr/bin/env python3
"""
ROS2 Bag to LeRobot Dataset Converter with End-Effector Space
Usage: python3 scripts/ros2_to_lerobot_ee.py --config scripts/ros2_to_lerobot_ee_config.yaml
"""

import os
import sys
import yaml
import argparse
import numpy as np
import cv2
import rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, CompressedImage, JointState
from cv_bridge import CvBridge
import torch
import shutil

# Add paths
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, "end-effector"))

# Try to import robust_pino_kinematics
try:
    from robust_pino_kinematics import RobotKinematics
except ImportError:
    print("Error: Could not import robust_pino_kinematics. Make sure scripts/end-effector/robust_pino_kinematics.py exists.")
    sys.exit(1)

# Add LeRobot to path
lerobot_path = os.path.expanduser("~/Lerobot_system/repos/lerobot/src")
if os.path.exists(lerobot_path):
    sys.path.append(lerobot_path)
else:
    print(f"Warning: LeRobot path {lerobot_path} not found. Ensure LeRobot is installed.")

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError:
    print("Error: Could not import LeRobotDataset. Is lerobot installed?")
    sys.exit(1)

def get_urdf_path(side):
    # Try common locations
    possible_paths = [
        f"/tmp/koch_{side}.urdf",
        f"koch_{side}.urdf",
        f"../koch_{side}.urdf"
    ]
    for p in possible_paths:
        if os.path.exists(p):
            return p
    print(f"Error: URDF file for {side} arm not found. Please run xacro first.")
    return None

class BagConverterEE:
    def __init__(self, config_path):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.bridge = CvBridge()
        self.fps = self.config['dataset']['fps']
        self.interval = 1.0 / self.fps
        
        # Prepare output path
        if 'output' in self.config and 'path' in self.config['output']:
            self.root_path = Path(self.config['output']['path']).expanduser()
        else:
            self.root_path = Path(self.config['dataset']['root']).expanduser()

        if self.root_path.exists():
            print(f"Warning: Output directory {self.root_path} exists. Cleaning it up...")
            shutil.rmtree(self.root_path)

        # Ensure parent directory exists
        self.root_path.parent.mkdir(parents=True, exist_ok=True)
            
        # Parse features
        self.features = self._build_features()
        
        # Initialize Kinematics Models
        self.kin_models = {}
        urdf_l = get_urdf_path("left")
        urdf_r = get_urdf_path("right")
        
        if urdf_l:
            print(f"Loading Left Arm Model from {urdf_l}")
            self.kin_models['left'] = RobotKinematics(urdf_l)
        if urdf_r:
            print(f"Loading Right Arm Model from {urdf_r}")
            self.kin_models['right'] = RobotKinematics(urdf_r)

        # Mapping for bag joints (from topic) to URDF joints
        self.bag_to_urdf_map_left = {
            'left_follower_shoulder_pan': 'left_joint1',
            'left_follower_shoulder_lift': 'left_joint2',
            'left_follower_elbow_flex': 'left_joint3',
            'left_follower_wrist_flex': 'left_joint4',
            'left_follower_wrist_roll': 'left_joint5',
        }
        
        self.bag_to_urdf_map_right = {
            'right_follower_shoulder_pan': 'right_joint1',
            'right_follower_shoulder_lift': 'right_joint2',
            'right_follower_elbow_flex': 'right_joint3',
            'right_follower_wrist_flex': 'right_joint4',
            'right_follower_wrist_roll': 'right_joint5',
        }

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
            
        # State (EE Pose 14 dim)
        state_dim = self.config['robot']['state_dim']
        features[self.config['mapping']['state']['key']] = {
            "dtype": "float32",
            "shape": (state_dim,),
            "names": [f"state_{i}" for i in range(state_dim)],
        }
        
        # Action (EE Pose 14 dim)
        action_dim = self.config['robot']['action_dim']
        features[self.config['mapping']['action']['key']] = {
            "dtype": "float32",
            "shape": (action_dim,),
            "names": [f"action_{i}" for i in range(action_dim)],
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
                'dim': topic['dim'],
                'arm': topic.get('arm', None)
            }
            current_idx += topic['dim']
        
        # Action
        action_conf = self.config['mapping']['action']
        self.data_buffers[action_conf['key']] = np.zeros(self.config['robot']['action_dim'], dtype=np.float32)
        self.action_fill_status = {t['name']: False for t in action_conf['topics']}

        current_idx = 0
        for topic in action_conf['topics']:
            # Make sure not to overwrite existing map entry if topic is same as state
            # If topic exists, update it to be dual-purpose or handle in loop
            # But dict keys are unique. If state and action use same topic, we need a list of handlers
            # Current structure: self.topic_map[topic] = {...}
            # We fix this by making the value a list of targets
            
            target_info = {
                'type': 'action', 
                'key': action_conf['key'],
                'start': current_idx,
                'end': current_idx + topic['dim'],
                'dim': topic['dim'],
                'arm': topic.get('arm', None)
            }
            
            if topic['name'] in self.topic_map:
                if isinstance(self.topic_map[topic['name']], list):
                    self.topic_map[topic['name']].append(target_info)
                else:
                    # convert existing dict to list
                    self.topic_map[topic['name']] = [self.topic_map[topic['name']], target_info]
            else:
                self.topic_map[topic['name']] = [target_info]
                
            current_idx += topic['dim']

        # Ensure all existing dict entries are lists for consistency
        for k, v in self.topic_map.items():
            if not isinstance(v, list):
                self.topic_map[k] = [v]

    def process_image(self, msg, compressed=False):
        if compressed:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def get_q_for_model(self, joint_map_msg, bag_to_urdf_map, kin_solver):
        q = np.zeros(kin_solver.model.nq)
        for i, name in enumerate(kin_solver.model.names):
            if name == 'universe': continue
            
            bag_key = None
            for b_key, u_name in bag_to_urdf_map.items():
                if u_name == name:
                    bag_key = b_key
                    break
            
            if bag_key and bag_key in joint_map_msg:
                joint_id = kin_solver.model.getJointId(name)
                idx_q = kin_solver.model.joints[joint_id].idx_q
                if 0 <= idx_q < len(q):
                    q[idx_q] = joint_map_msg[bag_key]
        return q

    def compute_ee_pose(self, msg, arm):
        if arm not in self.kin_models:
            print(f"Warning: No kinematics model for {arm}")
            return None
            
        kin = self.kin_models[arm]
        name_map = dict(zip(msg.name, msg.position))
        
        mapping = self.bag_to_urdf_map_left if arm == 'left' else self.bag_to_urdf_map_right
        
        # Check if we have enough matching joints
        matches = sum(1 for k in mapping.keys() if k in name_map)
        if matches < 3: # arbitrary threshold to say "this message contains our arm data"
            return None
            
        q = self.get_q_for_model(name_map, mapping, kin)
        pos, rot = kin.forward_kinematics(q)

        # Ensure rotation is valid quaternion [x,y,z,w]
        # Robust kinematics already returns it, but let's double check simple normalization
        norm = np.linalg.norm(rot)
        if norm > 1e-6:
            rot = rot / norm
        else:
            rot = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Get Gripper Value
        # Assuming the gripper joint has 'gripper' in its name
        gripper_val = 0.0
        found_gripper = False
        
        # Try specific names first
        prefix = f"{arm}_follower"
        gripper_names = [f"{prefix}_gripper", "gripper"]
        
        for g_name in gripper_names:
            if g_name in name_map:
                gripper_val = name_map[g_name]
                found_gripper = True
                break
        
        # If not found, search for any key with 'gripper'
        if not found_gripper:
            for name, val in name_map.items():
                if 'gripper' in name:
                    gripper_val = val
                    found_gripper = True
                    break

        # Return concatenated [x,y,z, qx,qy,qz,qw] (7 dims) + gripper (1 dim) => 8 dims
        # Note: rot from robust_pino_kinematics is [x,y,z,w] or whatever pinocchio returns
        return np.concatenate([pos, rot, [gripper_val]]).astype(np.float32)

    def enforce_quaternion_continuity(self, current_quat, prev_quat):
        """
        Ensure potential quaternion flip does not cause discontinuity.
        Pinocchio/Rotation convention: q and -q represent the same rotation.
        If dot product < 0, flip the sign of current_quat.
        Input: [x, y, z, w] (or any order, as long as consistent)
        """
        if prev_quat is None:
            return current_quat
        
        # Calculate dot product
        dot = np.dot(current_quat, prev_quat)
        
        if dot < 0:
            return -current_quat
        return current_quat

    def run(self):
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
        
        bag_paths = []
        if (base_path / "metadata.yaml").exists():
            bag_paths = [base_path]
        else:
            bag_paths = sorted([p for p in base_path.iterdir() if p.is_dir() and (p / "metadata.yaml").exists()])
            
        if not bag_paths:
            print(f"Error: No valid ROS2 bags found in {base_path}")
            return

        print(f"Found {len(bag_paths)} bags to process.")
        
        total_frames = 0
        
        for episode_idx, bag_path in enumerate(bag_paths):
            print(f"Processing Episode {episode_idx}: {bag_path.name}")
            
            self._setup_mappings()
            
            # --- Per-Episode State Tracking for Continuity ---
            # Use a dictionary to track previous rotation PER FEATURE (observation vs action)
            # Keys: 'feature_name_arm' or just '{feature_key}_{arm}'
            prev_rots = {} 
            # -----------------------------------------------
            
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
                if topic not in self.topic_map:
                    continue
                    
                target_infos = self.topic_map[topic]
                if not isinstance(target_infos, list):
                    target_infos = [target_infos]
                    
                for map_info in target_infos:
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
                            
                        elif map_info['type'] in ['state', 'action']:
                            # Compute EE Pose
                            msg = deserialize_message(data, JointState)
                            arm = map_info.get('arm')
                            val = self.compute_ee_pose(msg, arm)
                            
                            if val is not None:
                                # Process Quaternion Continuity
                                # val structure: [x,y,z, qx,qy,qz,qw, gripper] (8 dims)
                                pos = val[:3]
                                quart_xyzw = val[3:7] # [qx, qy, qz, qw]
                                grip = val[7:]

                                # We need to check continuity based on dot product
                                # Store previous rotation for this specific arm/topic combination?
                                # The loop iterates through topics.
                                # Use unique key for continuity tracking based on feature key and arm
                                continuity_key = f"{key}_{arm}"
                                
                                rot_to_store = quart_xyzw.copy()
                                
                                if continuity_key in prev_rots:
                                    prev_rot = prev_rots[continuity_key]
                                    # Check direction vs previous
                                    if np.dot(rot_to_store, prev_rot) < 0:
                                        rot_to_store = -rot_to_store # Flip
                                
                                prev_rots[continuity_key] = rot_to_store
                                
                                # Re-assemble using the potentially flipped rotation
                                val_processed = np.concatenate([pos, rot_to_store, grip]).astype(np.float32)

                                target_slice = slice(map_info['start'], map_info['end'])
                                self.data_buffers[key][target_slice] = val_processed
                                
                                if map_info['type'] == 'state':
                                    self.state_fill_status[topic] = True
                                else:
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
        print("Finalizing dataset...")
        dataset.finalize()
        print(f"Success! Dataset saved to: {self.root_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="scripts/ros2_to_lerobot_ee_config.yaml", help="Path to config file")
    args = parser.parse_args()
    
    converter = BagConverterEE(args.config)
    converter.run()
