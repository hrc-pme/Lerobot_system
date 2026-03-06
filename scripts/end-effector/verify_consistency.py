#!/usr/bin/env python3
import sys
import os
import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import csv
import matplotlib.pyplot as plt

# Add path for 'pin' package if installed via cmeel
cmeel_path = "/usr/local/lib/python3.10/dist-packages/cmeel.prefix/lib/python3.10/site-packages"
if os.path.exists(cmeel_path):
    # Insert at beginning to override system pinocchio
    sys.path.insert(0, cmeel_path)
    print(f"Added {cmeel_path} to sys.path")

# Local imports
# Adjust path to find sibling scripts
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from robust_pino_kinematics import RobotKinematics
except ImportError:
    # Try adding parent directory if running from script dir
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
    from robust_pino_kinematics import RobotKinematics

def get_urdf_path(side):
    path = f"/tmp/koch_{side}.urdf"
    if not os.path.exists(path):
        # Allow failing later if init fails
        pass 
    return path

class ConsistencyVerifier:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        
        # Init Kinematics
        print("Initializing Pinocchio Models...")
        self.kin_left = RobotKinematics(get_urdf_path("left"))
        self.kin_right = RobotKinematics(get_urdf_path("right"))
        
        # Debugging model structure
        names_list = []
        for i, n in enumerate(self.kin_left.model.names):
            names_list.append(n)
        print(f"Left Model names: {names_list}")
        print(f"Left Model nq: {self.kin_left.model.nq}")
        
        # Mappings (Robot Name -> URDF Joint Name)
        # Note: RobotKinematics expects q in order of model.joints (excluding universe)
        # We need to map from bag dict to this vector.
        
        # Left Map
        self.map_left = {
            'left_follower_shoulder_pan': 'left_joint1',
            'left_follower_shoulder_lift': 'left_joint2',
            'left_follower_elbow_flex': 'left_joint3',
            'left_follower_wrist_flex': 'left_joint4',
            'left_follower_wrist_roll': 'left_joint5',
            'left_follower_gripper': 'left_joint_gripper'
        }
        
        # Right Map
        self.map_right = {
            'right_follower_shoulder_pan': 'right_joint1',
            'right_follower_shoulder_lift': 'right_joint2',
            'right_follower_elbow_flex': 'right_joint3',
            'right_follower_wrist_flex': 'right_joint4',
            'right_follower_wrist_roll': 'right_joint5',
            'right_follower_gripper': 'right_joint_gripper'
        }

    def extract_q(self, msg, side):
        # msg: JointState
        # side: 'left' or 'right'
        
        param_map = self.map_left if side == 'left' else self.map_right
        model = self.kin_left.model if side == 'left' else self.kin_right.model
        
        msg_dict = dict(zip(msg.name, msg.position))
        q = []
        
        # Iterate over model joints
        for name in model.names:
            if name == 'universe': continue
            
            # Find which robot joint maps to this URDF joint
            # param_map is Robot -> URDF
            # We need URDF -> Robot to look up in msg_dict
            
            # Slow reverse lookup, do it once in init ideally, but fine here
            robot_key = None
            for r_key, u_val in param_map.items():
                if u_val == name:
                    robot_key = r_key
                    break
            
            if robot_key and robot_key in msg_dict:
                q.append(msg_dict[robot_key])
            elif 'gripper' in name:
                 # Try common gripper names
                 if side == 'left' and 'left_follower_gripper' in msg_dict:
                     q.append(msg_dict['left_follower_gripper'])
                 elif side == 'right' and 'right_follower_gripper' in msg_dict:
                     q.append(msg_dict['right_follower_gripper'])
                 else:
                     q.append(0.0)
            else:
                q.append(0.0)
                
        return np.array(q)

    def run(self, output_csv="consistency_results.csv"):
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        try:
            reader.open(storage_options, converter_options)
        except Exception as e:
            print(f"Failed to open bag with sqlite3: {e}. Trying mcap.")
            storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='mcap')
            reader.open(storage_options, converter_options)

        topics = [t.name for t in reader.get_all_topics_and_types()]
        
        # Data storage
        # errors: list of Norm(q_diff)
        errors_left = []
        errors_right = []
        
        # State for temporal coherence (warm start)
        q_left_prev = np.zeros(self.kin_left.dof)
        q_right_prev = np.zeros(self.kin_right.dof)
        
        count = 0
        
        with open(output_csv, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Side', 'RMSE_Joint', 'Max_Joint_Diff', 'Pos_Err', 'Rot_Err', 'Solve_Time'])
            
            print("Processing messages...")
            while reader.has_next():
                (topic, data, t) = reader.read_next()
                
                if 'joint_states' in topic:
                    msg = deserialize_message(data, JointState)
                    
                    if 'left' in topic:
                        side = 'left'
                        kin = self.kin_left
                        q_prev = q_left_prev
                    elif 'right' in topic:
                        side = 'right'
                        kin = self.kin_right
                        q_prev = q_right_prev
                    else:
                        continue
                        
                    # 1. Get GT Joint Config
                    q_gt = self.extract_q(msg, side)
                    if len(q_gt) != kin.dof:
                        # Mismatch, skip
                        continue

                    # 2. Compute FK -> EE Pose
                    pos, rot = kin.forward_kinematics(q_gt)
                    
                    # 3. Compute IK -> q_pred
                    # Correct Simulation Strategy:
                    # Frame 0: Initialize with Ground Truth (Simulate reading current robot state at startup)
                    # Frame 1+: Use previous prediction (Warm Start)
                    
                    if np.all(q_prev == 0): # First valid frame for this arm
                        q_seed = q_gt # Init with sensor data
                    else:
                        q_seed = q_prev # Warm start
                        
                    start_solve = time.time()
                    q_pred = kin.inverse_kinematics_5dof(pos, rot, q_seed)
                    solve_time = time.time() - start_solve
                    
                    # update prev
                    if side == 'left':
                        q_left_prev = q_pred
                    else:
                        q_right_prev = q_pred
                        
                    # 4. Compute Errors (Task Space & Joint Space)
                    
                    # 4A. Joint Error
                    joint_dim = 5
                    diff = (q_gt[:joint_dim] - q_pred[:joint_dim])
                    rmse = np.sqrt(np.mean(diff**2))
                    max_diff = np.max(np.abs(diff))

                    # 4B. Task Space Confirmation (Verify if IK reached the desired Pose)
                    pred_pos, pred_rot = kin.forward_kinematics(q_pred)
                    pos_err = np.linalg.norm(np.array(pred_pos) - np.array(pos))
                    
                    # Quaternion dot product for angle error
                    # pinocchio rot is list/array
                    dot = np.abs(np.dot(pred_rot, rot))
                    if dot > 1.0: dot = 1.0
                    rot_err = 2.0 * np.arccos(dot)
                    
                    writer.writerow([t, side, rmse, max_diff, pos_err, rot_err, solve_time])
                    
                    if side == 'left':
                        errors_left.append((rmse, pos_err))
                    else:
                        errors_right.append((rmse, pos_err))
                        
                    count += 1
                    if count % 1000 == 0:
                        l_val = np.mean([x[0] for x in errors_left]) if errors_left else 0.0
                        print(f"Processed {count} messages. Mean RMSE L: {l_val:.4f}")

        print("Done.")
        if errors_left:
            rmses = [x[0] for x in errors_left]
            poses = [x[1] for x in errors_left]
            print(f"Left Arm - Mean RMSE: {np.mean(rmses):.4f}, Mean Pos Err: {np.mean(poses):.4f}")
            
        if errors_right:
            rmses = [x[0] for x in errors_right]
            poses = [x[1] for x in errors_right]
            print(f"Right Arm - Mean RMSE: {np.mean(rmses):.4f}, Mean Pos Err: {np.mean(poses):.4f}")

import time

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", help="Path to input bag (containing joint_states)")
    args = parser.parse_args()
    
    verifier = ConsistencyVerifier(args.bag_path)
    verifier.run()
