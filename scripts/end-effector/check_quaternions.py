#!/usr/bin/env python3
import sys
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from lerobot.datasets.lerobot_dataset import LeRobotDataset

def check_dataset_consistency(repo_id, root_path):
    print(f"Loading LeRobot dataset: {repo_id} from {root_path}")
    dataset = LeRobotDataset(repo_id, root=root_path, tolerance_s=0.5) # tolerance needed if data has gaps? No, likely fine.
    
    print(f"Number of frames: {len(dataset)}")
    
    # Extract state (16 dim) typically
    # 0-2: Left Pos, 3-6: Left Quat, 7: Left Gripper
    # 8-10: Right Pos, 11-14: Right Quat, 15: Right Gripper
    
    states = []
    
    # Direct access to dataset.hf_dataset might be faster if large
    # dataset[i] returns a dict
    
    # Accelerated Loading: Use HuggingFace Dataset map/iter or DataLoader
    # LeRobotDataset wraps hf_dataset. We can access the underlying robust path.
    # Actually, we just need 'observation.state' and 'episode_index' (or 'index')
    
    print("Optimization: Extracting 'observation.state' and 'episode_index'...")
    try:
        hf_ds = dataset.hf_dataset
        states = np.array(hf_ds['observation.state'])
        
        # LeRobot dataset usually has 'episode_index' column
        if 'episode_index' in hf_ds.column_names:
            episode_indices = np.array(hf_ds['episode_index'])
        else:
            # Fallback: try to reconstruct from 'index' or assume single episode if not found
            # But usually it's there.
            episode_indices = np.zeros(len(states))
            print("Warning: 'episode_index' column not found, assuming single episode.")
            
        print(f"State shape: {states.shape}")
        
    except Exception as e:
        print(f"Direct access failed ({e}), falling back to iterative loader...")
        states = []
        try:
            from tqdm import tqdm
            iterator = tqdm(range(len(dataset)), desc="Loading frames")
        except ImportError:
            iterator = range(len(dataset))
            print("tqdm not installed, simple progress logging...")

        for i in iterator:
            if not isinstance(iterator, tqdm) and i % 1000 == 0:
                print(f"Loading frame {i}/{len(dataset)}...")
                
            item = dataset[i]
            states.append(item['observation.state'].numpy())
            
        states = np.array(states)
        print(f"State shape: {states.shape}")
    
    # Plot Quaternions for Left Arm
    plt.figure(figsize=(12, 10))
    
    # Left Quaternions (indices 3,4,5,6)
    plt.subplot(2, 2, 1)
    plt.plot(states[:, 3], label='qx')
    plt.plot(states[:, 4], label='qy')
    plt.plot(states[:, 5], label='qz')
    plt.plot(states[:, 6], label='qw')
    plt.title("Left Arm Rotation (Quaternion)")
    plt.legend()
    plt.grid(True)

    # Right Quaternions (indices 11,12,13,14)
    plt.subplot(2, 2, 2)
    plt.plot(states[:, 11], label='qx')
    plt.plot(states[:, 12], label='qy')
    plt.plot(states[:, 13], label='qz')
    plt.plot(states[:, 14], label='qw')
    plt.title("Right Arm Rotation (Quaternion)")
    plt.legend()
    plt.grid(True)
    
    # Check for flips/jumps
    # Diff of quaternions
    q_left = states[:, 3:7]
    diffs_l = np.linalg.norm(q_left[1:] - q_left[:-1], axis=1)
    
    plt.subplot(2, 2, 3)
    plt.plot(diffs_l)
    plt.title("Left Quaternion Delta Norm (Jumps?)")
    plt.grid(True)
    
    q_right = states[:, 11:15]
    diffs_r = np.linalg.norm(q_right[1:] - q_right[:-1], axis=1)
    
    plt.subplot(2, 2, 4)
    plt.plot(diffs_r)
    plt.title("Right Quaternion Delta Norm (Jumps?)")
    plt.grid(True)
    
    plt.tight_layout()
    output_img = "quaternion_check.png"
    plt.savefig(output_img)
    print(f"Plot saved to {output_img}. Please check it for vertical lines (jumps).")
    
    # Statistical Check
    threshold = 1.0 # Distance of 2.0 means full flip. Anything > 1.0 is suspicious.
    
    jumps_l = np.where(diffs_l > threshold)[0]
    jumps_r = np.where(diffs_r > threshold)[0]
    
    print("\n--- detailed Jump Analysis ---")
    
    def analyze_jumps(jumps, name):
        if len(jumps) == 0:
            print(f"[OK] {name} quaternions look continuous.")
            return

        print(f"[WARNING] {name} has {len(jumps)} potential quaternion flips/jumps.")
        real_issues = 0
        
        for idx in jumps:
            # idx is the index in diffs array.
            # diffs[i] = norm(state[i+1] - state[i])
            # So the jump happens between frame i and i+1.
            
            curr_ep = episode_indices[idx]
            next_ep = episode_indices[idx+1]
            
            if curr_ep != next_ep:
                print(f"  - Index {idx}: Jump occurs at Episode Boundary (Ep {curr_ep} -> {next_ep}). [IGNORED]")
            else:
                print(f"  - Index {idx}: Jump occurs WITHIN Episode {curr_ep}. [CRITICAL ERROR]")
                # Calculate dot product to confirm it's a flip
                q1 = states[idx, 3:7] if 'Left' in name else states[idx, 11:15]
                q2 = states[idx+1, 3:7] if 'Left' in name else states[idx+1, 11:15]
                dot = np.dot(q1, q2)
                print(f"    Dot Product: {dot:.4f} (Should be close to 1.0, flipped if close to -1.0)")
                real_issues += 1
                
        if real_issues == 0:
            print(f"-> All jumps in {name} are at episode boundaries. DATA IS SAFE.")
        else:
            print(f"-> Found {real_issues} REAL discontinuities in {name}. DATA NEEDS FIXING.")

    analyze_jumps(jumps_l, "Left Arm")
    analyze_jumps(jumps_r, "Right Arm")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo_id", type=str, required=True)
    parser.add_argument("--root", type=str, default=None)
    args = parser.parse_args()
    
    check_dataset_consistency(args.repo_id, args.root)
