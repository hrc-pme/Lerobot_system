#!/usr/bin/env python3
import yaml
import os
import sys

def main():
    # Attempt to locate config relative to script or cwd
    possible_paths = [
        'scripts/multi-cam/config/calibration_params.yaml',
        'config/calibration_params.yaml',
        os.path.join(os.path.dirname(__file__), 'config/calibration_params.yaml')
    ]
    
    config_path = None
    for p in possible_paths:
        if os.path.exists(p):
            config_path = p
            break
            
    if not config_path:
        print("ERROR: Could not find config/calibration_params.yaml")
        sys.exit(1)

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    print("="*60)
    print("UNITY COORDINATE CONVERSION HELPER")
    print("="*60)
    print("Use these values if you want to hardcode the transforms in Unity.")
    print("Rule: ROS(Flu) -> Unity(Ruf) [Right-Up-Forward]")
    print("  Position: Unity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x")
    print("  Rotation: Unity.x =  ROS.y, Unity.y = -ROS.z, Unity.z = -ROS.x, Unity.w = ROS.w")
    print("-" * 60)

    for cam, params in data.items():
        p = params['pos']
        q = params['quat'] # x,y,z,w
        
        # Unity Values
        # Pos: -y, z, x
        u_p = [-p[1], p[2], p[0]]
        # Rot: y, -z, -x, w
        u_q = [q[1], -q[2], -q[0], q[3]]
        
        print(f"CAMERA: {cam}")
        print(f"  Parent in ROS: {params['parent_frame']}")
        print(f"  [UNITY TRANSFORM]")
        print(f"    Position:  X: {u_p[0]:.6f}   Y: {u_p[1]:.6f}   Z: {u_p[2]:.6f}")
        print(f"    Rotation:  X: {u_q[0]:.6f}   Y: {u_q[1]:.6f}   Z: {u_q[2]:.6f}   W: {u_q[3]:.6f}")
        print("-" * 60)

if __name__ == "__main__":
    main()
