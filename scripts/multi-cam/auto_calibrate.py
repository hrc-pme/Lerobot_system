#!/usr/bin/env python3
import subprocess
import os
import sys
import argparse
import json
import yaml

# List of all available camera names in the system
# These names should match the ROS namespaces (e.g., /camera/camera_top) without the /camera/ prefix
DEFAULT_CAMERAS = ['camera_first2', 'camera_left', 'camera_right', 'camera_top']

def run_calibration_process(cam1_ns, cam2_ns, size, marker_id):
    # cam1 is base, cam2 is target
    # We calibrate cam2 relative to cam1
    
    print(f"\n{'='*60}")
    print(f"CALIBRATING TARGET: {cam2_ns}")
    print(f"RELATIVE TO BASE:   {cam1_ns}")
    print(f"{'='*60}")
    print(f"INSTRUCTIONS:")
    print(f"1. Place the AprilTag (Size: {size}m, ID: {marker_id}) so BOTH cameras can see it clearly.")
    print(f"2. Ensure cameras are static.")
    input("Press ENTER when ready to start sampling...")
    
    # Generate timestamp for unique output file
    import datetime
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    cam1_simple = cam1_ns.replace('/', '_')
    cam2_simple = cam2_ns.replace('/', '_')
    output_file = f"/tmp/calibration_{cam1_simple}_{cam2_simple}_{timestamp}.json"
    
    # Determine path to calibration script
    # Assumes we are running from workspace root
    script_path = "scripts/multi-cam/multi_cam_calibration.py"
    if not os.path.exists(script_path):
        # Fallback: try relative to this script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        script_path = os.path.join(script_dir, "multi_cam_calibration.py")
        
    if not os.path.exists(script_path):
        print(f"ERROR: Could not find multi_cam_calibration.py at {script_path}")
        return None
    
    cmd = [
        "python3", script_path,
        "--cam1", cam1_ns,
        "--cam2", cam2_ns,
        "--size", str(size),
        "--id", str(marker_id),
        "--output", output_file
    ]
    
    print(f"Executing: {' '.join(cmd)}")
    
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Calibration script process failed.")
        return None
    except KeyboardInterrupt:
        print("Process interrupted.")
        return None

    if os.path.exists(output_file):
        try:
            with open(output_file, 'r') as f:
                data = json.load(f)
            return data
        except json.JSONDecodeError:
            print("ERROR: Failed to decode output JSON.")
            return None
    else:
        print(f"ERROR: Calibration script finished but output file {output_file} not found.")
        return None

def main():
    parser = argparse.ArgumentParser(description="Automated Multi-Camera Calibration Wizard")
    parser.add_argument("--base", type=str, default="camera_top", help="The base camera name (Parent of all others). e.g. camera_top")
    parser.add_argument("--others", type=str, nargs="+", default=[], help="List of other cameras to calibrate. e.g. camera_left camera_right")
    parser.add_argument("--size", type=float, default=0.10, help="Marker size in meters")
    parser.add_argument("--id", type=int, default=0, help="Marker ID")
    parser.add_argument("--config", type=str, default="scripts/multi-cam/config/calibration_params.yaml", help="Output YAML config path")
    
    args = parser.parse_args()

    # Determine Base and Targets
    base_cam = args.base
    target_cams = args.others
    
    # If explicit list not provided, compute remaining from DEFAULT
    if not target_cams:
        # If base is in default list, remove it
        available = [c for c in DEFAULT_CAMERAS if c != base_cam]
        target_cams = available
    
    print(f"Configuration:")
    print(f"  Base Camera: {base_cam}")
    print(f"  Targets:     {target_cams}")
    print(f"  Tag Size:    {args.size}m")
    
    final_results = {}
    
    for target in target_cams:
        # Construct full ROS namespace
        # Assuming format /camera/<name>
        # Check if user already provided /camera/... prefix? No assumption: simple names.
        base_ns = f"/camera/{base_cam}"
        target_ns = f"/camera/{target}"
        
        # Run calibration
        calib_data = run_calibration_process(base_ns, target_ns, args.size, args.id)
        
        if calib_data:
            # Structure for multi_cam_publisher.py
            # Key is the camera name (simple name, e.g. 'camera_left')
            final_results[target] = {
                'pos': calib_data['translation'],
                'quat': calib_data['rotation'],
                'parent_frame': calib_data['parent_frame'],
                'child_optical_frame': calib_data['child_frame'],
                # We assume the link frame name follows standard convention: name_link
                'child_link_frame': f"{target}_link"
            }
            print(f"SUCCESS: Calibrated {target}")
            print(f"  Pos: {calib_data['translation']}")
        else:
            print(f"FAILURE: Could not calibrate {target}. Skipping.")

    # Save Results
    if final_results:
        # Determine strict absolute path for config
        if os.path.isabs(args.config):
            config_abs_path = args.config
        else:
            # relative to CWD
            config_abs_path = os.path.abspath(args.config)
            
        # Create directory if needed
        os.makedirs(os.path.dirname(config_abs_path), exist_ok=True)
        
        print(f"\nWriting results to {config_abs_path}...")
        with open(config_abs_path, 'w') as f:
            yaml.dump(final_results, f, default_flow_style=None)
            
        print("Done. You can now verify with: ros2 launch scripts/multi-cam/visualize_cameras.launch.py")
    else:
        print("\nNo valid calibration data acquired.")

if __name__ == "__main__":
    main()
