# Multi-Camera Extrinsic Calibration

This directory contains tools to calibrate the relative positions (extrinsics) of multiple cameras using AprilTag or ArUco markers.

This is useful when you have multiple RealSense cameras in your ROS 2 environment and need a precise static transform between them (e.g., to merge point clouds or have a unified coordinate system).

## Prerequisites

Ensure you have the following installed in your environment:
- `ros-humble-cv-bridge` (or applicable distro)
- `opencv-contrib-python` (usually includes ArUco modules)
- `scipy` (for rotation averaging)

```bash
pip install opencv-contrib-python scipy transforms3d
pip install "numpy<2.0.0"
```

## Setup

1.  **Print a Marker**: 
    - Recommended: AprilTag 36h11, ID 0.
    - You can generate one here: [https://chev.me/arucogen/](https://chev.me/arucogen/) (Select Dictionary: `APRILTAG_36h11`).
    - Print usage: Measure the side length of the black square printed on the paper *precisely* in meters (e.g., 0.166 m).

2.  **Placement**:
    - Place the marker on a flat surface or a wall where **both cameras can see it clearly** at the same time.
    - Ensure the marker is flat and not warped.
    - **Important**: The cameras and the marker must remain **completely static** during the calibration process.

3.  **Launch Cameras**:
    - Start your RealSense cameras as you normally do.
    - Verify they are publishing to topics like `/camera_1/color/image_raw` and `/camera_2/color/image_raw`.

## Robot Hand-Eye Calibration (Camera vs Robot Base)

If you have a robot arm (e.g., Koch Robot) and want to align your entire calibrated camera system to the robot's coordinate system (World Frame), you should use the **Hand-Eye Calibration** tool.

This uses the "Eye-to-Hand" method (Camera is static, Tag is on the gripper).

### 1. Prerequisite Check & Preparation
1.  **Launch Robot TF**: You must have your robot's description running.
    ```bash
    # Example: Launch your dual arm setup
    ros2 launch koch_simulation dual_real_robot_display.launch.py
    ```
2.  **Attach an AprilTag** (e.g., ID 0) rigidly to the robot's end-effector.
    *   **Crucial**: The Tag must NOT move relative to the Gripper during the process.
3.  **Verify Frame Names**:
    *   Find the Base: e.g., `left_base_link`
    *   Find the End-Effector: e.g., `left_gripper_moving_1` or `left_end_effector`
    *   Check with: `ros2 run tf2_ros tf2_echo left_base_link left_gripper_moving_1`

### 2. Run the Calibration Script
Run the script, specifying your camera namespace and robot frames.

```bash
# Example calibrating camera_top relative to the LEFT arm
./scripts/multi-cam/hand_eye_calibrate.py \
  --cam /camera/camera_top \
  --base left_base_link \
  --ee left_gripper_moving_1 \
  --size 0.10 \
  --id 0
```

**Arguments:**
- `--cam`: The namespace of the camera watching the robot (usually your "Base" camera, e.g., `/camera/camera_top`).
- `--base`: The robot's fixed base frame.
- `--ee`: The robot's moving end-effector frame.

### 3. Sampling Process (Interactive)
1.  The script will start and wait for input.
2.  **Move the robot** to a new pose where `camera_top` can clearly see the tag.
3.  **Press ENTER** in the terminal.
    *   The script records: (1) Robot TCP Pose, (2) Camera Detects Tag Pose.
4.  **Repeat 10-15 times**.
    *   Try to vary the position and rotation (pitch/yaw/roll) of the gripper if possible.
    *   Cover different depths and areas of the camera view.

### 4. Calculation & Result
1.  After collecting enough samples, the script solves the $AX=XB$ equation using OpenCV.
2.  It calculates the transform from **Robot Base** -> **Camera Top**.
3.  The result is automatically appended to `config/calibration_params.yaml`.

### 5. Update System
1.  Restart your visualization or publisher:
    ```bash
    ros2 launch scripts/multi-cam/visualize_cameras.launch.py
    ```
2.  **Result**: In RViz (or Unity via TF), the entire camera group (`camera_top` and its children) will now shift to align perfectly with the robot base.

---

## Automated Calibration (Recommended)

We provide a **Wizard Script** (`auto_calibrate.py`) that guides you through calibrating all your cameras relative to a base camera. This is the easiest way to set up multiple cameras (e.g., 3 or 4 camera setup).

### 1. Run the Auto-Calibration Wizard

This script will sequentially ask you to place the marker for each camera pair, perform the calibration, and automatically save the results to `config/calibration_params.yaml`.

```bash
# Run from workspace root
./scripts/multi-cam/auto_calibrate.py
```

**Options:**
- `--base`: The name of the central camera (parent frame). Default: `camera_top`.
- `--others`: List of cameras to calibrate relative to the base. Default: `['camera_first2', 'camera_left', 'camera_right', 'camera_top']`.
- `--size`: Marker size in meters. Default: `0.10`.
- `--id`: ArUco ID. Default: `0`.

Example for custom setup:
```bash
./scripts/multi-cam/auto_calibrate.py --base camera_top --others camera_left camera_right --size 0.05
```

### 2. Visualize Results

Once calibration is complete, launch the visualizer. It uses `multi_cam_publisher.py` to broadcast the transforms defined in `config/calibration_params.yaml`.

```bash
ros2 launch scripts/multi-cam/visualize_cameras.launch.py
```

This launch file starts:
- `multi_cam_publisher.py`: Reads the config and processes the dynamic TF tree.
- `rviz2`: Pre-configured view to see aligned point clouds.

---

## Manual Calibration Usage (Single Pair)

If you need to debug a single pair or run manually without the wizard:

Run the calibration script from the terminal. You need to verify the namespace of your cameras.

```bash
# Example: calibrating camera_2 relative to camera_1
# Assuming topic names are /camera_1/color/image_raw and /camera_2/color/image_raw
python3 scripts/multi-cam/multi_cam_calibration.py \
  --cam1 /camera/camera_first \
  --cam2 /camera/camera_far \
  --size 0.10 --id 0
  --dict APRILTAG_36h11
```

**Arguments:**
- `--cam1`: Namespace of the first camera (Parent frame).
- `--cam2`: Namespace of the second camera (Child frame).
- `--size`: The size of the black square of the marker in meters. (e.g., 5cm = 0.05).
- `--id`: The ID of the marker (default 0).
- `--dict`: The dictionary name (default `APRILTAG_36h11`).

## Output

The script will collect roughly 30 samples and then output the average translation and rotation.

Example output:
```text
Translation (x, y, z): [0.052, -0.420, 0.012]
Rotation description (x, y, z, w): [0.0, 0.0, 0.382, 0.924]
Command for static_transform_publisher:
ros2 run tf2_ros static_transform_publisher --x 0.052 --y -0.420 --z 0.012 --qx 0.0 --qy 0.0 --qz 0.382 --qw 0.924 --frame-id camera_1_link --child-frame-id camera_2_link
```

## Applying the Transform

To use this calibration in your system, you can:

1.  **Run manually**: Copy the printed `ros2 run tf2_ros ...` command and run it in a terminal.
2.  **Add to Launch File**: Add a `static_transform_publisher` node to your system's launch file.

```python
from launch_ros.actions import Node

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments = ['--x', '0.052', '--y', '-0.42', '--z', '0.012', 
                 '--qx', '0.0', '--qy', '0.0', '--qz', '0.382', '--qw', '0.924',
                 '--frame-id', 'camera_1_link', 
                 '--child-frame-id', 'camera_2_link']
)
```

# Three Camera Calibration Workflow (DEPRECATED - Use Auto-Calibration instead)

The old workflow for manual calibration is kept here for reference or debugging.

If you have 3 cameras (e.g., `camera_first`, `camera_far`, `camera_top`), you need to establish a transform tree. A common approach is to pick one as the "Base" (e.g. `camera_first`) and calibrate the others relative to it.

1.  **Launch all 3 cameras**. ensure they have unique namespaces.
2.  **Calibrate First -> Far**:
    ...

## Visualization verification

To verify your calibration result, run the visualizer mentioned in the "Automated Calibration" section.

```bash
ros2 launch scripts/multi-cam/visualize_cameras.launch.py
```

It will now automatically load from `scripts/multi-cam/config/calibration_params.yaml`.

## Unity Integration (Rosbridge)

If you are using `rosbridge_server` to connect with Unity (e.g., via [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) or [RosbridgeClient](https://github.com/siemens/ros-sharp)), you have two main options to apply the calibration:

### 1. Dynamic TF Subscription (Recommended)
Since `multi_cam_publisher.py` publishes static transforms to `/tf_static` (and `/tf`), you can use a TF Listener component in Unity to automatically update the camera positions.

- **Unity Package needed**: [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) or similar.
- **Topic**: Subscribe to `/tf` and `/tf_static`.
- **Target Frame**: E.g., `camera_left_link` relative to `camera_top_link`.

### 2. Hardcoded Values (Simple / Static)
If your cameras are fixed, you can simply copy the calibrated values into the Unity Inspector. However, **Coordinate Systems are different**:
- **ROS (Right-Handed)**: X=Forward, Y=Left, Z=Up
- **Unity (Left-Handed)**: Z=Forward, X=Right, Y=Up

**We provide a helper script to convert the values for you:**

```bash
./scripts/multi-cam/show_unity_coords.py
```

**Output Example:**
```text
CAMERA: camera_right
  Parent in ROS: camera_top_color_optical_frame
  [UNITY TRANSFORM]
    Position:  X: 0.045   Y: 0.012   Z: -0.420
    Rotation:  X: -0.05   Y: -0.38   Z: 0.00    W: 0.92
```

**How to use in Unity:**
1.  Create a Parent GameObject representing the Base Camera (e.g., `Camera_Top`).
2.  Create Child GameObjects for other cameras (e.g., `Camera_Right`) under the Parent.
3.  Copy the **Position (X, Y, Z)** and **Rotation (X, Y, Z, W)** from the script output into the Child's **Transform** component.

*Note: If your Unity scene uses a different coordinate convention (e.g., ROS-Coordinate-System wrapper), you might need to use the original ROS values. The script assumes standard Unity (Ruf) convention.*

## Troubleshooting

- **"Flight" cameras**: If your cameras appear to fly away or the TF tree is broken, ensure `multi_cam_publisher.py` is running and can find the transform between `_link` and `_optical_frame`.
- **No detection**: Check lighting logic. Ensure the white border of the AprilTag is visible.
- **Inaccurate**: Warped paper or bad intrinsic calibration of the cameras themselves. RealSense cameras usually come with factory calibration (visible in `/camera_info`), but custom intrinsic calibration might be needed for high precision.
- **Jitter**: Increase the number of samples in the script code if needed, or ensure the setup is rigid.
