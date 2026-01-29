# Robopoint System Launch
# Integrates:
# 1. Environment TFs (World, Camera, Robot Base)
# 2. Robot Drivers (MoveIt, Controller, Follower)
# 3. Application Logic (Grasp Node)
# 4. Visualization (RViz)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml

def generate_launch_description():
    
    # --- 1. Load Configuration ---
    moveit_config = MoveItConfigsBuilder("koch_v1.1", package_name="koch_moveit_config").to_moveit_configs()
    koch_control_pkg = get_package_share_directory('koch_control')
    kinematics_path = os.path.join(get_package_share_directory('koch_moveit_config'), 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r', encoding='utf-8') as handle:
        kinematics_yaml = yaml.safe_load(handle)

    # --- 2. TF Tree (The Environment) ---
    # We consolidate all TFs here to ensure a single "world" tree.
    
    # A. Robot Base Location (Right Arm position from dual_setup)
    # We map 'world' -> 'base_link' with the offset x=0.2
    # This matches the physical setup but keeps the frame name 'base_link' for MoveIt Compatibility.
    tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0', '0', '0', '0', 'world', 'base_link'],
        name='tf_world_base'
    )

    # B. Camera Locations (Copied from dual_robot_camera to ensure consistency)
    # Camera Top
    tf_world_cam_top = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.16', '0.5', '-1.57', '1.57', '0', 'world', 'camera_top_link'],
        name='tf_world_cam_top'
    )
    # Camera Top Bridge
    tf_cam_top_opt = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_top_link', 'camera_top'],
        name='tf_cam_top_opt'
    )

    # C. Camera Far (From dual_robot_camera)
    tf_world_cam_far = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.5', '0.1', '3.14', '0', '0', 'world', 'camera_far_link'],
        name='tf_world_camera_far'
    )
    tf_cam_far_opt = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_far_link', 'camera_far'],
        name='tf_cam_far_realsense'
    )

    # D. Camera First (From dual_robot_camera)
    tf_world_cam_first = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.04', '-0.18', '0.22', '1.57', '0.52', '0', 'world', 'camera_first_link'],
        name='tf_world_camera_first'
    )
    tf_cam_first_opt = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_first_link', 'camera_first'],
        name='tf_cam_first_realsense'
    )

    # --- 3. Robot Drivers & Control (The Body) ---
    
    # A. Real Robot Follower (Reads Hardware)
    follower_node = Node(
        package='koch_control',
        executable='koch_follower_control',
        name='koch_follower',
        output='screen',
        parameters=[
            {'config_file': os.path.join(koch_control_pkg, 'config', 'single_follower.yaml')},
            {'calibration_dir': '/home/hrc/Lerobot_system/calibration'},
            {'publish_rate': 50.0}
        ]
    )

    # B. Bridge (Renames topics for MoveIt)
    bridge_node = Node(
        package='koch_control',
        executable='moveit_bridge',
        name='moveit_bridge',
        output='screen'
    )

    # C. ROS2 Control (Manages Controllers)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("koch_moveit_config"), "config", "ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("/joint_states", "/fake_joint_states"), # Avoid conflict with real joint states
        ],
        output="screen",
    )

    # D. Spawners
    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["koch_arm_controller", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
    ]

    # --- 4. MoveIt (The Brain) ---
    
    # A. Robot State Publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # B. Move Group (explicit node to ensure robot_description_kinematics is set)
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'robot_description_kinematics': kinematics_yaml},
        ],
    )

    # C. RViz (Single tailored view)
    rviz_config = os.path.join(get_package_share_directory('koch_moveit_config'), 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # --- 5. Robopoint Application (The Logic) ---
    # We delay this slightly to ensure Robot Description is loaded
    grasp_node = Node(
        package='robopoint',
        executable='grasp_node',
        name='grasp_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        # TFs
        tf_world_base,
        tf_world_cam_top,
        tf_cam_top_opt,
        tf_world_cam_far,
        tf_cam_far_opt,
        tf_world_cam_first,
        tf_cam_first_opt,
        # Drivers
        ros2_control_node,
        follower_node,
        bridge_node,
        *spawners,
        # MoveIt
        rsp_launch,
        move_group_node,
        rviz_node,
        # App
        TimerAction(period=5.0, actions=[grasp_node])
    ])
