from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import math

# This launch file combines:
# 1. Dual Robot Display (Koch Simulation)
# 2. Multi-Camera Publisher (Your Calibration)
# 3. Custom RViz (Shows PointClouds + RobotModels)

def generate_launch_description():
    
    # --- 1. Load Robot Description logic (Copied partially from dual_real_robot_display) ---
    share_dir = get_package_share_directory('koch_simulation')
    xacro_file = os.path.join(share_dir, 'urdf', 'low_cost_robot.xacro')
    
    # Static Transforms (World -> Left/Right Base)
    # These match dual_real_robot_display.launch.py default:
    # Left: -0.2, Right: 0.2 (Total ~40cm apart?)
    tf_world_left = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.2', '0', '0', '0', '0', '0', 'world', 'left_base_link'],
        name='tf_world_left'
    )
    
    tf_world_right = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2', '0', '0', '0', '0', '0', 'world', 'right_base_link'],
        name='tf_world_right'
    )

    # Robot Groups
    left_group = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='left',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': xacro.process_file(xacro_file, mappings={'prefix': 'left_'}).toxml()
            }],
            output='screen'
        ),
        Node(
            package='koch_simulation',
            executable='real_robot_remapper',
            namespace='left',
            name='real_robot_remapper',
            parameters=[{
                'source_topic': '/left_follower/joint_states',
                'target_topic': 'joint_states',
                'target_prefix': 'left_'
            }]
        )
    ])

    right_group = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='right',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': xacro.process_file(xacro_file, mappings={'prefix': 'right_'}).toxml()
            }],
            output='screen'
        ),
        Node(
            package='koch_simulation',
            executable='real_robot_remapper',
            namespace='right',
            name='real_robot_remapper',
            parameters=[{
                'source_topic': '/right_follower/joint_states',
                'target_topic': 'joint_states',
                'target_prefix': 'right_'
            }]
        )
    ])

    # --- 2. Multi-Camera Publisher ---
    script_dir = os.path.dirname(os.path.realpath(__file__))
    multi_cam_publisher_path = os.path.join(script_dir, 'multi_cam_publisher.py')
    
    multi_cam_node = ExecuteProcess(
        cmd=['python3', multi_cam_publisher_path],
        output='screen'
    )

    # --- 3. RViz ---
    # We use our own RViz config to show pointclouds + robot models
    rviz_config_path = os.path.join(script_dir, 'config', 'integrated_system.rviz')
    
    # If the config doesn't exist yet, we can use the default multi_cam one or make a new one.
    # For now, let's point to multi_cam.rviz but user needs to add RobotModel manually once and save.
    if not os.path.exists(rviz_config_path):
        # Fallback to multi_cam.rviz if exists
        fallback = os.path.join(script_dir, 'config', 'multi_cam.rviz')
        if os.path.exists(fallback):
            rviz_config_path = fallback
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        tf_world_left,
        tf_world_right,
        left_group,
        right_group,
        multi_cam_node,
        rviz_node
    ])
