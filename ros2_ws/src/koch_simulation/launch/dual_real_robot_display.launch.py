from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import math

def generate_launch_description():
    share_dir = get_package_share_directory('koch_simulation')
    
    # Load URDF
    xacro_file = os.path.join(share_dir, 'urdf', 'low_cost_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz') 

    # --- Static Transforms (World -> Left/Right) ---
    # Separate them by 30cm (y-axis)
    # 格式: x y z yaw pitch roll parent child
    # 調這裡來改變位置
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

    # --- Left Arm Group ---
    left_group = GroupAction([
        # Robot State Publisher
        # Pass prefix='left_' to xacro
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
        # Remapper
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

    # --- Right Arm Group ---
    right_group = GroupAction([
        # Robot State Publisher
        # Pass prefix='right_' to xacro
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
        # Remapper
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

    # RViz
    # We might need a modified rviz config to show both, or user sets it up.
    # For now, launch rviz and user will set "Fixed Frame" to "world" and add RobotModels.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        tf_world_left,
        tf_world_right,
        left_group,
        right_group,
        rviz_node
    ])
