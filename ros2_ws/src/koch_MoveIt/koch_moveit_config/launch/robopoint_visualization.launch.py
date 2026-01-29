# Robot Point: Single Arm MoveIt Visualization
# Note: Since the existing real_robot_visualization uses base_link as root,
# we need to bridge world -> base_link to align with the camera system.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    
    moveit_config = MoveItConfigsBuilder("koch_v1.1", package_name="koch_moveit_config").to_moveit_configs()

    # 1. Static Transform: world -> base_link
    # We set base_link to origin of world (0,0,0) for this single-robot setup.
    # This ensures "world" maps to the robot base, just like dual_robot_camera mapped TFs.
    tf_world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        name='tf_world_base_link'
    )

    # 2. Launch Real Robot Driver (Follower)
    koch_wrapper_pkg = get_package_share_directory('koch_control')
    follower_node = Node(
        package='koch_control',
        executable='koch_follower_control',
        name='koch_follower',
        output='screen',
        parameters=[
            {'config_file': os.path.join(koch_wrapper_pkg, 'config', 'single_follower.yaml')},
            {'calibration_dir': '/home/hrc/Lerobot_system/calibration'},
            {'publish_rate': 50.0}
        ]
    )

    # 3. Launch Bridge Node
    bridge_node = Node(
        package='koch_control',
        executable='moveit_bridge',
        name='moveit_bridge',
        output='screen'
    )

    # 4. Launch ROS2 Control Node (Mock Hardware)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(get_package_share_directory("koch_moveit_config"), "config", "ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
            ("/joint_states", "/fake_joint_states"),
        ],
        output="screen",
    )

    koch_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["koch_arm_controller", "-c", "/controller_manager"],
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # 5. Launch MoveIt Components
    
    # RSP (Robot State Publisher) - Uses Real Robot States (/joint_states)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'), 'launch', 'move_group.launch.py')
        )
    )

    # RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('koch_moveit_config'), 'launch', 'moveit_rviz.launch.py')
        )
    )

    return LaunchDescription([
        tf_world_base, # CRITICAL: Connects robot to world
        ros2_control_node,
        koch_arm_spawner,
        gripper_spawner,
        jsb_spawner,
        follower_node,
        bridge_node,
        rsp_launch,
        move_group_launch,
        moveit_rviz_launch
    ])
