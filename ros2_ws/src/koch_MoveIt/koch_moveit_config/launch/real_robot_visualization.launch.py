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

    # 1. Launch Real Robot Driver (Follower)
    # This publishes /right_follower/joint_states
    koch_wrapper_pkg = get_package_share_directory('koch_control')
    follower_node = Node(
        package='koch_control',
        executable='koch_follower_control',
        name='koch_follower',
        output='screen',
        parameters=[
            {'config_file': os.path.join(koch_wrapper_pkg, 'config', 'single_follower.yaml')},
            {'calibration_dir': '/home/hrc/Lerobot_system/calibration'},  # ✅ Fixed path
            {'publish_rate': 50.0}
        ]
    )

    # 2. Launch Bridge Node
    # Converts /right_follower/joint_states -> /joint_states (renaming joints)
    # And listens to /fake_joint_states -> /right_follower/joint_states_control
    bridge_node = Node(
        package='koch_control',
        executable='moveit_bridge',
        name='moveit_bridge',
        output='screen'
    )

    # 3. Launch ROS2 Control Node (Mock Hardware)
    # This runs the controllers on a fake robot.
    # We remap /joint_states to /fake_joint_states so it doesn't conflict with real robot.
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

    # 4. Launch MoveIt Components
    
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
