from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. Load MoveIt Configuration
    # We use the same config as real_robot_visualization to ensure consistency
    moveit_config = MoveItConfigsBuilder("koch_v1.1", package_name="koch_moveit_config").to_moveit_configs()

    # 2. Define the Grasp Node
    # output='screen' allows us to see the logs
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
        grasp_node
    ])
