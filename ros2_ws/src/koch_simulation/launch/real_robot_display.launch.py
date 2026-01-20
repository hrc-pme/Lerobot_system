from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('koch_simulation')

    xacro_file = os.path.join(share_dir, 'urdf', 'low_cost_robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')
    
    # Argument to select which topic to listen to (e.g., /right_follower/joint_states)
    source_topic_arg = DeclareLaunchArgument(
        name='source_topic',
        default_value='/right_follower/joint_states',
        description='The real robot topic to map to simulation'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    # OUR CUSTOM REMAPPER NODE
    remapper_node = Node(
        package='koch_simulation',
        executable='real_robot_remapper',
        name='real_robot_remapper',
        parameters=[
            {'source_topic': LaunchConfiguration('source_topic')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        source_topic_arg,
        robot_state_publisher_node,
        remapper_node,
        rviz_node
    ])
