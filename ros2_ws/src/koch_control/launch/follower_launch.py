from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_koch_control = FindPackageShare('koch_control')

    # Declare the launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_koch_control, 'config', 'two_follower.yaml']),
        description='Path to the config file for koch_follower_control'
    )
    
    calibration_dir_arg = DeclareLaunchArgument(
        'calibration_dir',
        default_value='/home/hrc/Lerobot_system/calibration',
        description='Path to the calibration directory'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Publish rate for the follower control'
    )

    # Launch configuration variables
    config_file = LaunchConfiguration('config_file')
    calibration_dir = LaunchConfiguration('calibration_dir')
    publish_rate = LaunchConfiguration('publish_rate')

    # Define the node
    follower_node = Node(
        package='koch_control',
        executable='koch_follower_control',
        name='koch_follower_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_file': config_file,
            'calibration_dir': calibration_dir,
            'publish_rate': publish_rate
        }]
    )

    return LaunchDescription([
        config_file_arg,
        calibration_dir_arg,
        publish_rate_arg,
        follower_node
    ])
