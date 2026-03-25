from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # You can put your calibrated transforms here to test them immediately.
    # Unomment the lines below and replace the values with your calibration results.
    
    # replaced by a smarter node that handles the link transformations automatically.
    
    # Static publisher node
    multi_cam_tf_publisher = Node(
        package='scripts',  # This assumes scripts is not a package, so we run the python file directly
        executable='multi_cam_publisher.py', # This won't work directly if not installed.
        # So we use executable with full path or a python execution.
        parameters=[],
        output='screen'
    )
    
    # Since multi_cam_publisher.py is just a script, we use execute_process or define it properly
    # A better way for loose scripts:
    
    multi_cam_publisher_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 
        'multi_cam_publisher.py'
    )
    
    from launch.actions import ExecuteProcess
    
    multi_cam_node = ExecuteProcess(
        cmd=['python3', multi_cam_publisher_path],
        output='screen'
    )

    rviz_config_path = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), 
        'config', 
        'multi_cam.rviz'
    )

    return LaunchDescription([
        multi_cam_node,
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
