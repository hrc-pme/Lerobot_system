#!/usr/bin/env python3
"""
只發布相機相關的 TF，不啟動相機驅動
用於已經手動啟動相機的情況
"""

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # --- Static Transforms (World -> Camera Links) ---
    # 這些數值要和 dual_robot_camera.launch.py 保持一致
    
    # World frame (root)
    # 這個很重要！是整個 TF tree 的根
    
    # Camera Top (Looking down from above)
    tf_world_camera_top = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0.16', '0.5', '-1.57', '1.57', '0', 'world', 'camera_top_link'],
        name='tf_world_camera_top'
    )
    
    # Bridge: camera_top_link -> camera_top (RealSense base_link)
    # RealSense 驅動會從 camera_top 發出座標鏈
    tf_camera_top_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_top_link', 'camera_top'],
        name='tf_camera_top_realsense'
    )

    # Camera First (Side view)
    tf_world_camera_first = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.04', '-0.18', '0.22', '1.57', '0.52', '0', 'world', 'camera_first_link'],
        name='tf_world_camera_first'
    )
    
    # Bridge: camera_first_link -> camera_first (RealSense base_link)
    tf_camera_first_realsense = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_first_link', 'camera_first'],
        name='tf_camera_first_realsense'
    )

    return LaunchDescription([
        tf_world_camera_top,
        tf_camera_top_realsense,
        tf_world_camera_first,
        tf_camera_first_realsense,
    ])
