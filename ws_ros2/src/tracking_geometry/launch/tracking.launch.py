#!/usr/bin/env python3
"""
Launch file for AIRHOUND tracking node.

Subscribes to /detections and /camera/camera_info
Publishes /yaw_command for offboard control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    max_rate_arg = DeclareLaunchArgument(
        'max_rate',
        default_value='1.0',
        description='Maximum yaw rate command (rad/s)'
    )
    
    deadband_arg = DeclareLaunchArgument(
        'deadband',
        default_value='0.01',
        description='Deadband threshold to avoid jitter (rad)'
    )

    # Tracking node
    tracking_node = Node(
        package='tracking_geometry',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{
            'max_rate': LaunchConfiguration('max_rate'),
            'deadband': LaunchConfiguration('deadband'),
        }],
        remappings=[
            # Explicit topic remappings for clarity
            ('/detections', '/detections'),
            ('/camera/camera_info', '/camera/camera_info'),
            ('/yaw_command', '/yaw_command'),
        ]
    )

    return LaunchDescription([
        max_rate_arg,
        deadband_arg,
        tracking_node,
    ])
