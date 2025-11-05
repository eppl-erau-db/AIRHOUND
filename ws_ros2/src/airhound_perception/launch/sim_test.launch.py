#!/usr/bin/env python3
"""
Launch file for SITL testing with mock detector
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='airhound_perception',
            executable='mock_detector',
            name='mock_detector',
            output='screen',
            parameters=[{
                'detection_rate': 10.0,
                'mock_position_x': 1.0,
                'mock_position_y': 0.5,
            }]
        ),
    ])
