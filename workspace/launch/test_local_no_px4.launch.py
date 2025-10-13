
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='offboard_control',
            executable='yaw_controller_node',
            name='yaw_controller_node',
            output='screen',
        ),
        Node(
            package='offboard_control',
            executable='dummy_yaw_publisher',
            name='dummy_yaw_publisher',
            output='screen',
        ),
    ])
