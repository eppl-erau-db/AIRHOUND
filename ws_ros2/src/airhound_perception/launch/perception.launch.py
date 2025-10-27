from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('airhound_perception')
    params_file = os.path.join(pkg_share, 'config', 'perception.yaml')

    return LaunchDescription([
        Node(
            package='airhound_perception',
            executable='detector_node',
            name='perception_node',
            output='screen',
            parameters=[params_file],
        )
    ])
