from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Record camera and detections topics
    topics = [
        '/camera/color/image_raw',
        '/camera/camera_info',
        '/detections',
    ]
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', *topics],
            output='screen'
        )
    ])
