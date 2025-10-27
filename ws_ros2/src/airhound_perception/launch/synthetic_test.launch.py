from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    moving_target_arg = DeclareLaunchArgument(
        "moving_target",
        default_value="true",
        description="Whether to make the synthetic target move in a circle",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz", default_value="30.0", description="Camera publish rate in Hz"
    )

    # Get package paths
    pkg_share = get_package_share_directory("airhound_perception")
    params_file = os.path.join(pkg_share, "config", "perception.yaml")

    # Param file override argument
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Path to a perception parameter YAML file",
    )
    # Launch configurations
    moving_target = LaunchConfiguration("moving_target")
    publish_rate = LaunchConfiguration("publish_rate_hz")
    chosen_params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            moving_target_arg,
            publish_rate_arg,
            params_file_arg,
            # Synthetic camera node
            Node(
                package="airhound_perception",
                executable="synthetic_camera",
                name="synthetic_camera_node",
                output="screen",
                parameters=[
                    {
                        "publish_rate_hz": publish_rate,
                        "image_width": 1280,
                        "image_height": 720,
                        "frame_id": "camera_color_optical_frame",
                        "camera_info_topic": "/camera/camera_info",
                        "image_topic": "/camera/color/image_raw",
                        "compressed_topic": "/camera/color/image_raw/compressed",
                        "publish_compressed": True,
                        "moving_target": moving_target,
                    }
                ],
            ),
            # Perception node
            Node(
                package="airhound_perception",
                executable="detector_node",
                name="perception_node",
                output="screen",
                parameters=[chosen_params_file],
            ),
        ]
    )
