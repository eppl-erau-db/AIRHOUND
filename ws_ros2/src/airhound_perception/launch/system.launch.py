from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    record_bag_arg = DeclareLaunchArgument("record_bag", default_value="false")
    record_bag = LaunchConfiguration("record_bag")

    pkg_share = get_package_share_directory("airhound_perception")
    # Use provided realsense_profile.yaml from workspace config
    # Navigate to workspace root from package share directory (install -> workspace root)
    workspace_root = os.path.abspath(os.path.join(pkg_share, "..", "..", "..", ".."))
    realsense_cfg = os.path.join(workspace_root, "config", "realsense_profile.yaml")
    perception_params = os.path.join(pkg_share, "config", "perception.yaml")

    actions = [
        record_bag_arg,
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="realsense2_camera",
            output="screen",
            parameters=[realsense_cfg],
            remappings=[
                (
                    "/camera/realsense2_camera/color/image_raw",
                    "/camera/color/image_raw",
                ),
                ("/camera/realsense2_camera/color/camera_info", "/camera/camera_info"),
            ],
        ),
        Node(
            package="airhound_perception",
            executable="detector_node",
            name="perception_node",
            output="screen",
            parameters=[perception_params],
        ),
    ]

    # Optionally add rosbag2 recorder if requested
    # Users can start recording manually too.
    # Note: We avoid adding conditional bag recording here to keep dependencies minimal.
    return LaunchDescription(actions)
