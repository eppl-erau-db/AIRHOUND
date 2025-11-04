from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for simulation testing WITHOUT real YOLO perception.

    This is for testing control algorithms in PX4 SITL where you don't need
    actual object detection - just mock detection data to test yaw control.

    No camera, no YOLO, no model weights needed!

    Usage:
        ros2 launch airhound_perception sim_test.launch.py

        # With moving target (circular motion)
        ros2 launch airhound_perception sim_test.launch.py moving_target:=true

        # Static target at image center
        ros2 launch airhound_perception sim_test.launch.py moving_target:=false

        # Custom motion pattern
        ros2 launch airhound_perception sim_test.launch.py motion_type:=sinusoidal
    """

    # Launch arguments
    moving_target_arg = DeclareLaunchArgument(
        "moving_target",
        default_value="true",
        description="Whether to make the target move (true/false)",
    )

    motion_type_arg = DeclareLaunchArgument(
        "motion_type",
        default_value="circular",
        description="Motion pattern: circular, sinusoidal, figure8, or static",
    )

    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="30.0",
        description="Detection publish rate in Hz",
    )

    num_targets_arg = DeclareLaunchArgument(
        "num_targets",
        default_value="1",
        description="Number of targets to simulate",
    )

    add_noise_arg = DeclareLaunchArgument(
        "add_noise",
        default_value="false",
        description="Add random noise to target positions",
    )

    image_width_arg = DeclareLaunchArgument(
        "image_width",
        default_value="1280",
        description="Simulated image width in pixels",
    )

    image_height_arg = DeclareLaunchArgument(
        "image_height",
        default_value="720",
        description="Simulated image height in pixels",
    )

    # Launch configurations
    moving_target = LaunchConfiguration("moving_target")
    motion_type = LaunchConfiguration("motion_type")
    publish_rate = LaunchConfiguration("publish_rate_hz")
    num_targets = LaunchConfiguration("num_targets")
    add_noise = LaunchConfiguration("add_noise")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")

    return LaunchDescription(
        [
            # Declare all arguments
            moving_target_arg,
            motion_type_arg,
            publish_rate_arg,
            num_targets_arg,
            add_noise_arg,
            image_width_arg,
            image_height_arg,
            # Mock detector node (no YOLO, no camera, no model weights)
            Node(
                package="airhound_perception",
                executable="mock_detector",
                name="mock_detector_node",
                output="screen",
                parameters=[
                    {
                        "output_detections_topic": "/detections",
                        "frame_id": "camera_color_optical_frame",
                        "publish_rate_hz": publish_rate,
                        "image_width": image_width,
                        "image_height": image_height,
                        # Motion parameters
                        "moving_target": moving_target,
                        "motion_type": motion_type,
                        "motion_radius": 200.0,  # pixels from center
                        "motion_speed": 0.5,  # rad/s or Hz
                        # Target appearance
                        "target_width": 120,  # bbox width in pixels
                        "target_height": 80,  # bbox height in pixels
                        "confidence": 0.85,  # detection confidence
                        "class_id": "drone",  # class label
                        # Multi-target
                        "num_targets": num_targets,
                        "add_noise": add_noise,
                        "noise_std": 5.0,  # pixel noise std dev
                    }
                ],
            ),
        ]
    )
