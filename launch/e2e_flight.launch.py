#!/usr/bin/env python3
"""
AIRHOUND E2E Flight Launch File

Launches the complete end-to-end pipeline for real flight:
  1. RealSense Camera - Intel RealSense D455 RGB + Depth (or synthetic)
  2. Detector Node - RF-DETR/YOLOv8 with TensorRT (or mock detector)
  3. Tracking Node - Converts detections to yaw rate commands
  4. PX4 Converter - Sends yaw commands to PX4 via DDS

Prerequisites:
  - For camera_source="realsense": Intel RealSense D455 camera connected
  - For camera_source="synthetic": No camera needed (uses mock detector)
  - Model weights available (TensorRT engine or PyTorch weights) - only for realsense
  - MicroXRCE-DDS Agent running
  - PX4 flight controller connected
  - Workspace built and sourced

Usage:
  ros2 launch airhound e2e_flight.launch.py
  ros2 launch airhound e2e_flight.launch.py camera_source:=synthetic
  ros2 launch airhound e2e_flight.launch.py model_path:=/path/to/model.engine
  ros2 launch airhound e2e_flight.launch.py max_rate:=0.5 auto_arm:=false

Data Flow (camera_source="realsense"):
  RealSense D455
    ├── RGB ──────────────► detector_node ──► /detections ─────────────────────┐
    ├── Depth ────────────► detector_node ──► /perception/target_3d (3D pos)  │
    └── CameraInfo ───────► detector_node                                      │
                                                                                ▼
                                              tracking_node ──► /target_yaw ──► px4_converter_node ──► PX4

Data Flow (camera_source="synthetic"):
  mock_detector ──► /detections ──► tracking_node ──► /target_yaw ──► px4_converter_node ──► PX4

Topic Reference:
  Published by detector_node (or mock_detector):
    /detections             (vision_msgs/Detection2DArray) - 2D bbox + depth in pose.z
    /perception/target_3d   (geometry_msgs/PointStamped)   - 3D position in camera frame
    /perception/latency_ms  (std_msgs/Float32)             - Inference latency
    /perception/fps         (std_msgs/Float32)             - Current FPS

  Published by tracking_node:
    /target_yaw             (std_msgs/Float32)             - Yaw rate command (rad/s)

  Published by px4_converter_node:
    /fmu/in/*               (px4_msgs/*)                   - PX4 offboard commands

  See docs/PERCEPTION_INTERFACE.md for detailed message formats.

SAFETY WARNING:
  - Always test with auto_arm:=false first!
  - Ensure RC transmitter is ready for manual override
  - Have a spotter when testing outdoors
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    
    # Camera source selection (NEW)
    camera_source_arg = DeclareLaunchArgument(
        'camera_source',
        default_value='realsense',
        description='Detection source: "realsense" (real camera) or "synthetic" (mock detector)'
    )
    
    # Camera arguments (only used when camera_source=realsense)
    camera_serial_arg = DeclareLaunchArgument(
        'camera_serial',
        default_value='',
        description='RealSense camera serial number (empty = first available)'
    )
    
    rgb_width_arg = DeclareLaunchArgument(
        'rgb_width',
        default_value='1280',
        description='RGB camera width'
    )
    
    rgb_height_arg = DeclareLaunchArgument(
        'rgb_height',
        default_value='720',
        description='RGB camera height'
    )
    
    rgb_fps_arg = DeclareLaunchArgument(
        'rgb_fps',
        default_value='30',
        description='RGB camera FPS'
    )
    
    # Detector arguments (only used when camera_source=realsense)
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='models/yolov8Detector.engine',
        description='Path to YOLO model (TensorRT .engine or PyTorch .pt)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.25',
        description='Detection confidence threshold'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='GPU device ID or "cpu"'
    )
    
    # Mock detector arguments (only used when camera_source=synthetic)
    moving_target_arg = DeclareLaunchArgument(
        'moving_target',
        default_value='true',
        description='Whether to make the simulated target move'
    )
    
    motion_type_arg = DeclareLaunchArgument(
        'motion_type',
        default_value='circular',
        description='Motion pattern: circular, sinusoidal, figure8, static'
    )
    
    motion_speed_arg = DeclareLaunchArgument(
        'motion_speed',
        default_value='0.5',
        description='Motion speed in rad/s'
    )
    
    motion_radius_arg = DeclareLaunchArgument(
        'motion_radius',
        default_value='200.0',
        description='Motion radius in pixels from center'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='30.0',
        description='Detection publish rate in Hz (synthetic mode)'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Simulated image width (synthetic mode)'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Simulated image height (synthetic mode)'
    )
    
    add_noise_arg = DeclareLaunchArgument(
        'add_noise',
        default_value='false',
        description='Add random noise to target positions (synthetic mode)'
    )
    
    # Tracking arguments
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
    
    # PX4 converter arguments
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',  # SAFETY: Default to false for flight mode!
        description='Automatically arm and enter offboard mode (USE WITH CAUTION)'
    )
    
    px4_publish_rate_arg = DeclareLaunchArgument(
        'px4_publish_rate',
        default_value='10.0',
        description='PX4 offboard control message rate (Hz)'
    )
    
    safety_timeout_arg = DeclareLaunchArgument(
        'safety_timeout',
        default_value='5.0',
        description='Seconds without command before failsafe'
    )
    
    # Optional: skip camera (if already running) - only relevant for realsense mode
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch RealSense camera node (only for camera_source=realsense)'
    )

    # ==========================================================================
    # Condition helpers
    # ==========================================================================
    
    # Condition: camera_source == "realsense"
    use_realsense = IfCondition(
        PythonExpression(["'", LaunchConfiguration('camera_source'), "' == 'realsense'"])
    )
    
    # Condition: camera_source == "synthetic"
    use_synthetic = IfCondition(
        PythonExpression(["'", LaunchConfiguration('camera_source'), "' == 'synthetic'"])
    )
    
    # Condition: launch camera AND camera_source == "realsense"
    launch_realsense = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration('camera_source'), "' == 'realsense' and ",
            "'", LaunchConfiguration('launch_camera'), "' == 'true'"
        ])
    )

    # ==========================================================================
    # Nodes
    # ==========================================================================
    
    # RealSense Camera Node (only when camera_source=realsense and launch_camera=true)
    # Publishes: /camera/color/image_raw, /camera/camera_info, etc.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        namespace='camera',
        output='screen',
        condition=launch_realsense,
        parameters=[{
            'serial_no': LaunchConfiguration('camera_serial'),
            'rgb_camera.profile': [
                LaunchConfiguration('rgb_width'),
                LaunchConfiguration('rgb_height'),
                LaunchConfiguration('rgb_fps')
            ],
            'enable_color': True,
            'enable_depth': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            'align_depth.enable': False,
        }],
        remappings=[
            ('color/image_raw', '/camera/color/image_raw'),
            ('color/camera_info', '/camera/camera_info'),
        ]
    )
    
    # Detector Node - Real YOLO/RF-DETR (only when camera_source=realsense)
    # Subscribes: /camera/color/image_raw
    # Publishes: /detections (Detection2DArray)
    detector_node = Node(
        package='airhound_perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        condition=use_realsense,
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
            'input_image_topic': '/camera/color/image_raw',
            'output_detections_topic': '/detections',
        }]
    )
    
    # Mock Detector Node (only when camera_source=synthetic)
    # Publishes: /detections (Detection2DArray), /camera/camera_info (CameraInfo)
    mock_detector_node = Node(
        package='airhound_perception',
        executable='mock_detector',
        name='mock_detector_node',
        output='screen',
        condition=use_synthetic,
        parameters=[{
            'output_detections_topic': '/detections',
            'frame_id': 'camera_color_optical_frame',
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'moving_target': LaunchConfiguration('moving_target'),
            'motion_type': LaunchConfiguration('motion_type'),
            'motion_radius': LaunchConfiguration('motion_radius'),
            'motion_speed': LaunchConfiguration('motion_speed'),
            'target_width': 120,
            'target_height': 80,
            'confidence': 0.85,
            'class_id': 'drone',
            'num_targets': 1,
            'add_noise': LaunchConfiguration('add_noise'),
            'noise_std': 5.0,
        }]
    )
    
    # Tracking Node (always runs)
    # Subscribes: /detections, /camera/camera_info
    # Publishes: /target_yaw (Float32)
    tracking_node = Node(
        package='tracking_geometry',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{
            'max_rate': LaunchConfiguration('max_rate'),
            'deadband': LaunchConfiguration('deadband'),
        }]
    )
    
    # PX4 Converter Node (always runs)
    # Subscribes: /target_yaw (Float32)
    # Publishes: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
    px4_converter_node = Node(
        package='offboard_control',
        executable='px4_converter_node',
        name='px4_converter_node',
        output='screen',
        parameters=[{
            'auto_arm': LaunchConfiguration('auto_arm'),
            'publish_rate': LaunchConfiguration('px4_publish_rate'),
            'safety_timeout': LaunchConfiguration('safety_timeout'),
        }]
    )

    # ==========================================================================
    # Launch Description
    # ==========================================================================
    
    return LaunchDescription([
        # Declare arguments
        camera_source_arg,
        camera_serial_arg,
        rgb_width_arg,
        rgb_height_arg,
        rgb_fps_arg,
        model_path_arg,
        confidence_threshold_arg,
        device_arg,
        moving_target_arg,
        motion_type_arg,
        motion_speed_arg,
        motion_radius_arg,
        publish_rate_arg,
        image_width_arg,
        image_height_arg,
        add_noise_arg,
        max_rate_arg,
        deadband_arg,
        auto_arm_arg,
        px4_publish_rate_arg,
        safety_timeout_arg,
        launch_camera_arg,
        
        # Safety warnings
        LogInfo(msg='=== AIRHOUND E2E Flight Mode ==='),
        LogInfo(msg='WARNING: This mode controls a REAL drone!'),
        LogInfo(msg=['Camera source: ', LaunchConfiguration('camera_source')]),
        LogInfo(msg='Ensure MicroXRCE Agent is running and PX4 is connected!'),
        
        # Launch nodes
        realsense_node,
        detector_node,
        mock_detector_node,
        tracking_node,
        px4_converter_node,
        
        LogInfo(msg='All nodes started. Monitor detections and yaw commands.'),
    ])
