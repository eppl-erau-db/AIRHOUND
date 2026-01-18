#!/usr/bin/env python3
"""
AIRHOUND E2E Flight Launch File

Launches the complete end-to-end pipeline for real flight:
  1. RealSense Camera - Intel RealSense D455 RGB camera
  2. Detector Node - YOLOv8/TensorRT object detection
  3. Tracking Node - Converts detections to yaw rate commands
  4. PX4 Converter - Sends yaw commands to PX4 via DDS

Prerequisites:
  - Intel RealSense camera connected
  - YOLO model weights available (TensorRT engine or .pt file)
  - MicroXRCE-DDS Agent running
  - PX4 flight controller connected
  - Workspace built and sourced

Usage:
  ros2 launch airhound e2e_flight.launch.py
  ros2 launch airhound e2e_flight.launch.py model_path:=/path/to/model.engine
  ros2 launch airhound e2e_flight.launch.py max_rate:=0.5 auto_arm:=false

Data Flow:
  RealSense -> detector_node -> /detections -> tracking_node -> /target_yaw -> px4_converter_node -> /fmu/in/* -> PX4

SAFETY WARNING:
  - Always test with auto_arm:=false first!
  - Ensure RC transmitter is ready for manual override
  - Have a spotter when testing outdoors
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    
    # Camera arguments
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
    
    # Detector arguments
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
    
    # Optional: skip camera (if already running)
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch RealSense camera node'
    )

    # ==========================================================================
    # Nodes
    # ==========================================================================
    
    # RealSense Camera Node
    # Publishes: /camera/color/image_raw, /camera/camera_info, etc.
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        namespace='camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_camera')),
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
    
    # Detector Node (YOLO)
    # Subscribes: /camera/color/image_raw
    # Publishes: /detections (Detection2DArray)
    detector_node = Node(
        package='airhound_perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'device': LaunchConfiguration('device'),
            'input_image_topic': '/camera/color/image_raw',
            'output_detections_topic': '/detections',
        }]
    )
    
    # Tracking Node
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
    
    # PX4 Converter Node
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
        camera_serial_arg,
        rgb_width_arg,
        rgb_height_arg,
        rgb_fps_arg,
        model_path_arg,
        confidence_threshold_arg,
        device_arg,
        max_rate_arg,
        deadband_arg,
        auto_arm_arg,
        px4_publish_rate_arg,
        safety_timeout_arg,
        launch_camera_arg,
        
        # Safety warnings
        LogInfo(msg='=== AIRHOUND E2E Flight Mode ==='),
        LogInfo(msg='WARNING: This mode controls a REAL drone!'),
        LogInfo(msg='Starting: camera -> detector -> tracking -> px4_converter'),
        LogInfo(msg='Ensure MicroXRCE Agent is running and PX4 is connected!'),
        
        # Launch nodes
        realsense_node,
        detector_node,
        tracking_node,
        px4_converter_node,
        
        LogInfo(msg='All nodes started. Monitor detections and yaw commands.'),
    ])
