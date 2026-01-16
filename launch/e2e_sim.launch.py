#!/usr/bin/env python3
"""
AIRHOUND E2E Simulation Launch File

Launches the complete end-to-end pipeline for SITL simulation testing:
  1. Mock Detector - Generates synthetic detections (no camera/YOLO needed)
  2. Tracking Node - Converts detections to yaw rate commands
  3. PX4 Converter - Sends yaw commands to PX4 via DDS

Prerequisites:
  - PX4 SITL + Gazebo running (make px4_sitl gz_x500)
  - MicroXRCE-DDS Agent running (./middleware/start_microxrce_agent.sh)
  - Workspace built and sourced

Usage:
  ros2 launch airhound e2e_sim.launch.py
  ros2 launch airhound e2e_sim.launch.py config_file:=/path/to/custom.yaml
  ros2 launch airhound e2e_sim.launch.py motion_type:=sinusoidal max_rate:=0.5

Data Flow:
  mock_detector -> /detections -> tracking_node -> /target_yaw -> px4_converter_node -> /fmu/in/* -> PX4
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    
    # Config file path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to YAML config file (optional, uses defaults if not provided)'
    )
    
    # Mock detector arguments
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
        description='Detection publish rate in Hz'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Simulated image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Simulated image height'
    )
    
    add_noise_arg = DeclareLaunchArgument(
        'add_noise',
        default_value='false',
        description='Add random noise to target positions'
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
        default_value='true',
        description='Automatically arm and enter offboard mode'
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

    # ==========================================================================
    # Nodes
    # ==========================================================================
    
    # Mock Detector Node
    # Publishes: /detections (Detection2DArray), /camera/camera_info (CameraInfo)
    mock_detector_node = Node(
        package='airhound_perception',
        executable='mock_detector',
        name='mock_detector_node',
        output='screen',
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
    
    # Tracking Node
    # Subscribes: /detections, /camera/camera_info
    # Publishes: /target_yaw (Float32)
    tracking_node = Node(
        package='Tracking-Geometry',
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
        config_file_arg,
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
        
        # Info messages
        LogInfo(msg='=== AIRHOUND E2E Simulation Mode ==='),
        LogInfo(msg='Starting: mock_detector -> tracking -> px4_converter'),
        LogInfo(msg='Make sure PX4 SITL + Gazebo and MicroXRCE Agent are running!'),
        
        # Launch nodes
        mock_detector_node,
        tracking_node,
        px4_converter_node,
        
        LogInfo(msg='All nodes started. Watch Gazebo for drone movement!'),
    ])
