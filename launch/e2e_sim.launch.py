#!/usr/bin/env python3
"""
AIRHOUND E2E Simulation Launch File

Launches the complete end-to-end pipeline for SITL simulation testing:
  1. Mock Detector - Generates synthetic detections (no camera/YOLO needed)
  2. Tracking Node - Converts detections to yaw commands (+ Kalman + optional PINN)
  3. PX4 Converter - Sends yaw commands to PX4 via DDS
  4. PINN Node (optional) - Predicts trajectory during detection dropout

Prerequisites:
  - PX4 SITL + Gazebo running (make px4_sitl gz_x500)
  - MicroXRCE-DDS Agent running
  - Workspace built and sourced

Usage:
  ros2 launch airhound e2e_sim.launch.py
  ros2 launch airhound e2e_sim.launch.py enable_pinn:=true
  ros2 launch airhound e2e_sim.launch.py motion_type:=sinusoidal enable_kalman:=true

Data Flow:
  mock_detector -> /detections -> tracking_node -> /yaw_command -> px4_converter -> PX4
                                       |
                                       +-> /tracking/kalman_state -> pinn_node (if enabled)
                                       +<- /tracking/pinn_predicted <- pinn_node
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==========================================================================
    # Launch Arguments
    # ==========================================================================

    # Mock detector
    moving_target_arg = DeclareLaunchArgument(
        'moving_target', default_value='true')
    motion_type_arg = DeclareLaunchArgument(
        'motion_type', default_value='circular',
        description='circular, sinusoidal, figure8, static')
    motion_speed_arg = DeclareLaunchArgument(
        'motion_speed', default_value='0.3')
    motion_radius_arg = DeclareLaunchArgument(
        'motion_radius', default_value='10.0')
    target_distance_arg = DeclareLaunchArgument(
        'target_distance', default_value='15.0')
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value='30.0')
    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='1280')
    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='720')
    add_noise_arg = DeclareLaunchArgument(
        'add_noise', default_value='false')

    # Simulated dropout for PINN testing
    dropout_interval_arg = DeclareLaunchArgument(
        'dropout_interval', default_value='0.0',
        description='Seconds between forced dropouts (0=disabled)')
    dropout_duration_arg = DeclareLaunchArgument(
        'dropout_duration', default_value='0.0',
        description='Seconds of forced dropout per interval')

    # Tracking
    max_rate_arg = DeclareLaunchArgument(
        'max_rate', default_value='1.0')
    deadband_arg = DeclareLaunchArgument(
        'deadband', default_value='0.01')
    hold_time_arg = DeclareLaunchArgument(
        'hold_time', default_value='0.3',
        description='Seconds to hold last yaw command during target dropout')
    decay_time_arg = DeclareLaunchArgument(
        'decay_time', default_value='1.0',
        description='Seconds to linearly decay yaw to zero after hold period')

    # Kalman filter
    enable_kalman_arg = DeclareLaunchArgument(
        'enable_kalman', default_value='true',
        description='Enable 3D Kalman filter for state estimation')

    # PINN
    enable_pinn_arg = DeclareLaunchArgument(
        'enable_pinn', default_value='false',
        description='Enable PINN trajectory prediction during dropout')
    pinn_model_arg = DeclareLaunchArgument(
        'pinn_model_path', default_value='models/pinn/pinn_best.onnx')
    pinn_norm_arg = DeclareLaunchArgument(
        'pinn_norm_stats_path', default_value='models/pinn/norm_stats.npz')

    # PX4
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm', default_value='true')
    px4_publish_rate_arg = DeclareLaunchArgument(
        'px4_publish_rate', default_value='10.0')
    safety_timeout_arg = DeclareLaunchArgument(
        'safety_timeout', default_value='5.0')

    # Recording (off by default for sim, on by default for flight)
    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record rosbag for post-flight analysis')

    # ==========================================================================
    # Nodes
    # ==========================================================================

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
            'target_distance': LaunchConfiguration('target_distance'),
            'target_width': 120,
            'target_height': 80,
            'confidence': 0.85,
            'class_id': 'drone',
            'num_targets': 1,
            'add_noise': LaunchConfiguration('add_noise'),
            'noise_std': 5.0,
            'dropout_interval': LaunchConfiguration('dropout_interval'),
            'dropout_duration': LaunchConfiguration('dropout_duration'),
        }]
    )

    tracking_node = Node(
        package='tracking_geometry',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{
            'max_rate': LaunchConfiguration('max_rate'),
            'deadband': LaunchConfiguration('deadband'),
            'hold_time': LaunchConfiguration('hold_time'),
            'decay_time': LaunchConfiguration('decay_time'),
            'enable_kalman': LaunchConfiguration('enable_kalman'),
            'kalman_process_noise_pos': 0.1,
            'kalman_process_noise_vel': 1.0,
            'kalman_measurement_noise': 0.05,
            'kalman_gate_threshold': 3.0,
            'kalman_max_missed': 30,
            'enable_pinn': LaunchConfiguration('enable_pinn'),
            'pinn_dropout_timeout': 0.1,
        }]
    )

    pinn_node = Node(
        package='tracking_geometry',
        executable='pinn_node',
        name='pinn_prediction_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_pinn')),
        parameters=[{
            'model_path': LaunchConfiguration('pinn_model_path'),
            'norm_stats_path': LaunchConfiguration('pinn_norm_stats_path'),
            'prediction_horizon': 0.1,
            'dropout_timeout': 0.1,
            'max_prediction_time': 2.0,
            'use_onnx': True,
            'publish_rate_hz': 30.0,
        }]
    )

    px4_converter_gazebo = Node(
        package='offboard_control',
        executable='px4_converter_gazebo',
        name='px4_converter_gazebo',
        output='screen',
        parameters=[{
            'auto_arm': LaunchConfiguration('auto_arm'),
            'publish_rate': LaunchConfiguration('px4_publish_rate'),
            'safety_timeout': LaunchConfiguration('safety_timeout'),
            'force_arm': True,  # SITL needs force-arm to bypass pre-flight checks
        }]
    )

    # ==========================================================================
    # Data Recording (rosbag)
    # ==========================================================================

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(os.path.expanduser('~/airhound_bags'), f'sim_{timestamp}')

    record_topics = [
        '/detections',
        '/perception/target_3d',
        '/perception/latency_ms',
        '/perception/fps',
        '/perception/depth_available',
        '/perception/depth_quality',
        '/yaw_command',
        '/tracking/kalman_state',
        '/tracking/mode',
        '/tracking/pinn_predicted',
        '/tracking/pinn_state',
        '/tracking/pinn_active',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/vehicle_command',
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_local_position_v1',
        '/camera/camera_info',
        '/px4_converter_status',
    ]

    qos_override = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'config', 'recording_qos.yaml'
    )

    record_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_path,
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--qos-profile-overrides-path', qos_override,
            *record_topics,
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('record')),
        sigterm_timeout='5',   # give recorder 5s to finalize MCAP on SIGTERM
        sigkill_timeout='10',  # give 10s before SIGKILL
    )

    # ==========================================================================
    # Launch Description
    # ==========================================================================

    return LaunchDescription([
        # Arguments
        moving_target_arg,
        motion_type_arg,
        motion_speed_arg,
        motion_radius_arg,
        target_distance_arg,
        publish_rate_arg,
        image_width_arg,
        image_height_arg,
        add_noise_arg,
        dropout_interval_arg,
        dropout_duration_arg,
        max_rate_arg,
        deadband_arg,
        hold_time_arg,
        decay_time_arg,
        enable_kalman_arg,
        enable_pinn_arg,
        pinn_model_arg,
        pinn_norm_arg,
        auto_arm_arg,
        px4_publish_rate_arg,
        safety_timeout_arg,
        record_arg,

        # Info
        LogInfo(msg='=== AIRHOUND E2E Simulation ==='),
        LogInfo(msg='mock_detector -> tracking (Kalman/PINN) -> px4_converter'),

        # Nodes
        mock_detector_node,
        tracking_node,
        pinn_node,
        px4_converter_gazebo,

        # Recording
        record_bag,
    ])
