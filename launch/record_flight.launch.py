#!/usr/bin/env python3
"""
AIRHOUND Flight Data Recording

Records all topics needed for post-flight analysis and paper metrics.
Can be launched standalone or included from e2e_flight.launch.py via record:=true.

Output: timestamped rosbag in ~/airhound_bags/flight_YYYYMMDD_HHMMSS/

Topics recorded (grouped by subsystem):

  Perception:
    /detections                    - Detection2DArray (bbox + depth)
    /perception/target_3d          - PointStamped (3D position in camera frame)
    /perception/latency_ms         - Float32 (inference time)
    /perception/fps                - Float32 (detector throughput)
    /perception/depth_available    - Float32 (depth availability flag)
    /perception/depth_quality      - Float32 (valid depth ratio)

  Tracking:
    /yaw_command                   - Float64 (yaw rate command to PX4)
    /tracking/kalman_state         - Float64MultiArray (6-state Kalman output)
    /tracking/mode                 - Float64 (active fallback mode: 0=detect, 1=PINN, 2=Kalman, 3=hold)

  PINN (if enabled):
    /tracking/pinn_predicted       - PointStamped (PINN predicted position)
    /tracking/pinn_state           - Float64MultiArray (PINN full state)
    /tracking/pinn_active          - Bool (PINN currently predicting)

  PX4 Commands (outgoing):
    /fmu/in/trajectory_setpoint    - TrajectorySetpoint (what we sent)
    /fmu/in/offboard_control_mode  - OffboardControlMode
    /fmu/in/vehicle_command        - VehicleCommand (arm/mode requests)

  PX4 Ground Truth (incoming):
    /fmu/out/vehicle_attitude      - VehicleAttitude (quaternion orientation)
    /fmu/out/vehicle_local_position_v1 - VehicleLocalPosition (NED position + velocity)

  Camera (optional, large bandwidth):
    /camera/color/image_raw        - Image (only if record_images:=true)
    /camera/camera_info            - CameraInfo (intrinsics, always recorded)
"""

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    # Arguments
    record_images_arg = DeclareLaunchArgument(
        'record_images',
        default_value='false',
        description='Record raw camera images (large files). Default false.'
    )

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.expanduser('~/airhound_bags'),
        description='Base directory for rosbag output'
    )

    # Timestamp for bag name
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(
        os.path.expanduser('~/airhound_bags'),
        f'flight_{timestamp}'
    )

    # Core topics (always recorded)
    core_topics = [
        # Perception
        '/detections',
        '/perception/target_3d',
        '/perception/latency_ms',
        '/perception/fps',
        '/perception/depth_available',
        '/perception/depth_quality',
        # Tracking
        '/yaw_command',
        '/tracking/kalman_state',
        '/tracking/mode',
        # PINN
        '/tracking/pinn_predicted',
        '/tracking/pinn_state',
        '/tracking/pinn_active',
        # PX4 outgoing
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/vehicle_command',
        # PX4 ground truth
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_local_position_v1',
        # Camera intrinsics (small, always useful)
        '/camera/camera_info',
        # Converter status
        '/px4_converter_status',
    ]

    qos_override = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'config', 'recording_qos.yaml'
    )

    # Record core topics
    record_core = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_path,
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--qos-profile-overrides-path', qos_override,
            *core_topics,
        ],
        output='screen',
    )

    # Optionally record images (separate process so core recording isn't slowed)
    record_images = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', bag_path + '_images',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--qos-profile-overrides-path', qos_override,
            '/camera/color/image_raw',
            '/camera/depth/image_raw',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_images')),
    )

    return LaunchDescription([
        record_images_arg,
        bag_dir_arg,
        LogInfo(msg=f'Recording flight data to: {bag_path}'),
        LogInfo(msg=f'Recording {len(core_topics)} core topics'),
        record_core,
        record_images,
    ])
