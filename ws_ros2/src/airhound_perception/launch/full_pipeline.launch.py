"""
Full AIRHOUND Pipeline Launch File

Brings up the complete perception-to-control pipeline:
  1. RealSense D455 camera (RGB + depth)
  2. Perception node (YOLOv8 or RF-DETR detector)
  3. Tracking node (yaw error computation)
  4. PX4 converter node (offboard control bridge)
  5. MicroXRCE-DDS agent (PX4 communication)

Usage:
  # Default (YOLOv8):
  ros2 launch airhound_perception full_pipeline.launch.py

  # With RF-DETR:
  ros2 launch airhound_perception full_pipeline.launch.py detector:=rfdetr

  # Without starting the DDS agent (if running separately):
  ros2 launch airhound_perception full_pipeline.launch.py start_agent:=false

  # Simulation mode (no real camera):
  ros2 launch airhound_perception full_pipeline.launch.py sim_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ==================== ARGUMENTS ====================
    detector_arg = DeclareLaunchArgument(
        'detector',
        default_value='yolo',
        description='Detector type: yolo or rfdetr'
    )
    
    start_agent_arg = DeclareLaunchArgument(
        'start_agent',
        default_value='true',
        description='Start MicroXRCE-DDS agent for PX4 communication'
    )
    
    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Simulation mode (skip real camera)'
    )
    
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',
        description='Auto-arm the drone (DANGEROUS - use only in controlled tests)'
    )

    # ==================== PATHS ====================
    perception_pkg = get_package_share_directory('airhound_perception')
    workspace_root = os.path.abspath(os.path.join(perception_pkg, '..', '..', '..', '..'))
    
    # Config files
    realsense_cfg = os.path.join(workspace_root, 'config', 'realsense_profile.yaml')
    perception_yolo_cfg = os.path.join(perception_pkg, 'config', 'perception.yaml')
    perception_rfdetr_cfg = os.path.join(perception_pkg, 'config', 'perception_rfdetr.yaml')

    # ==================== NODES ====================
    
    # 1. RealSense Camera (only in non-sim mode)
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[realsense_cfg],
        remappings=[
            ('/camera/realsense2_camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/realsense2_camera/color/camera_info', '/camera/camera_info'),
            ('/camera/realsense2_camera/depth/image_rect_raw', '/camera/depth/image_raw'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )
    
    # 2. Perception Node (YOLOv8)
    perception_yolo = Node(
        package='airhound_perception',
        executable='detector_node',
        name='perception_node',
        output='screen',
        parameters=[perception_yolo_cfg],
        condition=UnlessCondition(
            PythonExpression(["'", LaunchConfiguration('detector'), "' == 'rfdetr'"])
        )
    )
    
    # 2. Perception Node (RF-DETR)
    perception_rfdetr = Node(
        package='airhound_perception',
        executable='detector_node',
        name='perception_node',
        output='screen',
        parameters=[perception_rfdetr_cfg],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('detector'), "' == 'rfdetr'"])
        )
    )
    
    # 3. Tracking Node
    tracking_node = Node(
        package='Tracking-Geometry',
        executable='tracking_node',
        name='tracking_node',
        output='screen',
        parameters=[{
            'max_rate': 1.0,
            'deadband': 0.01,
        }]
    )
    
    # 4. PX4 Converter Node
    px4_converter = Node(
        package='offboard_control',
        executable='px4_converter_node',
        name='px4_converter_node',
        output='screen',
        parameters=[{
            'auto_arm': LaunchConfiguration('auto_arm'),
            'publish_rate': 10.0,
            'safety_timeout': 5.0,
        }]
    )
    
    # 5. MicroXRCE-DDS Agent (for PX4 communication)
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_agent'))
    )

    # ==================== LAUNCH ====================
    return LaunchDescription([
        # Arguments
        detector_arg,
        start_agent_arg,
        sim_mode_arg,
        auto_arm_arg,
        
        # Nodes
        realsense_node,
        perception_yolo,
        perception_rfdetr,
        tracking_node,
        px4_converter,
        xrce_agent,
    ])
