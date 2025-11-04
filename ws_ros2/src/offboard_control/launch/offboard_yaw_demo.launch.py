from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('offboard_control')
    
    # Declare launch arguments
    hover_height_arg = DeclareLaunchArgument(
        'hover_height',
        default_value='5.0',
        description='Hover height in meters'
    )
    
    stream_rate_arg = DeclareLaunchArgument(
        'stream_rate',
        default_value='10.0',
        description='Command streaming rate in Hz'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    # Start micro-xrce-dds agent
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen'
    )
    
    # Yaw controller node
    yaw_controller = Node(
        package='offboard_control',
        executable='yaw_controller_node',
        name='yaw_controller',
        output='screen',
        parameters=[{
            'hover_height': LaunchConfiguration('hover_height'),
            'stream_rate_hz': LaunchConfiguration('stream_rate'),
            'enable_debug': LaunchConfiguration('debug'),
            'pre_stream_cycles': 50,
            'target_timeout_sec': 1.0
        }]
    )
    
    # Optional: Test publisher for yaw commands
    test_publisher = Node(
        package='ros2',
        executable='topic',
        name='test_yaw_publisher',
        output='screen',
        arguments=['pub', '-r', '1', '/target_yaw', 'std_msgs/Float32', '{data: 0.0}']
    )
    
    return LaunchDescription([
        hover_height_arg,
        stream_rate_arg,
        debug_arg,
        xrce_agent,
        yaw_controller,
        # test_publisher  # Uncomment to auto-publish test commands
    ])

