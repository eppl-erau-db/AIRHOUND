"""
PX4 Integration Test Launch File
Converts demo-publisher data to PX4 format and publishes to simulation environment
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='30',
        description='Test duration in seconds'
    )
    
    yaw_rate_arg = DeclareLaunchArgument(
        'yaw_rate',
        default_value='2.0',
        description='Rate of yaw command publishing (Hz)'
    )

    return LaunchDescription([
        use_sim_time_arg,
        test_duration_arg,
        yaw_rate_arg,
        
        LogInfo(msg="🚀 Starting PX4 Integration Test with DDS to PX4 conversion"),
        
        # Enhanced demo yaw publisher with more comprehensive test patterns
        Node(
            package='offboard_control',
            executable='demo_publisher_enhanced',
            name='demo_publisher_enhanced',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'publish_rate': LaunchConfiguration('yaw_rate'),
                'test_duration': LaunchConfiguration('test_duration')
            }],
            remappings=[
                ('/demo_output', '/yaw_command')
            ]
        ),
        
        # PX4 converter and publisher - converts demo data to PX4 format
        Node(
            package='offboard_control',
            executable='px4_converter_node',
            name='px4_converter_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'auto_arm': True,
                'publish_rate': 10.0,  # 10 Hz for PX4 setpoints
                'safety_timeout': 5.0
            }],
            remappings=[
                ('/yaw_command', '/yaw_command'),
                ('/fmu/in/offboard_control_mode', '/fmu/in/offboard_control_mode'),
                ('/fmu/in/trajectory_setpoint', '/fmu/in/trajectory_setpoint'),
                ('/fmu/in/vehicle_command', '/fmu/in/vehicle_command')
            ]
        ),
        
        # Test monitor node - analyzes the conversion performance
        Node(
            package='offboard_control',
            executable='integration_test_monitor',
            name='integration_test_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'monitor_topics': ['/yaw_command', '/fmu/in/trajectory_setpoint', '/fmu/out/vehicle_status'],
                'test_duration': LaunchConfiguration('test_duration')
            }]
        ),
        
        LogInfo(msg="✅ PX4 Integration Test nodes started. Monitor output for conversion results."),
    ])