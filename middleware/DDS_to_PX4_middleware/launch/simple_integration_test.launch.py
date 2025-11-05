"""
Simple Integration Test Launch File (No px4_msgs required)
Tests the DDS to PX4 conversion workflow using standard ROS messages
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
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
        
        LogInfo(msg="🚀 Starting Simple Integration Test (no px4_msgs required)"),
        
        # Enhanced demo yaw publisher
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
        
        # Simple PX4 converter (uses standard ROS messages)
        Node(
            package='offboard_control',
            executable='px4_converter_node_simple',
            name='px4_converter_node_simple',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'auto_arm': True,
                'publish_rate': 10.0,
                'safety_timeout': 5.0
            }]
        ),
        
        # Test monitor node
        Node(
            package='offboard_control',
            executable='integration_test_monitor',
            name='integration_test_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'monitor_topics': ['/yaw_command', '/px4_setpoint', '/px4_converter_status'],
                'test_duration': LaunchConfiguration('test_duration')
            }]
        ),
        
        LogInfo(msg="✅ Simple integration test nodes started. Check output for conversion results."),
    ])