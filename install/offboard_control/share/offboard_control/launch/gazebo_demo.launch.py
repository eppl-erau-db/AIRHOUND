"""
Gazebo Demo Launch File
Runs the DDS to PX4 conversion with real PX4 topics for Gazebo integration
This will make the drone actually move in Gazebo!
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    test_duration_arg = DeclareLaunchArgument(
        'test_duration',
        default_value='60',
        description='Demo duration in seconds (60s = full test cycle)'
    )
    
    yaw_rate_arg = DeclareLaunchArgument(
        'yaw_rate',
        default_value='0.5',
        description='Rate of yaw command publishing (Hz) - slower for Gazebo'
    )
    
    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='true',
        description='Automatically arm the vehicle and enter offboard mode'
    )

    return LaunchDescription([
        test_duration_arg,
        yaw_rate_arg,
        auto_arm_arg,
        
        LogInfo(msg="🚁 Starting PX4 Gazebo Demo - Your drone will move!"),
        LogInfo(msg="⚠️  Make sure PX4 SITL and Gazebo are running first"),
        
        # Enhanced demo yaw publisher (slower for visible movement)
        Node(
            package='offboard_control',
            executable='demo_publisher_enhanced',
            name='demo_publisher_enhanced',
            output='screen',
            parameters=[{
                'publish_rate': LaunchConfiguration('yaw_rate'),
                'test_duration': LaunchConfiguration('test_duration')
            }],
            remappings=[
                ('/demo_output', '/yaw_command')
            ]
        ),
        
        # Real PX4 converter for Gazebo integration
        Node(
            package='offboard_control',
            executable='px4_converter_gazebo',
            name='px4_converter_gazebo',
            output='screen',
            parameters=[{
                'auto_arm': LaunchConfiguration('auto_arm'),
                'publish_rate': 10.0,  # 10 Hz for PX4
                'safety_timeout': 10.0  # Longer timeout for demo
            }]
        ),
        
        # Monitor for status updates
        Node(
            package='offboard_control',
            executable='integration_test_monitor',
            name='gazebo_monitor',
            output='screen',
            parameters=[{
                'monitor_topics': ['/yaw_command', '/fmu/in/trajectory_setpoint', '/px4_converter_status'],
                'test_duration': LaunchConfiguration('test_duration')
            }]
        ),
        
        LogInfo(msg="✅ Gazebo demo nodes started!"),
        LogInfo(msg="🎯 Watch your drone rotate through 23 different angles"),
        LogInfo(msg="📊 Monitor the terminal for conversion statistics"),
    ])