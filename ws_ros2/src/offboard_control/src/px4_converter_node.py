#!/usr/bin/env python3
"""
PX4 Converter Node (Python)
Converts yaw commands to PX4 offboard control messages
Python version to avoid C++ template compatibility issues with ROS2 Jazzy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float64, String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import math

# Debug flag
DEBUG = True


class PX4ConverterNode(Node):
    def __init__(self):
        super().__init__('px4_converter_node')

        # Parameters
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('safety_timeout', 5.0)

        self.auto_arm = self.get_parameter('auto_arm').value
        publish_rate = self.get_parameter('publish_rate').value
        self.safety_timeout = self.get_parameter('safety_timeout').value

        # Setup QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers (PX4 topics)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/px4_converter_status', 10)

        # Subscriber for yaw commands
        self.yaw_command_sub = self.create_subscription(
            Float64, '/yaw_command', self.yaw_command_callback, 10)

        # Control timer
        timer_period = 1.0 / publish_rate
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)

        # Safety timer
        self.safety_timer = self.create_timer(1.0, self.safety_timer_callback)

        # State variables
        self.target_yaw = 0.0
        self.has_received_command = False
        self.control_counter = 0
        self.command_count = 0
        self.setpoint_count = 0
        self.last_command_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        self.get_logger().info('PX4 Converter Node (Python) started')
        self.get_logger().info(f'Publishing at {publish_rate} Hz, auto_arm: {self.auto_arm}, safety_timeout: {self.safety_timeout}s')

    def yaw_command_callback(self, msg):
        """Convert demo command to PX4 format"""
        self.target_yaw = float(msg.data)
        self.last_command_time = self.get_clock().now()
        self.command_count += 1

        # Normalize yaw to [-π, π] range for PX4
        while self.target_yaw > math.pi:
            self.target_yaw -= 2 * math.pi
        while self.target_yaw < -math.pi:
            self.target_yaw += 2 * math.pi

        degrees = self.target_yaw * 180.0 / math.pi
        self.get_logger().info(
            f' Converted yaw command {self.command_count}: {self.target_yaw:.3f} rad ({degrees:.1f}°) → PX4 setpoint')

        self.has_received_command = True

    def control_timer_callback(self):
        """Main control loop"""
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Publish offboard control mode
        self.publish_offboard_control_mode(timestamp)

        # Publish trajectory setpoint with converted yaw
        self.publish_trajectory_setpoint(timestamp)

        # Handle arming sequence - CORRECT ORDER like PX4 example!
        # First send 10 setpoints, THEN offboard mode, THEN arm
        if self.auto_arm and self.control_counter == 10:
            self.send_offboard_command(timestamp)
            self.get_logger().info('IMPORTANT: Step 1/2: Switched to OFFBOARD mode')
        elif self.auto_arm and self.control_counter == 11:
            self.send_arm_command(timestamp)
            self.get_logger().info('IMPORTANT: Step 2/2: ARM command sent')

        self.control_counter += 1

        # Publish status every second
        if self.control_counter % 10 == 0:
            self.publish_status_update()

    def safety_timer_callback(self):
        """Check for command timeouts"""
        current_time = self.get_clock().now()
        time_since_command = (current_time - self.last_command_time).nanoseconds / 1e9

        if self.has_received_command and time_since_command > self.safety_timeout:
            self.get_logger().warn(
                f'Safety timeout: No yaw commands for {time_since_command:.1f}s (timeout: {self.safety_timeout}s)')

    def publish_offboard_control_mode(self, timestamp):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True  # Use POSITION control (like PX4 example!)
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = timestamp

        self.offboard_control_mode_pub.publish(msg)

        if DEBUG and self.control_counter % 50 == 0:
            self.get_logger().info(f'Publishing offboard_control_mode: position={msg.position}')

    def publish_trajectory_setpoint(self, timestamp):
        """Publish trajectory setpoint with yaw"""
        msg = TrajectorySetpoint()

        # Maintain current position (hover) - SIMPLIFIED like PX4 example
        msg.position = [0.0, 0.0, -5.0]  # -5m altitude (NED frame)

        # Apply converted yaw command - JUST yaw and timestamp!
        msg.yaw = self.target_yaw
        msg.timestamp = timestamp

        self.trajectory_setpoint_pub.publish(msg)
        self.setpoint_count += 1

        if DEBUG and self.setpoint_count % 50 == 0:
            self.get_logger().info(f'Publishing trajectory_setpoint #{self.setpoint_count}: yaw={self.target_yaw:.3f}')

    def send_arm_command(self, timestamp):
        """Send ARM command to PX4"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1 to arm, 0 to disarm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = timestamp

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('IMPORTANT: ARM command sent to PX4')

    def send_offboard_command(self, timestamp):
        """Send OFFBOARD mode command to PX4"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # main mode
        msg.param2 = 6.0  # offboard mode
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = timestamp

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('IMPORTANT: OFFBOARD mode command sent to PX4')

    def publish_status_update(self):
        """Publish status message"""
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        status_msg = String()
        status_msg.data = (f'PX4 Converter Status - Commands: {self.command_count}, '
                          f'Setpoints: {self.setpoint_count}, '
                          f'Current yaw: {self.target_yaw:.3f} rad, '
                          f'Runtime: {elapsed_time:.1f}s')
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4ConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
