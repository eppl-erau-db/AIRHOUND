#!/usr/bin/env python3
# Offboard position takeoff example for PX4 (ROS 2, px4_msgs)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

MICROS = 1_000_000


class OffboardControl(Node):
    """Node for controlling a vehicle in OFFBOARD position mode (takeoff to -5 m NED)."""

    def __init__(self) -> None:
        super().__init__("offboard_control_takeoff_and_land")

        # QoS: use RELIABLE; keep TRANSIENT_LOCAL to match PX4 bridges
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        # Subscribers
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._on_local_position,
            qos_profile,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._on_vehicle_status, qos_profile
        )

        # State
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Target: take off to z = -5 m (NED up is negative)
        self.takeoff_height_ned = -5.0

        # Control loop @ 50 Hz
        self.tick_hz = 50.0
        self.tick_period = 1.0 / self.tick_hz
        self.offboard_setpoint_counter = 0

        self.timer = self.create_timer(self.tick_period, self._on_timer)

        self.get_logger().info("Offboard control node ready (50 Hz).")

    # --- Helpers ----------------------------------------------------------------

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_local_position(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        # We command POSITION in this example
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self.offboard_control_mode_pub.publish(msg)

    def _publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        msg = TrajectorySetpoint()
        # Position setpoint in local NED frame
        msg.position = [x, y, z]
        msg.yaw = yaw  # rad
        msg.timestamp = self._now_us()
        self.trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _engage_offboard(self):
        # VEHICLE_CMD_DO_SET_MODE: param1=1 (custom), param2=6 (OFFBOARD)
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to OFFBOARD mode...")

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def _disarm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to LAND...")

    # --- Main control loop -------------------------------------------------------

    def _on_timer(self):
        # Always stream heartbeat + position setpoint to satisfy PX4 OFFBOARD watchdog
        self._publish_offboard_heartbeat()
        self._publish_position_setpoint(0.0, 0.0, self.takeoff_height_ned, yaw=0.0)

        self.offboard_setpoint_counter += 1

        # After ~1s of streaming at 50 Hz, engage OFFBOARD
        if self.offboard_setpoint_counter == 50:
            self._engage_offboard()

        # Arm a short moment after OFFBOARD mode request (gives mode time to flip)
        if self.offboard_setpoint_counter == 60:
            self._arm()

        # Optional: auto-land once we're near the target altitude, while in OFFBOARD
        # vehicle_local_position.z is NED (down positive). Near -5 means we've risen ~5 m.
        if (
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and abs(self.vehicle_local_position.z - self.takeoff_height_ned) < 0.2
            and self.offboard_setpoint_counter > 200  # let it hover ~3s first
        ):
            self._land()
            # Keep streaming for a few more ticks; shutdown in main on Ctrl+C

        # Note: we intentionally do NOT stop publishing setpoints; OFFBOARD requires continuous streaming.

def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

