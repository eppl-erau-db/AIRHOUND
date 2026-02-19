#!/usr/bin/env python3
"""
Closed-Loop Mock Detection Publisher for SITL Testing

Simulates a target at a world position, projects it into the drone's camera
frame using the drone's actual attitude from PX4. This closes the feedback
loop: when the drone yaws toward the target, the target moves toward image
center, just like a real camera would see.

Publishes:
  - /detections (vision_msgs/Detection2DArray): Synthetic object detections
  - /perception/fps (std_msgs/Float32): Mock FPS

Subscribes:
  - /fmu/out/vehicle_attitude (px4_msgs/VehicleAttitude): Drone orientation
  - /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition): Drone pos
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

import time
import math
import random


def quat_to_yaw(q: list) -> float:
    """Extract yaw from PX4 quaternion [w, x, y, z] (NED frame)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class MockDetectorNode(Node):
    """Closed-loop mock detector: world-space target projected through drone camera."""

    def __init__(self):
        super().__init__("mock_detector_node")

        # === Parameters ===
        self.declare_parameter("output_detections_topic", "/detections")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)

        # Target world position: where the target orbits (NED, meters)
        self.declare_parameter("target_distance", 15.0)  # meters from drone
        self.declare_parameter("target_altitude", -5.0)  # NED (same as drone)

        # Motion in world space
        self.declare_parameter("moving_target", True)
        self.declare_parameter("motion_type", "circular")
        self.declare_parameter("motion_speed", 0.3)  # rad/s in world
        self.declare_parameter("motion_radius", 10.0)  # meters

        # Target appearance
        self.declare_parameter("target_width", 120)   # pixels at reference dist
        self.declare_parameter("target_height", 80)
        self.declare_parameter("confidence", 0.85)
        self.declare_parameter("class_id", "drone")

        # Noise
        self.declare_parameter("num_targets", 1)
        self.declare_parameter("add_noise", True)
        self.declare_parameter("noise_std", 3.0)  # pixels

        # Camera intrinsics (D435i-like for 1280x720)
        self.declare_parameter("fx", 615.0)
        self.declare_parameter("fy", 615.0)

        # === Read parameters ===
        det_topic = self.get_parameter("output_detections_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value

        self.target_distance = self.get_parameter("target_distance").value
        self.target_altitude = self.get_parameter("target_altitude").value

        self.moving_target = self.get_parameter("moving_target").value
        self.motion_type = self.get_parameter("motion_type").value
        self.motion_speed = self.get_parameter("motion_speed").value
        self.motion_radius = self.get_parameter("motion_radius").value

        self.target_width = self.get_parameter("target_width").value
        self.target_height = self.get_parameter("target_height").value
        self.confidence = self.get_parameter("confidence").value
        self.class_id = self.get_parameter("class_id").value

        self.num_targets = self.get_parameter("num_targets").value
        self.add_noise = self.get_parameter("add_noise").value
        self.noise_std = self.get_parameter("noise_std").value

        self.fx = self.get_parameter("fx").value
        self.fy = self.get_parameter("fy").value
        self.cx = self.image_width / 2.0
        self.cy = self.image_height / 2.0

        # === Subscribers (PX4 attitude) ===
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleAttitude, "/fmu/out/vehicle_attitude", self._attitude_cb, qos_px4
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._position_cb,
            qos_px4,
        )

        # === Publishers ===
        qos_default = QoSProfile(depth=10)
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos_default)
        self.fps_pub = self.create_publisher(Float32, "/perception/fps", qos_default)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_default
        )

        # === State ===
        self.drone_yaw = 0.0  # radians, NED heading
        self.drone_pos = [0.0, 0.0, -5.0]  # NED [north, east, down]
        self.initial_yaw = None  # captured on first attitude message
        self.start_time = time.time()
        self.frame_count = 0
        self.has_attitude = False

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

        self.get_logger().info(
            f"Closed-loop mock detector: {self.image_width}x{self.image_height} "
            f"@ {self.publish_rate_hz} Hz"
        )
        self.get_logger().info(
            f"Target at {self.target_distance}m, motion={self.motion_type} "
            f"@ {self.motion_speed} rad/s, radius={self.motion_radius}m"
        )

    # ------------------------------------------------------------------
    # PX4 callbacks
    # ------------------------------------------------------------------

    def _attitude_cb(self, msg: VehicleAttitude):
        self.drone_yaw = quat_to_yaw(msg.q)
        if self.initial_yaw is None:
            self.initial_yaw = self.drone_yaw
            self.get_logger().info(
                f"Initial drone heading captured: {math.degrees(self.initial_yaw):.1f}°"
            )
        self.has_attitude = True

    def _position_cb(self, msg: VehicleLocalPosition):
        self.drone_pos = [msg.x, msg.y, msg.z]

    # ------------------------------------------------------------------
    # World-space target position
    # ------------------------------------------------------------------

    def _target_world_position(self, target_index: int = 0) -> tuple:
        """
        Returns (north, east, down) of the target in NED local frame.

        The target orbits at `target_distance` meters from the drone,
        centered on the drone's initial heading so it starts in view.
        """
        elapsed = time.time() - self.start_time

        # Base heading: place target where the drone is initially looking
        base_yaw = self.initial_yaw if self.initial_yaw is not None else 0.0

        if not self.moving_target:
            # Static target straight ahead of initial heading
            return (
                self.target_distance * math.cos(base_yaw),
                self.target_distance * math.sin(base_yaw),
                self.target_altitude,
            )

        # Base angle for this target (spread multiple targets evenly)
        phase = target_index * 2.0 * math.pi / max(1, self.num_targets)

        if self.motion_type == "circular":
            angle = elapsed * self.motion_speed + phase + base_yaw
            north = self.target_distance * math.cos(angle)
            east = self.target_distance * math.sin(angle)

        elif self.motion_type == "sinusoidal":
            # Swings left-right relative to initial heading
            lateral = self.motion_radius * math.sin(
                elapsed * self.motion_speed + phase
            )
            # Forward along initial heading, lateral perpendicular
            north = self.target_distance * math.cos(base_yaw) - lateral * math.sin(base_yaw)
            east = self.target_distance * math.sin(base_yaw) + lateral * math.cos(base_yaw)

        elif self.motion_type == "figure8":
            t = elapsed * self.motion_speed + phase
            fwd = self.target_distance + self.motion_radius * math.sin(t)
            lat = self.motion_radius * math.sin(2.0 * t) / 2.0
            north = fwd * math.cos(base_yaw) - lat * math.sin(base_yaw)
            east = fwd * math.sin(base_yaw) + lat * math.cos(base_yaw)

        else:
            north = self.target_distance
            east = 0.0

        return (north, east, self.target_altitude)

    # ------------------------------------------------------------------
    # World → pixel projection
    # ------------------------------------------------------------------

    def _project_to_pixel(self, target_ned: tuple) -> tuple:
        """
        Project a world NED point into pixel coordinates given drone pose.

        Camera is forward-facing, aligned with drone body X axis.
        Returns (u, v, in_fov) where in_fov indicates visibility.
        """
        tn, te, td = target_ned
        dn, de, dd = self.drone_pos

        # Vector from drone to target in NED
        delta_n = tn - dn
        delta_e = te - de
        delta_d = td - dd

        # Rotate into body frame (yaw only; assume level flight)
        cos_y = math.cos(self.drone_yaw)
        sin_y = math.sin(self.drone_yaw)

        # Body frame: X = forward, Y = right, Z = down
        body_x = cos_y * delta_n + sin_y * delta_e   # forward
        body_y = -sin_y * delta_n + cos_y * delta_e   # right
        body_z = delta_d                               # down

        # Target must be in front of camera
        if body_x < 0.5:
            return (0.0, 0.0, False)

        # Camera optical frame: Z = forward, X = right, Y = down
        cam_z = body_x  # depth (forward)
        cam_x = body_y  # right
        cam_y = body_z  # down

        # Pinhole projection
        u = self.fx * (cam_x / cam_z) + self.cx
        v = self.fy * (cam_y / cam_z) + self.cy

        # Check if within image bounds (with margin)
        margin = 50
        in_fov = (
            -margin < u < self.image_width + margin
            and -margin < v < self.image_height + margin
        )

        return (u, v, in_fov)

    # ------------------------------------------------------------------
    # Detection creation
    # ------------------------------------------------------------------

    def _create_detection(self, u: float, v: float, distance: float) -> Detection2D:
        """Create a Detection2D at pixel (u, v) with size scaled by distance."""
        # Scale bbox size by distance (closer = bigger)
        ref_dist = 10.0  # reference distance for nominal bbox size
        scale = ref_dist / max(distance, 1.0)
        w = self.target_width * scale
        h = self.target_height * scale

        # Add noise
        if self.add_noise:
            u += random.gauss(0, self.noise_std)
            v += random.gauss(0, self.noise_std)

        # Clamp to image
        u = max(w / 2, min(self.image_width - w / 2, u))
        v = max(h / 2, min(self.image_height - h / 2, v))

        bbox = BoundingBox2D()
        bbox.center.position.x = u
        bbox.center.position.y = v
        bbox.center.theta = 0.0
        bbox.size_x = float(w)
        bbox.size_y = float(h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.class_id
        hyp.hypothesis.score = self.confidence

        det = Detection2D()
        det.bbox = bbox
        det.results.append(hyp)
        return det

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------

    def _tick(self):
        """Publish detections each frame."""
        if not self.has_attitude:
            return  # Wait for first attitude msg

        now = self.get_clock().now()

        # Publish camera info
        self._publish_camera_info(now)

        # Build detection array
        det_array = Detection2DArray()
        det_array.header.stamp = now.to_msg()
        det_array.header.frame_id = self.frame_id

        visible_count = 0
        for i in range(self.num_targets):
            target_ned = self._target_world_position(i)
            u, v, in_fov = self._project_to_pixel(target_ned)

            if in_fov:
                dn, de, dd = self.drone_pos
                dist = math.sqrt(
                    (target_ned[0] - dn) ** 2
                    + (target_ned[1] - de) ** 2
                    + (target_ned[2] - dd) ** 2
                )
                det = self._create_detection(u, v, dist)
                det_array.detections.append(det)
                visible_count += 1

        self.det_pub.publish(det_array)
        self.fps_pub.publish(Float32(data=float(self.publish_rate_hz)))
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            elapsed = time.time() - self.start_time
            actual_fps = self.frame_count / elapsed if elapsed > 0 else 0.0
            bearing = math.degrees(self.drone_yaw)
            target0 = self._target_world_position(0)
            target_angle = math.degrees(
                math.atan2(target0[1] - self.drone_pos[1],
                           target0[0] - self.drone_pos[0])
            )
            self.get_logger().info(
                f"Frame {self.frame_count} ({actual_fps:.1f} Hz) | "
                f"visible={visible_count}/{self.num_targets} | "
                f"drone_yaw={bearing:.1f}° | target_bearing={target_angle:.1f}°"
            )

    def _publish_camera_info(self, now):
        msg = CameraInfo()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.width = self.image_width
        msg.height = self.image_height
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
