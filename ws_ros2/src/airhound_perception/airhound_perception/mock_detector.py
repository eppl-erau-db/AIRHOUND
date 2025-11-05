#!/usr/bin/env python3
"""
Mock Detection Publisher for Simulation Testing

This node publishes synthetic detections without requiring YOLO or camera input.
Perfect for testing control algorithms in SITL simulation without real perception.

Publishes:
  - /detections (vision_msgs/Detection2DArray): Synthetic object detections
  - /perception/fps (std_msgs/Float32): Mock FPS (always reports target rate)

Use this when:
  - Testing control logic in PX4 SITL
  - No real camera available
  - Don't want to run YOLO inference
  - Need deterministic/controllable detection data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
)
from geometry_msgs.msg import Pose2D

import time
import math


class MockDetectorNode(Node):
    """Publishes synthetic detections for simulation testing"""

    def __init__(self):
        super().__init__("mock_detector_node")

        # Parameters
        self.declare_parameter("output_detections_topic", "/detections")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)

        # Motion parameters for moving target
        self.declare_parameter("moving_target", True)
        self.declare_parameter(
            "motion_type", "circular"
        )  # circular, sinusoidal, static
        self.declare_parameter("motion_radius", 200.0)  # pixels
        self.declare_parameter("motion_speed", 0.5)  # rad/s or Hz

        # Target appearance parameters
        self.declare_parameter("target_width", 120)  # pixels
        self.declare_parameter("target_height", 80)  # pixels
        self.declare_parameter("confidence", 0.85)  # detection confidence
        self.declare_parameter("class_id", "drone")  # class name

        # Multi-target parameters
        self.declare_parameter("num_targets", 1)  # number of detections
        self.declare_parameter("add_noise", True)  # add random noise to position
        self.declare_parameter("noise_std", 5.0)  # pixel noise std dev

        # Get parameters
        det_topic = (
            self.get_parameter("output_detections_topic")
            .get_parameter_value()
            .string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self.image_width = (
            self.get_parameter("image_width").get_parameter_value().integer_value
        )
        self.image_height = (
            self.get_parameter("image_height").get_parameter_value().integer_value
        )

        self.moving_target = (
            self.get_parameter("moving_target").get_parameter_value().bool_value
        )
        self.motion_type = (
            self.get_parameter("motion_type").get_parameter_value().string_value
        )
        self.motion_radius = (
            self.get_parameter("motion_radius").get_parameter_value().double_value
        )
        self.motion_speed = (
            self.get_parameter("motion_speed").get_parameter_value().double_value
        )

        self.target_width = (
            self.get_parameter("target_width").get_parameter_value().integer_value
        )
        self.target_height = (
            self.get_parameter("target_height").get_parameter_value().integer_value
        )
        self.confidence = (
            self.get_parameter("confidence").get_parameter_value().double_value
        )
        self.class_id = (
            self.get_parameter("class_id").get_parameter_value().string_value
        )

        self.num_targets = (
            self.get_parameter("num_targets").get_parameter_value().integer_value
        )
        self.add_noise = (
            self.get_parameter("add_noise").get_parameter_value().bool_value
        )
        self.noise_std = (
            self.get_parameter("noise_std").get_parameter_value().double_value
        )

        # Publishers
        qos_default = QoSProfile(depth=10)
        self.det_pub = self.create_publisher(Detection2DArray, det_topic, qos_default)
        self.fps_pub = self.create_publisher(Float32, "/perception/fps", qos_default)

        # State
        self.start_time = time.time()
        self.frame_count = 0

        # Timer
        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.publish_detections)

        self.get_logger().info(
            f"Mock Detector started: {self.image_width}x{self.image_height} @ {self.publish_rate_hz} Hz"
        )
        self.get_logger().info(
            f"Publishing {self.num_targets} target(s) on {det_topic}"
        )
        self.get_logger().info(
            f"Motion: {self.motion_type}, Moving: {self.moving_target}"
        )

    def get_target_position(self, target_index: int = 0) -> tuple:
        """
        Calculate target position based on motion parameters

        Returns:
            (center_x, center_y) in pixels
        """
        elapsed = time.time() - self.start_time

        # Base position at image center
        base_x = self.image_width / 2.0
        base_y = self.image_height / 2.0

        if not self.moving_target:
            # Static target at center
            return (base_x, base_y)

        if self.motion_type == "circular":
            # Circular motion around center
            angle = elapsed * self.motion_speed + (
                target_index * 2.0 * math.pi / max(1, self.num_targets)
            )
            offset_x = self.motion_radius * math.cos(angle)
            offset_y = self.motion_radius * math.sin(angle) * 0.5  # elliptical

        elif self.motion_type == "sinusoidal":
            # Horizontal sinusoidal motion
            offset_x = self.motion_radius * math.sin(elapsed * self.motion_speed)
            offset_y = (
                target_index * 50.0 - (self.num_targets - 1) * 25.0
            )  # vertical spacing

        elif self.motion_type == "figure8":
            # Figure-8 pattern
            t = elapsed * self.motion_speed
            offset_x = self.motion_radius * math.sin(t)
            offset_y = self.motion_radius * math.sin(2 * t) / 2.0

        else:  # 'static' or unknown
            offset_x = target_index * 100.0 - (self.num_targets - 1) * 50.0
            offset_y = 0.0

        center_x = base_x + offset_x
        center_y = base_y + offset_y

        # Add random noise if enabled
        if self.add_noise:
            import random

            center_x += random.gauss(0, self.noise_std)
            center_y += random.gauss(0, self.noise_std)

        # Clamp to image bounds
        half_w = self.target_width / 2.0
        half_h = self.target_height / 2.0
        center_x = max(half_w, min(self.image_width - half_w, center_x))
        center_y = max(half_h, min(self.image_height - half_h, center_y))

        return (center_x, center_y)

    def create_detection(self, target_index: int = 0) -> Detection2D:
        """Create a single Detection2D message"""

        # Get target position
        center_x, center_y = self.get_target_position(target_index)

        # Create bounding box
        bbox = BoundingBox2D()
        bbox.center.position.x = center_x
        bbox.center.position.y = center_y
        bbox.center.theta = 0.0  # No rotation
        bbox.size_x = float(self.target_width)
        bbox.size_y = float(self.target_height)

        # Create hypothesis (class + confidence)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = self.class_id
        hyp.hypothesis.score = self.confidence

        # Assemble detection
        det = Detection2D()
        det.bbox = bbox
        det.results.append(hyp)

        return det

    def publish_detections(self):
        """Publish mock detections at configured rate"""

        now = self.get_clock().now()

        # Create detection array
        det_array = Detection2DArray()
        det_array.header.stamp = now.to_msg()
        det_array.header.frame_id = self.frame_id

        # Add configured number of targets
        for i in range(self.num_targets):
            detection = self.create_detection(target_index=i)
            det_array.detections.append(detection)

        # Publish detections
        self.det_pub.publish(det_array)

        # Publish mock FPS (just report the configured rate)
        self.fps_pub.publish(Float32(data=float(self.publish_rate_hz)))

        self.frame_count += 1

        # Log periodically
        if self.frame_count % 100 == 0:
            elapsed = time.time() - self.start_time
            actual_fps = self.frame_count / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(
                f"Published {self.frame_count} detection frames "
                f"({actual_fps:.1f} Hz actual, {self.num_targets} targets)"
            )


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
