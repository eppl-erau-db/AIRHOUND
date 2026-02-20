"""
AIRHOUND Tracking Node with Kalman Filter and PINN Integration.

Converts 2D detections to yaw commands with optional 3D Kalman filtering
and PINN-based dropout prediction. Modular: each component can be
enabled/disabled via ROS parameters.

Data flow (all components enabled):
    /detections --> pixel-to-angle --> yaw_command
    /perception/target_3d --> Kalman filter --> /tracking/kalman_state --> (PINN node)
    /tracking/pinn_predicted --> (used during dropout for yaw command)

Data flow (PINN disabled):
    /detections --> pixel-to-angle --> yaw_command
    /perception/target_3d --> Kalman filter --> Kalman extrapolation during dropout

Data flow (Kalman disabled):
    /detections --> pixel-to-angle --> yaw_command
    (last-rate extrapolation during dropout)

Author: EPPL Lab
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float64, Float64MultiArray
from vision_msgs.msg import Detection2DArray

from tracking_geometry.kalman_filter import create_filter_from_params


class YawErrorNode(Node):
    def __init__(self):
        super().__init__("yaw_error_node")

        # ==================== PARAMETERS ====================
        # Control
        self.max_rate = self.declare_parameter("max_rate", 1.0).value
        self.deadband = self.declare_parameter("deadband", 0.01).value

        # Fallback dropout behavior: hold last command briefly, then decay to zero
        self.hold_time = self.declare_parameter("hold_time", 0.3).value   # seconds
        self.decay_time = self.declare_parameter("decay_time", 1.0).value  # seconds

        # Default camera intrinsics (D455 @ 1280x720, ~69 deg HFOV)
        self.default_fx = self.declare_parameter("default_fx", 615.0).value
        self.default_fy = self.declare_parameter("default_fy", 615.0).value
        self.default_cx = self.declare_parameter("default_cx", 640.0).value
        self.default_cy = self.declare_parameter("default_cy", 360.0).value

        # Kalman filter
        self.enable_kalman = self.declare_parameter("enable_kalman", True).value
        self.kalman_process_noise_pos = self.declare_parameter(
            "kalman_process_noise_pos", 0.1
        ).value
        self.kalman_process_noise_vel = self.declare_parameter(
            "kalman_process_noise_vel", 1.0
        ).value
        self.kalman_measurement_noise = self.declare_parameter(
            "kalman_measurement_noise", 0.05
        ).value
        self.kalman_gate_threshold = self.declare_parameter(
            "kalman_gate_threshold", 3.0
        ).value
        self.kalman_max_missed = self.declare_parameter(
            "kalman_max_missed", 30
        ).value

        # PINN integration
        self.enable_pinn = self.declare_parameter("enable_pinn", False).value
        self.pinn_dropout_timeout = self.declare_parameter(
            "pinn_dropout_timeout", 0.1
        ).value

        # ==================== STATE ====================
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False
        self.using_defaults_warned = False

        # Detection tracking
        self.last_detection_time = None
        self.last_yaw_rate = 0.0

        # Kalman filter
        self.kf = None
        if self.enable_kalman:
            self.kf = create_filter_from_params(
                process_noise_pos=self.kalman_process_noise_pos,
                process_noise_vel=self.kalman_process_noise_vel,
                measurement_noise=self.kalman_measurement_noise,
                use_gating=True,
                gate_threshold=self.kalman_gate_threshold,
            )
            # Override max_missed from parameter
            if hasattr(self.kf, "max_missed"):
                self.kf.max_missed = self.kalman_max_missed
            self.get_logger().info(
                f"Kalman filter ENABLED (gate={self.kalman_gate_threshold}, "
                f"max_missed={self.kalman_max_missed})"
            )
        else:
            self.get_logger().info("Kalman filter DISABLED")

        # PINN state
        self.pinn_prediction = None
        self.pinn_active = False
        if self.enable_pinn:
            self.get_logger().info(
                f"PINN integration ENABLED (dropout_timeout={self.pinn_dropout_timeout}s)"
            )
        else:
            self.get_logger().info("PINN integration DISABLED")

        # ==================== QOS ====================
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_default = QoSProfile(depth=10)

        # ==================== SUBSCRIBERS ====================
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, qos_sensor
        )
        self.create_subscription(
            Detection2DArray, "/detections", self._on_detection, qos_default
        )

        # 3D target position (for Kalman filter input)
        if self.enable_kalman:
            self.create_subscription(
                PointStamped,
                "/perception/target_3d",
                self._on_target_3d,
                qos_default,
            )

        # PINN predictions (for dropout recovery)
        if self.enable_pinn:
            self.create_subscription(
                PointStamped,
                "/tracking/pinn_predicted",
                self._on_pinn_prediction,
                qos_default,
            )
            self.create_subscription(
                Bool,
                "/tracking/pinn_active",
                self._on_pinn_active,
                qos_default,
            )

        # ==================== PUBLISHERS ====================
        self.pub_yaw = self.create_publisher(Float64, "/yaw_command", qos_default)

        # Kalman state output (consumed by PINN node)
        if self.enable_kalman:
            self.pub_kalman_state = self.create_publisher(
                Float64MultiArray, "/tracking/kalman_state", qos_default
            )

        # Tracking status for monitoring
        self.pub_tracking_mode = self.create_publisher(
            Float64, "/tracking/mode", qos_default
        )
        # Mode encoding: 0=zero/no_target, 1=detection, 2=kalman_extrapolation,
        #                3=pinn_prediction, 4=fallback_hold, 5=fallback_decay

        self.get_logger().info("YawErrorNode initialized.")

    # ==================== CALLBACKS ====================

    def _on_camera_info(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}"
            )

    def _on_target_3d(self, msg: PointStamped):
        """Feed 3D position into Kalman filter and publish state."""
        if self.kf is None:
            return

        measurement = np.array([msg.point.x, msg.point.y, msg.point.z])

        # Skip NaN measurements
        if np.any(np.isnan(measurement)):
            return

        now = time.time()

        if not self.kf.is_initialized:
            self.kf.initialize(measurement, timestamp=now)
        else:
            dt = now - self.kf.state.timestamp
            if dt > 0:
                self.kf.predict(dt)
            self.kf.update(measurement)
            self.kf.state.timestamp = now

        # Publish Kalman state for PINN consumption
        state_msg = Float64MultiArray()
        state_msg.data = [float(v) for v in self.kf.state.x]
        self.pub_kalman_state.publish(state_msg)

    def _on_pinn_prediction(self, msg: PointStamped):
        """Store latest PINN prediction for use during dropout."""
        self.pinn_prediction = np.array([msg.point.x, msg.point.y, msg.point.z])

    def _on_pinn_active(self, msg: Bool):
        """Track whether PINN is actively predicting."""
        self.pinn_active = msg.data

    def _on_detection(self, msg: Detection2DArray):
        """Main detection callback: compute yaw command."""
        now_time = time.time()

        if len(msg.detections) > 0:
            self._handle_detection(msg, now_time)
        else:
            self._handle_dropout(now_time)

    # ==================== CORE LOGIC ====================

    def _get_intrinsics(self):
        """Get camera intrinsics, falling back to defaults."""
        if self.camera_info_received:
            return self.fx, self.fy, self.cx, self.cy
        if not self.using_defaults_warned:
            self.get_logger().warn(
                f"No camera_info; using defaults: fx={self.default_fx:.0f}, "
                f"cx={self.default_cx:.0f}"
            )
            self.using_defaults_warned = True
        return self.default_fx, self.default_fy, self.default_cx, self.default_cy

    def _handle_detection(self, msg: Detection2DArray, now_time: float):
        """Process detections: compute and publish yaw command."""
        fx, fy, cx, cy = self._get_intrinsics()

        # Highest confidence detection
        detection = max(
            msg.detections, key=lambda d: d.results[0].hypothesis.score
        )
        u = detection.bbox.center.position.x

        # Pixel offset to angular error
        yaw_err = (u - cx) / fx

        # P controller with deadband and rate limit
        yaw_rate = np.clip(yaw_err, -self.max_rate, self.max_rate)
        if abs(yaw_rate) < self.deadband:
            yaw_rate = 0.0

        self.pub_yaw.publish(Float64(data=float(yaw_rate)))
        self._publish_mode(1.0)  # detection mode

        # Mark Kalman as receiving data
        if self.kf is not None and hasattr(self.kf, "missed_count"):
            self.kf.missed_count = 0

        self.last_detection_time = now_time
        self.last_yaw_rate = yaw_rate

        self.get_logger().debug(
            f"[Detection] u={u:.1f}, err={yaw_err:.4f}, yaw_rate={yaw_rate:.4f}"
        )

    def _handle_dropout(self, now_time: float):
        """No detection this frame: use PINN, Kalman, or last-rate fallback."""
        if self.last_detection_time is None:
            # Never had a detection
            self._publish_mode(0.0)
            return

        dt_since = now_time - self.last_detection_time

        # Strategy 1: PINN prediction (if enabled and active)
        if (
            self.enable_pinn
            and self.pinn_active
            and self.pinn_prediction is not None
            and dt_since > self.pinn_dropout_timeout
        ):
            yaw_rate = self._yaw_from_3d_position(self.pinn_prediction)
            self.pub_yaw.publish(Float64(data=float(yaw_rate)))
            self._publish_mode(3.0)  # PINN mode
            self.get_logger().info(
                f"[PINN] dropout {dt_since:.2f}s, yaw_rate={yaw_rate:.4f}"
            )
            return

        # Strategy 2: Kalman extrapolation (if enabled and initialized)
        if self.enable_kalman and self.kf is not None and self.kf.is_initialized:
            predicted_pos = self.kf.predict_position(dt_since)
            if not np.any(np.isnan(predicted_pos)):
                yaw_rate = self._yaw_from_3d_position(predicted_pos)
                self.pub_yaw.publish(Float64(data=float(yaw_rate)))
                self._publish_mode(2.0)  # Kalman mode

                # Mark missed for gating logic
                if hasattr(self.kf, "mark_missed"):
                    self.kf.mark_missed()

                self.get_logger().info(
                    f"[Kalman] dropout {dt_since:.2f}s, yaw_rate={yaw_rate:.4f}"
                )
                return

        # Strategy 3: Hold-then-decay fallback (always available).
        # Hold the last commanded yaw for hold_time seconds, then linearly
        # decay to zero over decay_time seconds.  Keeps output bounded and
        # stable during dropouts without letting a stale rate run forever.
        if dt_since <= self.hold_time:
            yaw_rate = self.last_yaw_rate
            mode_val, mode_str = 4.0, "Hold"
        elif self.decay_time > 0.0 and dt_since <= (self.hold_time + self.decay_time):
            alpha = 1.0 - (dt_since - self.hold_time) / self.decay_time
            yaw_rate = alpha * self.last_yaw_rate
            mode_val, mode_str = 5.0, "Decay"
        else:
            yaw_rate = 0.0
            mode_val, mode_str = 0.0, "Zero"

        yaw_rate = float(np.clip(yaw_rate, -self.max_rate, self.max_rate))
        if abs(yaw_rate) < self.deadband:
            yaw_rate = 0.0
        self.pub_yaw.publish(Float64(data=yaw_rate))
        self._publish_mode(mode_val)
        self.get_logger().warn(
            f"[Fallback:{mode_str}] dropout {dt_since:.2f}s, yaw_rate={yaw_rate:.4f}"
        )

    def _yaw_from_3d_position(self, position: np.ndarray) -> float:
        """
        Compute yaw command from a 3D position in camera optical frame.

        In camera optical frame: +X is right, +Z is forward.
        Yaw error = atan2(X, Z) — positive X means target is to the right.
        """
        x, y, z = position
        if z <= 0 or np.isnan(z):
            return 0.0
        yaw_err = np.arctan2(x, z)
        yaw_rate = np.clip(yaw_err, -self.max_rate, self.max_rate)
        if abs(yaw_rate) < self.deadband:
            yaw_rate = 0.0
        return float(yaw_rate)

    def _publish_mode(self, mode: float):
        """Publish tracking mode for monitoring."""
        self.pub_tracking_mode.publish(Float64(data=mode))


def main(args=None):
    rclpy.init(args=args)
    node = YawErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
