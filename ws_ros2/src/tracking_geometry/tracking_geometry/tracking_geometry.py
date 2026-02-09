import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray


from std_msgs.msg import Float64
import numpy as np
import tf2_ros

from tracking_geometry.kalman_filter import create_filter_from_params


class YawErrorNode(Node):
    def __init__(self):
        super().__init__("yaw_error_node")

        # Parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.max_rate = self.declare_parameter("max_rate", 1.0).value
        self.deadband = self.declare_parameter("deadband", 0.01).value
        
        # Default camera intrinsics (used if no camera_info received)
        # Based on Intel RealSense D455 @ 1280x720 with ~69° HFOV
        self.default_fx = self.declare_parameter("default_fx", 615.0).value
        self.default_fy = self.declare_parameter("default_fy", 615.0).value
        self.default_cx = self.declare_parameter("default_cx", 640.0).value  # 1280/2
        self.default_cy = self.declare_parameter("default_cy", 360.0).value  # 720/2
        
        # Flag to track if we've warned about using defaults
        self.using_defaults = False
        self.camera_info_received = False

        # State memory for predictive mode
        self.last_time = None
        self.last_yaw = 0.0
        self.last_rate = 0.0

        # Subscribers
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, 10
        )
        self.create_subscription(
            Detection2DArray, "/detections", self.detection_callback, 10
        )

        # Publisher — matches offboard_control subscriber on /yaw_command (Float64)
        self.pub_yaw = self.create_publisher(Float64, "/yaw_command", 10)

        # TF2 for camera → body frame (optional)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("YawErrorNode initialized.")
        self.get_logger().info(
            f"Default intrinsics: fx={self.default_fx:.1f}, fy={self.default_fy:.1f}, "
            f"cx={self.default_cx:.1f}, cy={self.default_cy:.1f}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True
        self.get_logger().info(
            f"Camera intrinsics received: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}"
        )
    
    def _get_intrinsics(self):
        """Get camera intrinsics, falling back to defaults if not received."""
        if self.camera_info_received:
            return self.fx, self.fy, self.cx, self.cy
        
        # Use defaults and warn once
        if not self.using_defaults:
            self.get_logger().warn(
                f"No camera_info received, using default intrinsics: "
                f"fx={self.default_fx:.1f}, fy={self.default_fy:.1f}, "
                f"cx={self.default_cx:.1f}, cy={self.default_cy:.1f}"
            )
            self.using_defaults = True
        
        return self.default_fx, self.default_fy, self.default_cx, self.default_cy

    def detection_callback(self, msg: Detection2DArray):
        now = self.get_clock().now()
        target_detected = False

        if len(msg.detections) > 0:
            # Get intrinsics (from camera_info or defaults)
            fx, fy, cx, cy = self._get_intrinsics()
            
            # Pick detection with highest confidence
            detection = max(msg.detections, key=lambda d: d.results[0].hypothesis.score)
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y

            # Pixel, angle (radians)
            ex = (u - cx) / fx  # horizontal → yaw
            ey = (v - cy) / fy  # vertical → pitch

            yaw_err = ex  # control only yaw for now

            # P controller + deadband
            yaw_rate = np.clip(yaw_err, -self.max_rate, self.max_rate)
            if abs(yaw_rate) < self.deadband:
                yaw_rate = 0.0

            self.pub_yaw.publish(Float64(data=float(yaw_rate)))
            self.get_logger().info(
                f"[Detection] u={u:.1f}, v={v:.1f}, ex={ex:.4f}, ey={ey:.4f} → yaw_rate={yaw_rate:.4f}"
            )

            # Store for predictive mode
            self.last_time = now
            self.last_yaw = yaw_rate
            target_detected = True

        # Predict
        if not target_detected and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            predicted_yaw = np.clip(
                self.last_yaw + self.last_rate * dt, -self.max_rate, self.max_rate
            )
            if abs(predicted_yaw) < self.deadband:
                predicted_yaw = 0.0
            self.pub_yaw.publish(Float64(data=float(predicted_yaw)))
            self.get_logger().warn(
                f"[Predictive] target lost → yaw_rate={predicted_yaw:.4f}"
            )
            self.last_rate = predicted_yaw


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