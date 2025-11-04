import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32
import numpy as np
import tf2_ros

class YawErrorNode(Node):
    def __init__(self):
        super().__init__('yaw_error_node')

        # Parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.max_rate = self.declare_parameter('max_rate', 1.0).value
        self.deadband = self.declare_parameter('deadband', 0.01).value

        # State memory for predictive mode
        self.last_time = None
        self.last_yaw = 0.0
        self.last_rate = 0.0

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)

        # Publisher
        self.pub_yaw = self.create_publisher(Float32, '/target_yaw', 10)

        # TF2 for camera → body frame (optional)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("YawErrorNode initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(f"Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")

    def detection_callback(self, msg: Detection2DArray):
        now = self.get_clock().now()
        target_detected = False

        if len(msg.detections) > 0 and None not in (self.fx, self.fy, self.cx, self.cy):
            # Pick detection with highest confidence
            detection = max(msg.detections, key=lambda d: d.results[0].score)
            u = detection.bbox.center.x
            v = detection.bbox.center.y

            # Pixel, angle (radians)
            ex = (u - self.cx) / self.fx  # horizontal → yaw
            ey = (v - self.cy) / self.fy  # vertical → pitch

            yaw_err = ex  # control only yaw for now

            # P controller + deadband
            yaw_rate = np.clip(yaw_err, -self.max_rate, self.max_rate)
            if abs(yaw_rate) < self.deadband:
                yaw_rate = 0.0

            # Yaw are publishing
            self.pub_yaw.publish(Float32(data=yaw_rate))
            self.get_logger().info(f"[Detection] u={u:.1f}, v={v:.1f}, ex={ex:.4f}, ey={ey:.4f} → yaw_rate={yaw_rate:.4f}")

            # Store for predictive mode
            self.last_time = now
            self.last_yaw = yaw_rate
            target_detected = True

        # Predict
        if not target_detected and self.last_time is not None:
            dt = (now - self.last_time).nanoseconds * 1e-9
            predicted_yaw = np.clip(self.last_yaw + self.last_rate * dt, -self.max_rate, self.max_rate)
            if abs(predicted_yaw) < self.deadband:
                predicted_yaw = 0.0
            self.pub_yaw.publish(Float32(data=predicted_yaw))
            self.get_logger().warn(f"[Predictive] target lost → yaw_rate={predicted_yaw:.4f}")
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


if __name__ == '__main__':
    main()
