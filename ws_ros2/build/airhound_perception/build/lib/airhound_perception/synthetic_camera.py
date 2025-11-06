#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header

import time
import math


class SyntheticCameraNode(Node):
    def __init__(self):
        super().__init__('synthetic_camera_node')

        # Parameters
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('compressed_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('moving_target', True)

        # Get parameters
        self.rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.width = int(self.get_parameter('image_width').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('image_height').get_parameter_value().integer_value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        compressed_topic = self.get_parameter('compressed_topic').get_parameter_value().string_value
        self.publish_compressed = self.get_parameter('publish_compressed').get_parameter_value().bool_value
        self.moving_target = self.get_parameter('moving_target').get_parameter_value().bool_value

        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, image_topic, qos_sensor)
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(CompressedImage, compressed_topic, qos_sensor)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, qos_sensor)

        # CV Bridge
        self.bridge = CvBridge()

        # Animation state
        self.frame_count = 0
        self.start_time = time.time()

        # Create timer
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info(f'Synthetic camera started: {self.width}x{self.height} @ {self.rate} Hz')
        self.get_logger().info(f'Publishing on: {image_topic}')
        if self.publish_compressed:
            self.get_logger().info(f'Publishing compressed on: {compressed_topic}')

    def create_camera_info(self) -> CameraInfo:
        """Create a realistic CameraInfo message for D435i-like camera."""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.frame_id

        # Typical D435i intrinsics for 1280x720
        fx = 910.0  # focal length in pixels
        fy = 910.0
        cx = self.width / 2.0  # principal point
        cy = self.height / 2.0

        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.distortion_model = "plumb_bob"

        # Camera matrix (3x3)
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        # Projection matrix (3x4)
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Distortion coefficients (5 parameters for plumb_bob)
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix (3x3) - identity for monocular
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]

        return camera_info

    def create_test_image(self) -> np.ndarray:
        """Create a test image with synthetic targets."""
        # Create base image
        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add gradient background
        for y in range(self.height):
            for x in range(self.width):
                img[y, x] = [50 + (x * 100) // self.width,
                           80 + (y * 100) // self.height,
                           60]

        # Add some noise for realism
        noise = np.random.randint(-20, 20, (self.height, self.width, 3), dtype=np.int16)
        img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

        # Add synthetic targets
        if self.moving_target:
            # Moving target - circular motion
            elapsed = time.time() - self.start_time
            center_x = self.width // 2 + int(200 * math.cos(elapsed * 0.5))
            center_y = self.height // 2 + int(100 * math.sin(elapsed * 0.5))
        else:
            # Static target
            center_x = self.width // 2
            center_y = self.height // 2

        # Draw main target (rectangle to simulate object detection target)
        target_w = 120
        target_h = 80
        x1 = max(0, center_x - target_w // 2)
        y1 = max(0, center_y - target_h // 2)
        x2 = min(self.width, center_x + target_w // 2)
        y2 = min(self.height, center_y + target_h // 2)

        # Draw filled rectangle (simulate detected object)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 100), -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)

        # Add text label
        cv2.putText(img, 'TARGET', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Add secondary static target
        static_x = int(self.width * 0.25)
        static_y = int(self.height * 0.7)
        cv2.rectangle(img, (static_x - 30, static_y - 20), (static_x + 30, static_y + 20), (255, 100, 0), -1)
        cv2.rectangle(img, (static_x - 30, static_y - 20), (static_x + 30, static_y + 20), (255, 255, 255), 1)
        cv2.putText(img, 'OBJ', (static_x - 20, static_y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Add frame counter and timestamp
        timestamp_text = f"Frame: {self.frame_count} | {elapsed:.1f}s"
        cv2.putText(img, timestamp_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Add crosshair at center
        cv2.line(img, (self.width//2 - 20, self.height//2), (self.width//2 + 20, self.height//2), (128, 128, 128), 1)
        cv2.line(img, (self.width//2, self.height//2 - 20), (self.width//2, self.height//2 + 20), (128, 128, 128), 1)

        return img

    def publish_frame(self):
        """Publish a synthetic camera frame."""
        now = self.get_clock().now()
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.frame_id

        # Create test image
        cv_image = self.create_test_image()

        try:
            # Publish raw image
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            image_msg.header = header
            self.image_pub.publish(image_msg)

            # Publish compressed image if enabled
            if self.publish_compressed:
                # Encode as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
                result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
                if result:
                    compressed_msg = CompressedImage()
                    compressed_msg.header = header
                    compressed_msg.format = 'jpeg'
                    compressed_msg.data = encimg.tobytes()
                    self.compressed_pub.publish(compressed_msg)

            # Publish camera info
            camera_info = self.create_camera_info()
            camera_info.header = header
            self.camera_info_pub.publish(camera_info)

        except Exception as e:
            self.get_logger().error(f'Failed to publish frame: {e}')

        self.frame_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
