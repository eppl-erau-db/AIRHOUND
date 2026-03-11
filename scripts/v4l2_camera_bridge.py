#!/usr/bin/env python3
"""
V4L2 Camera Bridge for RealSense D455 on Jetson.

Workaround for librealsense2 v2.56+ "bad optional access" bug on ARM64.
Opens /dev/video2 (D455 RGB stream) via OpenCV V4L2 and publishes to ROS2 topics
that the detector node expects.

Threaded capture to avoid blocking the ROS2 executor.
"""

import sys
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class V4L2CameraBridge(Node):
    def __init__(self):
        super().__init__('v4l2_camera_bridge')

        from rcl_interfaces.msg import ParameterDescriptor
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('width', 640, dyn)
        self.declare_parameter('height', 480, dyn)
        self.declare_parameter('fps', 15.0, dyn)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        device = self.get_parameter('device').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        # Open camera
        dev = device
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            try:
                self.cap = cv2.VideoCapture(int(dev), cv2.CAP_V4L2)
            except ValueError:
                pass
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {dev}')
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        # Minimize V4L2 internal buffer to reduce latency
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera opened: {dev} @ {actual_w}x{actual_h} {actual_fps:.1f}fps')

        # Publishers (keep depth=1 to drop stale frames)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)

        # Build static camera info (D455 approximate intrinsics)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = actual_w
        self.camera_info.height = actual_h
        self.camera_info.distortion_model = 'plumb_bob'
        # Scale intrinsics proportional to resolution
        fx = 383.0 * (actual_w / 640.0)
        fy = 383.0 * (actual_h / 480.0)
        cx = actual_w / 2.0
        cy = actual_h / 2.0
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Pre-allocate Image message template
        self._img_msg = Image()
        self._img_msg.header.frame_id = self.frame_id
        self._img_msg.height = actual_h
        self._img_msg.width = actual_w
        self._img_msg.encoding = 'bgr8'
        self._img_msg.is_bigendian = 0
        self._img_msg.step = actual_w * 3

        # Threaded capture loop
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        self.frame_count = 0

    def _capture_loop(self):
        """Dedicated capture thread — reads frames as fast as the camera delivers."""
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                continue

            now = self.get_clock().now().to_msg()

            # Build Image message directly (avoid cv_bridge copy overhead)
            msg = self._img_msg
            msg.header.stamp = now
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)

            self.camera_info.header.stamp = now
            self.info_pub.publish(self.camera_info)

            self.frame_count += 1
            if self.frame_count % 300 == 0:
                self.get_logger().info(f'Frame {self.frame_count}')

    def destroy_node(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = V4L2CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
