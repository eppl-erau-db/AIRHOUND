#!/usr/bin/env python3
"""
V4L2 Camera Bridge for RealSense D455 on Jetson.

Workaround for librealsense2 v2.56+ "bad optional access" bug on ARM64 + USB 2.0.
Opens /dev/video2 (D455 RGB stream) via OpenCV V4L2 and publishes to ROS2 topics
that the detector node expects.

Usage:
    python3 v4l2_camera_bridge.py
    # or with ROS args:
    ros2 run airhound_perception v4l2_camera_bridge --ros-args -p device:=/dev/video2
"""

import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class V4L2CameraBridge(Node):
    def __init__(self):
        super().__init__('v4l2_camera_bridge')

        self.declare_parameter('device', '/dev/video2')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()

        # Open camera
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            try:
                self.cap = cv2.VideoCapture(int(device), cv2.CAP_V4L2)
            except ValueError:
                pass
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera: {device}')
            sys.exit(1)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Camera opened: {device} @ {actual_w}x{actual_h} {actual_fps:.1f}fps')

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Build static camera info (D455 approximate intrinsics for 640x480)
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = actual_w
        self.camera_info.height = actual_h
        self.camera_info.distortion_model = 'plumb_bob'
        fx = 383.0
        fy = 383.0
        cx = actual_w / 2.0
        cy = actual_h / 2.0
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # Timer for capture
        period = 1.0 / fps
        self.timer = self.create_timer(period, self.capture_callback)
        self.frame_count = 0

    def capture_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        self.camera_info.header.stamp = now
        self.info_pub.publish(self.camera_info)

        self.frame_count += 1
        if self.frame_count % 150 == 0:
            self.get_logger().info(f'Frame {self.frame_count}')

    def destroy_node(self):
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
