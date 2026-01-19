#!/usr/bin/env python3
"""
RealSense D455 Camera Validation Script for AIRHOUND.

Validates that the D455 camera is producing valid RGB and depth data
suitable for the perception pipeline.

Usage:
    # With ROS2 already running camera node:
    python3 validate_camera.py
    
    # Standalone (launches camera node):
    python3 validate_camera.py --launch-camera

Author: Perception Lead
Date: January 2026
"""

import argparse
import sys
import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np


@dataclass
class ValidationResult:
    """Results from camera validation."""
    rgb_received: bool = False
    depth_received: bool = False
    camera_info_received: bool = False
    target_3d_received: bool = False
    
    rgb_fps: float = 0.0
    depth_fps: float = 0.0
    rgb_resolution: Tuple[int, int] = (0, 0)
    depth_resolution: Tuple[int, int] = (0, 0)
    
    depth_min: float = float('inf')
    depth_max: float = 0.0
    depth_valid_percent: float = 0.0
    
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    
    errors: list = None
    warnings: list = None
    
    def __post_init__(self):
        if self.errors is None:
            self.errors = []
        if self.warnings is None:
            self.warnings = []
    
    @property
    def passed(self) -> bool:
        return (self.rgb_received and self.depth_received and 
                self.camera_info_received and len(self.errors) == 0)


class CameraValidator(Node):
    """ROS2 node for validating D455 camera output."""
    
    def __init__(self, timeout_sec: float = 10.0):
        super().__init__('camera_validator')
        self.timeout_sec = timeout_sec
        self.bridge = CvBridge()
        self.result = ValidationResult()
        
        # Counters for FPS calculation
        self.rgb_count = 0
        self.depth_count = 0
        self.start_time = None
        
        # Last messages
        self.last_depth_image: Optional[np.ndarray] = None
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers - try multiple topic naming conventions
        # Standard naming
        self.create_subscription(
            Image, '/camera/color/image_raw', 
            self.rgb_callback, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            self.depth_callback_alt, sensor_qos
        )
        # Nested namespace naming (default realsense2_camera)
        self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.rgb_callback, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, sensor_qos
        )
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self.depth_callback_alt, sensor_qos
        )
        self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self.camera_info_callback, 10
        )
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )
        self.create_subscription(
            PointStamped, '/perception/target_3d',
            self.target_3d_callback, 10
        )
        
        self.get_logger().info("Camera validator initialized. Waiting for data...")
    
    def rgb_callback(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
        
        self.result.rgb_received = True
        self.rgb_count += 1
        self.result.rgb_resolution = (msg.width, msg.height)
        
        # Calculate FPS
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.result.rgb_fps = self.rgb_count / elapsed
    
    def depth_callback(self, msg: Image):
        self._process_depth(msg)
    
    def depth_callback_alt(self, msg: Image):
        # Fallback for non-aligned depth
        if not self.result.depth_received:
            self._process_depth(msg)
    
    def _process_depth(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
        
        self.result.depth_received = True
        self.depth_count += 1
        self.result.depth_resolution = (msg.width, msg.height)
        
        # Calculate FPS
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            self.result.depth_fps = self.depth_count / elapsed
        
        # Analyze depth values
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.last_depth_image = depth_image
            
            # D455 depth is in mm (16UC1), convert to meters
            if msg.encoding == '16UC1':
                depth_m = depth_image.astype(np.float32) * 0.001
            else:
                depth_m = depth_image.astype(np.float32)
            
            # Valid depth range for D455: 0.4m - 6.0m
            valid_mask = (depth_m > 0.1) & (depth_m < 10.0)
            valid_depths = depth_m[valid_mask]
            
            if len(valid_depths) > 0:
                self.result.depth_min = min(self.result.depth_min, float(np.min(valid_depths)))
                self.result.depth_max = max(self.result.depth_max, float(np.max(valid_depths)))
                self.result.depth_valid_percent = 100.0 * len(valid_depths) / depth_m.size
        except Exception as e:
            self.result.errors.append(f"Depth processing error: {e}")
    
    def camera_info_callback(self, msg: CameraInfo):
        self.result.camera_info_received = True
        self.result.fx = msg.k[0]
        self.result.fy = msg.k[4]
        self.result.cx = msg.k[2]
        self.result.cy = msg.k[5]
    
    def target_3d_callback(self, msg: PointStamped):
        self.result.target_3d_received = True
    
    def validate(self) -> ValidationResult:
        """Run validation for timeout_sec seconds and return results."""
        end_time = time.time() + self.timeout_sec
        
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Early exit if we have everything
            if (self.result.rgb_received and self.result.depth_received and 
                self.result.camera_info_received and self.rgb_count >= 10):
                break
        
        # Post-validation checks
        self._run_checks()
        
        return self.result
    
    def _run_checks(self):
        """Run validation checks and populate errors/warnings."""
        # Critical errors
        if not self.result.rgb_received:
            self.result.errors.append("No RGB images received from /camera/color/image_raw")
        
        if not self.result.depth_received:
            self.result.errors.append("No depth images received from /camera/depth/image_raw")
        
        if not self.result.camera_info_received:
            self.result.errors.append("No camera info received from /camera/color/camera_info")
        
        # Warnings
        if self.result.rgb_fps < 25.0 and self.result.rgb_received:
            self.result.warnings.append(f"Low RGB FPS: {self.result.rgb_fps:.1f} (expected ~30)")
        
        if self.result.depth_fps < 25.0 and self.result.depth_received:
            self.result.warnings.append(f"Low depth FPS: {self.result.depth_fps:.1f} (expected ~30)")
        
        if self.result.depth_valid_percent < 50.0 and self.result.depth_received:
            self.result.warnings.append(
                f"Low valid depth coverage: {self.result.depth_valid_percent:.1f}% "
                "(check for occlusions or lighting)"
            )
        
        if self.result.fx == 0 and self.result.camera_info_received:
            self.result.errors.append("Camera intrinsics not populated (fx=0)")
        
        # Resolution checks
        if self.result.rgb_resolution != (0, 0):
            w, h = self.result.rgb_resolution
            if w < 640 or h < 480:
                self.result.warnings.append(f"Low RGB resolution: {w}x{h}")
        
        if self.result.depth_resolution != (0, 0):
            w, h = self.result.depth_resolution
            if w < 640 or h < 480:
                self.result.warnings.append(f"Low depth resolution: {w}x{h}")


def print_results(result: ValidationResult):
    """Print validation results in a formatted way."""
    print("\n" + "=" * 60)
    print("AIRHOUND D455 Camera Validation Results")
    print("=" * 60)
    
    # Overall status
    status = "PASSED" if result.passed else "FAILED"
    status_color = "\033[92m" if result.passed else "\033[91m"
    print(f"\nOverall Status: {status_color}{status}\033[0m")
    
    # Stream status
    print("\n--- Stream Status ---")
    print(f"  RGB Stream:     {'OK' if result.rgb_received else 'MISSING'}")
    print(f"  Depth Stream:   {'OK' if result.depth_received else 'MISSING'}")
    print(f"  Camera Info:    {'OK' if result.camera_info_received else 'MISSING'}")
    print(f"  Target 3D:      {'OK' if result.target_3d_received else 'N/A (perception node not running)'}")
    
    # Performance
    if result.rgb_received or result.depth_received:
        print("\n--- Performance ---")
        if result.rgb_received:
            print(f"  RGB FPS:        {result.rgb_fps:.1f}")
            print(f"  RGB Resolution: {result.rgb_resolution[0]}x{result.rgb_resolution[1]}")
        if result.depth_received:
            print(f"  Depth FPS:      {result.depth_fps:.1f}")
            print(f"  Depth Resolution: {result.depth_resolution[0]}x{result.depth_resolution[1]}")
    
    # Depth analysis
    if result.depth_received and result.depth_min < float('inf'):
        print("\n--- Depth Analysis ---")
        print(f"  Min Depth:      {result.depth_min:.2f}m")
        print(f"  Max Depth:      {result.depth_max:.2f}m")
        print(f"  Valid Coverage: {result.depth_valid_percent:.1f}%")
        
        # D455 range check
        if result.depth_min < 0.4:
            print(f"  Note: Some depths below D455 min range (0.4m)")
        if result.depth_max > 6.0:
            print(f"  Note: Some depths above D455 optimal range (6.0m)")
    
    # Camera intrinsics
    if result.camera_info_received and result.fx > 0:
        print("\n--- Camera Intrinsics ---")
        print(f"  fx: {result.fx:.2f}")
        print(f"  fy: {result.fy:.2f}")
        print(f"  cx: {result.cx:.2f}")
        print(f"  cy: {result.cy:.2f}")
    
    # Errors
    if result.errors:
        print("\n--- ERRORS ---")
        for err in result.errors:
            print(f"  \033[91m[ERROR]\033[0m {err}")
    
    # Warnings
    if result.warnings:
        print("\n--- Warnings ---")
        for warn in result.warnings:
            print(f"  \033[93m[WARN]\033[0m {warn}")
    
    print("\n" + "=" * 60)
    
    # Next steps
    if result.passed:
        print("\nCamera validation PASSED. Ready for perception pipeline testing.")
        print("Next: Launch perception node with 'ros2 launch airhound_perception detector.launch.py'")
    else:
        print("\nCamera validation FAILED. Please check:")
        print("  1. D455 is connected: lsusb | grep -i intel")
        print("  2. Camera node is running: ros2 node list | grep realsense")
        print("  3. Topics are publishing: ros2 topic list | grep camera")
    
    print()


def main():
    parser = argparse.ArgumentParser(description='Validate RealSense D455 camera for AIRHOUND')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='Validation timeout in seconds (default: 10)')
    parser.add_argument('--launch-camera', action='store_true',
                        help='Launch RealSense camera node before validation')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        if args.launch_camera:
            print("Note: --launch-camera not implemented. Please launch camera manually:")
            print("  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true")
            print()
        
        validator = CameraValidator(timeout_sec=args.timeout)
        result = validator.validate()
        print_results(result)
        
        # Exit code for CI
        sys.exit(0 if result.passed else 1)
        
    except KeyboardInterrupt:
        print("\nValidation interrupted.")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
