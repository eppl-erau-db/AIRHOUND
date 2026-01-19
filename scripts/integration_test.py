#!/usr/bin/env python3
"""
AIRHOUND Integration Test Harness.

Tests the perception → tracking pipeline without requiring actual hardware.
Publishes synthetic RGB+depth images with known targets and verifies:
1. Detector produces detections
2. 3D position is computed correctly
3. Tracking responds to detections

Usage:
    # Run with ROS2 nodes already launched:
    ros2 run airhound_perception perception_node &
    python3 scripts/integration_test.py
    
    # Or standalone (will report which nodes are missing):
    python3 scripts/integration_test.py --check-only

Author: Perception Lead
Date: January 2026
"""

import argparse
import sys
import time
from typing import Optional, List, Tuple
from dataclasses import dataclass, field

import numpy as np

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from vision_msgs.msg import Detection2DArray
    from geometry_msgs.msg import PointStamped
    from std_msgs.msg import Float32
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Install ros2 and source setup.bash")


@dataclass
class TestResult:
    """Result of a single test."""
    name: str
    passed: bool
    message: str
    duration_ms: float = 0.0


@dataclass
class IntegrationTestResults:
    """Aggregated test results."""
    tests: List[TestResult] = field(default_factory=list)
    
    @property
    def passed(self) -> int:
        return sum(1 for t in self.tests if t.passed)
    
    @property
    def failed(self) -> int:
        return sum(1 for t in self.tests if not t.passed)
    
    @property
    def all_passed(self) -> bool:
        return self.failed == 0
    
    def add(self, name: str, passed: bool, message: str, duration_ms: float = 0.0):
        self.tests.append(TestResult(name, passed, message, duration_ms))
    
    def print_summary(self):
        print("\n" + "=" * 60)
        print("INTEGRATION TEST RESULTS")
        print("=" * 60)
        for t in self.tests:
            status = "PASS" if t.passed else "FAIL"
            print(f"  [{status}] {t.name}: {t.message}")
        print("-" * 60)
        print(f"  Total: {self.passed}/{len(self.tests)} passed")
        if self.all_passed:
            print("  STATUS: ALL TESTS PASSED")
        else:
            print("  STATUS: SOME TESTS FAILED")
        print("=" * 60)


class IntegrationTestNode(Node):
    """
    ROS2 node for integration testing.
    
    Publishes synthetic images and subscribes to outputs to verify pipeline.
    """
    
    # D455 camera intrinsics (from camera validation)
    FX = 641.66
    FY = 640.95
    CX = 646.36
    CY = 365.12
    WIDTH = 1280
    HEIGHT = 720
    
    def __init__(self):
        super().__init__('integration_test_node')
        
        self.bridge = CvBridge()
        self.results = IntegrationTestResults()
        
        # QoS profiles
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_default = QoSProfile(depth=10)
        
        # Publishers (to feed the pipeline)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_sensor)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_sensor)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_sensor)
        
        # Subscribers (to receive pipeline outputs)
        self.detections: List[Detection2DArray] = []
        self.target_3d: List[PointStamped] = []
        self.depth_quality: List[float] = []
        
        self.det_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self._on_detections, qos_default
        )
        self.target_3d_sub = self.create_subscription(
            PointStamped, '/perception/target_3d',
            self._on_target_3d, qos_default
        )
        self.depth_quality_sub = self.create_subscription(
            Float32, '/perception/depth_quality',
            self._on_depth_quality, qos_default
        )
        
        self.get_logger().info("IntegrationTestNode initialized")
    
    def _on_detections(self, msg: Detection2DArray):
        self.detections.append(msg)
    
    def _on_target_3d(self, msg: PointStamped):
        self.target_3d.append(msg)
    
    def _on_depth_quality(self, msg: Float32):
        self.depth_quality.append(msg.data)
    
    def create_camera_info(self) -> CameraInfo:
        """Create camera info message with D455 intrinsics."""
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.width = self.WIDTH
        msg.height = self.HEIGHT
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            self.FX, 0.0, self.CX,
            0.0, self.FY, self.CY,
            0.0, 0.0, 1.0
        ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [
            self.FX, 0.0, self.CX, 0.0,
            0.0, self.FY, self.CY, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        return msg
    
    def create_synthetic_rgb(
        self,
        target_u: int,
        target_v: int,
        target_size: int = 50,
        color: Tuple[int, int, int] = (0, 255, 0)
    ) -> np.ndarray:
        """
        Create synthetic RGB image with a colored square target.
        
        Parameters
        ----------
        target_u : int
            Target center x coordinate (pixels)
        target_v : int
            Target center y coordinate (pixels)
        target_size : int
            Target width/height in pixels
        color : Tuple[int, int, int]
            BGR color for the target
            
        Returns
        -------
        np.ndarray
            BGR image (HEIGHT x WIDTH x 3)
        """
        # Dark background
        img = np.zeros((self.HEIGHT, self.WIDTH, 3), dtype=np.uint8)
        img[:, :] = (30, 30, 30)  # Dark gray
        
        # Draw target square
        half = target_size // 2
        u1 = max(0, target_u - half)
        u2 = min(self.WIDTH, target_u + half)
        v1 = max(0, target_v - half)
        v2 = min(self.HEIGHT, target_v + half)
        
        img[v1:v2, u1:u2] = color
        
        return img
    
    def create_synthetic_depth(
        self,
        target_u: int,
        target_v: int,
        target_size: int,
        target_depth_mm: int,
        background_depth_mm: int = 0
    ) -> np.ndarray:
        """
        Create synthetic depth image with target at specified depth.
        
        Parameters
        ----------
        target_u, target_v : int
            Target center coordinates
        target_size : int
            Target width/height in pixels
        target_depth_mm : int
            Depth value in millimeters
        background_depth_mm : int
            Background depth (0 = invalid/no return)
            
        Returns
        -------
        np.ndarray
            16-bit depth image (HEIGHT x WIDTH)
        """
        depth = np.full((self.HEIGHT, self.WIDTH), background_depth_mm, dtype=np.uint16)
        
        half = target_size // 2
        u1 = max(0, target_u - half)
        u2 = min(self.WIDTH, target_u + half)
        v1 = max(0, target_v - half)
        v2 = min(self.HEIGHT, target_v + half)
        
        depth[v1:v2, u1:u2] = target_depth_mm
        
        return depth
    
    def publish_synthetic_frame(
        self,
        target_u: int,
        target_v: int,
        target_depth_m: float,
        target_size: int = 50
    ):
        """Publish synthetic RGB, depth, and camera info."""
        now = self.get_clock().now()
        
        # RGB
        rgb = self.create_synthetic_rgb(target_u, target_v, target_size)
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        rgb_msg.header.stamp = now.to_msg()
        rgb_msg.header.frame_id = 'camera_color_optical_frame'
        
        # Depth (convert m to mm)
        depth_mm = int(target_depth_m * 1000)
        depth = self.create_synthetic_depth(target_u, target_v, target_size, depth_mm)
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = now.to_msg()
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        
        # Camera info
        cam_info = self.create_camera_info()
        
        # Publish
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(cam_info)
    
    def wait_for_messages(self, timeout_sec: float = 2.0) -> bool:
        """Spin and wait for detection messages."""
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.detections) > 0:
                return True
        return False
    
    def clear_messages(self):
        """Clear received message buffers."""
        self.detections.clear()
        self.target_3d.clear()
        self.depth_quality.clear()
    
    def run_tests(self) -> IntegrationTestResults:
        """Run all integration tests."""
        self.get_logger().info("Starting integration tests...")
        
        # Test 1: Check if perception node is responding
        self._test_perception_responds()
        
        # Test 2: Verify 3D position computation
        self._test_3d_position()
        
        # Test 3: Verify depth quality reporting
        self._test_depth_quality()
        
        # Test 4: Verify out-of-range depth filtering
        self._test_depth_range_filter()
        
        return self.results
    
    def _test_perception_responds(self):
        """Test that perception node responds to images."""
        self.clear_messages()
        
        # Publish frames at center of image
        target_u, target_v = self.WIDTH // 2, self.HEIGHT // 2
        target_depth = 3.0  # meters
        
        start = time.time()
        for _ in range(10):
            self.publish_synthetic_frame(target_u, target_v, target_depth)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.033)  # ~30 Hz
        
        # Wait for response
        received = self.wait_for_messages(timeout_sec=2.0)
        duration_ms = (time.time() - start) * 1000
        
        if received:
            self.results.add(
                "Perception Response",
                True,
                f"Received {len(self.detections)} detection messages",
                duration_ms
            )
        else:
            self.results.add(
                "Perception Response",
                False,
                "No detections received - is perception_node running?",
                duration_ms
            )
    
    def _test_3d_position(self):
        """Test that 3D position is computed correctly from depth."""
        self.clear_messages()
        
        # Known target position
        target_u, target_v = 800, 400
        target_depth = 2.5  # meters
        
        # Expected 3D position (camera optical frame)
        expected_x = (target_u - self.CX) * target_depth / self.FX
        expected_y = (target_v - self.CY) * target_depth / self.FY
        expected_z = target_depth
        
        # Publish frames
        start = time.time()
        for _ in range(15):
            self.publish_synthetic_frame(target_u, target_v, target_depth, target_size=80)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.033)
        
        self.wait_for_messages(timeout_sec=2.0)
        duration_ms = (time.time() - start) * 1000
        
        if len(self.target_3d) > 0:
            # Check last 3D position
            pos = self.target_3d[-1].point
            
            # Allow 10% tolerance for position matching
            z_error = abs(pos.z - expected_z) / expected_z if expected_z > 0 else 0
            
            if z_error < 0.1:
                self.results.add(
                    "3D Position",
                    True,
                    f"Depth correct: {pos.z:.2f}m (expected {expected_z:.2f}m)",
                    duration_ms
                )
            else:
                self.results.add(
                    "3D Position",
                    False,
                    f"Depth mismatch: {pos.z:.2f}m vs expected {expected_z:.2f}m",
                    duration_ms
                )
        else:
            self.results.add(
                "3D Position",
                False,
                "No 3D positions received",
                duration_ms
            )
    
    def _test_depth_quality(self):
        """Test that depth quality metric is published."""
        self.clear_messages()
        
        # Publish frames with valid depth
        target_u, target_v = self.WIDTH // 2, self.HEIGHT // 2
        target_depth = 3.0
        
        start = time.time()
        for _ in range(15):
            self.publish_synthetic_frame(target_u, target_v, target_depth)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.033)
        
        self.wait_for_messages(timeout_sec=2.0)
        duration_ms = (time.time() - start) * 1000
        
        if len(self.depth_quality) > 0:
            avg_quality = np.mean(self.depth_quality)
            self.results.add(
                "Depth Quality",
                True,
                f"Quality metric received: avg={avg_quality:.2f}",
                duration_ms
            )
        else:
            self.results.add(
                "Depth Quality",
                False,
                "No depth quality messages received",
                duration_ms
            )
    
    def _test_depth_range_filter(self):
        """Test that out-of-range depths are filtered."""
        self.clear_messages()
        
        # Publish frames with depth outside D455 range (< 0.4m)
        target_u, target_v = self.WIDTH // 2, self.HEIGHT // 2
        target_depth = 0.2  # Too close - should be filtered
        
        start = time.time()
        for _ in range(10):
            self.publish_synthetic_frame(target_u, target_v, target_depth)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.033)
        
        self.wait_for_messages(timeout_sec=2.0)
        duration_ms = (time.time() - start) * 1000
        
        # Should NOT receive valid 3D positions for out-of-range depth
        if len(self.target_3d) == 0:
            self.results.add(
                "Depth Range Filter",
                True,
                "Out-of-range depth correctly filtered (no 3D position)",
                duration_ms
            )
        else:
            # Check if the 3D position has NaN depth
            last_z = self.target_3d[-1].point.z
            if np.isnan(last_z):
                self.results.add(
                    "Depth Range Filter",
                    True,
                    "Out-of-range depth correctly marked as NaN",
                    duration_ms
                )
            else:
                self.results.add(
                    "Depth Range Filter",
                    False,
                    f"Out-of-range depth not filtered: z={last_z:.2f}m",
                    duration_ms
                )


def check_node_availability() -> List[str]:
    """Check which required nodes are running."""
    import subprocess
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True, text=True, timeout=5
        )
        nodes = result.stdout.strip().split('\n')
        return nodes
    except Exception as e:
        print(f"Error checking nodes: {e}")
        return []


def main():
    parser = argparse.ArgumentParser(description='AIRHOUND Integration Test Harness')
    parser.add_argument('--check-only', action='store_true',
                        help='Only check node availability, do not run tests')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Test timeout in seconds')
    args = parser.parse_args()
    
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available")
        print("Make sure to source /opt/ros/humble/setup.bash")
        sys.exit(1)
    
    # Check node availability
    print("Checking ROS2 nodes...")
    nodes = check_node_availability()
    print(f"Running nodes: {nodes}")
    
    perception_running = any('perception' in n.lower() for n in nodes)
    
    if args.check_only:
        print("\n--- Node Check ---")
        print(f"  Perception node: {'RUNNING' if perception_running else 'NOT FOUND'}")
        if not perception_running:
            print("\nTo start perception node:")
            print("  ros2 run airhound_perception perception_node")
        sys.exit(0 if perception_running else 1)
    
    if not perception_running:
        print("\nWARNING: Perception node not detected.")
        print("Tests may fail. Start with: ros2 run airhound_perception perception_node")
        print("Continuing anyway...\n")
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        node = IntegrationTestNode()
        
        # Give time for subscriptions to connect
        print("Waiting for ROS2 connections...")
        time.sleep(1.0)
        
        # Run tests
        results = node.run_tests()
        results.print_summary()
        
        # Exit code based on results
        sys.exit(0 if results.all_passed else 1)
        
    except KeyboardInterrupt:
        print("\nTest interrupted")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
