#!/usr/bin/env python3
"""
Mock detector node for SITL testing
Publishes fake detections without needing a camera or YOLO
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import math

class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')
        
        self.declare_parameter('detection_rate', 10.0)
        self.declare_parameter('mock_position_x', 320.0)
        self.declare_parameter('mock_position_y', 240.0)
        
        rate = self.get_parameter('detection_rate').value
        self.mock_x = self.get_parameter('mock_position_x').value
        self.mock_y = self.get_parameter('mock_position_y').value
        
        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Timer
        self.timer = self.create_timer(1.0 / rate, self.publish_mock_detection)
        
        self.get_logger().info(f'Mock detector started - publishing at {rate} Hz')
        self.get_logger().info(f'Mock position: ({self.mock_x}, {self.mock_y})')
        
        self.counter = 0
        
    def publish_mock_detection(self):
        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header = Header()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = 'camera'
        camera_info.width = 640
        camera_info.height = 480
        self.camera_info_pub.publish(camera_info)
        
        # Create mock detection
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera'
        
        detection = Detection2D()
        detection.header = detection_array.header
        
        # Simulate moving target (circular motion)
        t = self.counter * 0.1
        x_offset = 50 * math.sin(t)
        y_offset = 50 * math.cos(t)
        
        detection.bbox.center.position.x = self.mock_x + x_offset
        detection.bbox.center.position.y = self.mock_y + y_offset
        detection.bbox.size_x = 50.0
        detection.bbox.size_y = 50.0
        
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = '0'
        hypothesis.hypothesis.score = 0.95
        detection.results.append(hypothesis)
        
        detection_array.detections.append(detection)
        
        self.detection_pub.publish(detection_array)
        
        if self.counter % 10 == 0:
            self.get_logger().info(f'Published mock detection at ({detection.bbox.center.position.x:.1f}, {detection.bbox.center.position.y:.1f})')
        
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = MockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
