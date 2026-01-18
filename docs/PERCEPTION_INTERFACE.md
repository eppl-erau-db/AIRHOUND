# Perception Interface Documentation

## Overview

The perception node (`detector_node.py`) provides object detection with depth integration for the AIRHOUND tracking pipeline. This document describes the published topics and message formats for downstream consumers (tracking, PINN, control).

---

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/detections` | `vision_msgs/Detection2DArray` | ~25-30 Hz | 2D bounding boxes with embedded depth |
| `/perception/target_3d` | `geometry_msgs/PointStamped` | ~25-30 Hz | **3D position of highest-confidence detection** |
| `/perception/latency_ms` | `std_msgs/Float32` | ~25-30 Hz | End-to-end detection latency |
| `/perception/fps` | `std_msgs/Float32` | 1 Hz | Current inference FPS |
| `/perception/depth_available` | `std_msgs/Float32` | ~25-30 Hz | 1.0 if depth is available, 0.0 otherwise |

---

## Message Formats

### `/detections` - Detection2DArray

Standard `vision_msgs/Detection2DArray` with depth embedded in the pose fields.

```
vision_msgs/Detection2DArray
├── header
│   ├── stamp              # Timestamp
│   └── frame_id           # "camera_color_optical_frame"
└── detections[]           # Array of Detection2D
    ├── bbox
    │   ├── center.position.x   # Pixel U coordinate (horizontal)
    │   ├── center.position.y   # Pixel V coordinate (vertical)
    │   ├── size_x              # Bbox width in pixels
    │   └── size_y              # Bbox height in pixels
    └── results[]
        └── [0]
            ├── hypothesis
            │   ├── class_id    # "drone" (or class label)
            │   └── score       # Confidence [0.0, 1.0]
            └── pose.pose.position
                ├── x           # Pixel U (center of bbox)
                ├── y           # Pixel V (center of bbox)
                └── z           # **DEPTH in meters** (from D455)
```

#### Depth Field Details

- **Location:** `detection.results[0].pose.pose.position.z`
- **Units:** Meters
- **Source:** Median-filtered depth from RealSense D455 at bbox center
- **Valid range:** 0.4m - 6.0m (D455 optimal range)
- **Invalid value:** `NaN` if depth unavailable or out of range

#### Example: Extracting Depth from Detection

```python
def process_detection(msg: Detection2DArray):
    for det in msg.detections:
        # 2D bbox
        u = det.bbox.center.position.x  # pixels
        v = det.bbox.center.position.y  # pixels
        w = det.bbox.size_x
        h = det.bbox.size_y
        
        # Depth (meters)
        depth_m = det.results[0].pose.pose.position.z
        
        if not math.isnan(depth_m):
            print(f"Target at pixel ({u}, {v}), depth={depth_m:.2f}m")
```

---

### `/perception/target_3d` - PointStamped

**NEW:** Direct 3D position of the highest-confidence detection in camera optical frame.

```
geometry_msgs/PointStamped
├── header
│   ├── stamp              # Timestamp
│   └── frame_id           # "camera_color_optical_frame"
└── point
    ├── x                   # X position in meters (right)
    ├── y                   # Y position in meters (down)
    └── z                   # Z position in meters (forward/depth)
```

#### Coordinate System

Uses **camera optical frame** (standard ROS convention):
- **+X:** Right
- **+Y:** Down  
- **+Z:** Forward (into scene)

#### 3D Projection Formula

The 3D position is computed from 2D pixel + depth using camera intrinsics:

```python
# Camera intrinsics from /camera/camera_info
fx = K[0]  # Focal length X
fy = K[4]  # Focal length Y
cx = K[2]  # Principal point X
cy = K[5]  # Principal point Y

# 2D pixel (u, v) + depth z
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = z  # depth in meters
```

#### Usage for Tracking/PINN

```python
from geometry_msgs.msg import PointStamped

def target_callback(msg: PointStamped):
    # Direct 3D position - no extra computation needed
    x = msg.point.x  # meters, right
    y = msg.point.y  # meters, down
    z = msg.point.z  # meters, forward (depth)
    
    # Feed directly to Kalman filter
    kalman.update([x, y, z])
```

---

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RGB image from D455 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Depth image from D455 (16UC1, mm) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

---

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detector_type` | string | `"yolo"` | Detector backend: `"yolo"` or `"rfdetr"` |
| `model_path` | string | `"yolov8Detector.pt"` | Path to model file |
| `conf` | float | `0.25` | Confidence threshold |
| `enable_depth` | bool | `true` | Enable depth integration |
| `depth_scale` | float | `0.001` | Depth scale factor (D455: mm→m) |
| `depth_window_size` | int | `5` | Window size for median depth filtering |

---

## Frame IDs

| Frame | Description |
|-------|-------------|
| `camera_color_optical_frame` | RGB camera optical frame (Z forward) |
| `camera_depth_optical_frame` | Depth camera optical frame |
| `camera_link` | Physical camera body frame |

The D455 hardware-synchronizes RGB and depth, so `camera_color_optical_frame` is used for all detections.

---

## Example: Full Pipeline Integration

```python
#!/usr/bin/env python3
"""Example subscriber for tracking team."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray
import math


class TrackingSubscriber(Node):
    def __init__(self):
        super().__init__('tracking_subscriber')
        
        # Option 1: Use pre-computed 3D position (recommended)
        self.create_subscription(
            PointStamped,
            '/perception/target_3d',
            self.on_target_3d,
            10
        )
        
        # Option 2: Use raw detections with depth
        self.create_subscription(
            Detection2DArray,
            '/detections',
            self.on_detections,
            10
        )
    
    def on_target_3d(self, msg: PointStamped):
        """Direct 3D position - simplest for Kalman/PINN."""
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        
        if not math.isnan(z):
            self.get_logger().info(f"Target 3D: ({x:.2f}, {y:.2f}, {z:.2f})m")
            # Feed to Kalman filter: state = [x, y, z, vx, vy, vz]
    
    def on_detections(self, msg: Detection2DArray):
        """Raw detections with embedded depth."""
        for det in msg.detections:
            score = det.results[0].hypothesis.score
            depth = det.results[0].pose.pose.position.z
            
            if not math.isnan(depth):
                self.get_logger().info(f"Detection: conf={score:.2f}, depth={depth:.2f}m")


def main():
    rclpy.init()
    node = TrackingSubscriber()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
```

---

## Latency Considerations

| Stage | Typical Latency |
|-------|-----------------|
| D455 capture | ~33ms (30fps) |
| RF-DETR inference | ~35-40ms |
| YOLOv8 inference | ~25-30ms |
| Depth extraction | <1ms |
| **Total pipeline** | **~60-75ms** |

For PINN prediction during dropouts, account for this latency in the `dt` prediction horizon.

---

## Troubleshooting

### No depth values (all NaN)
1. Check depth stream: `ros2 topic hz /camera/depth/image_raw`
2. Verify D455 is in correct mode: depth should be 16UC1
3. Check target is within D455 range (0.4m - 6.0m)

### Noisy depth values
- Increase `depth_window_size` parameter (default: 5)
- Ensure adequate lighting (D455 active IR helps)

### Missing `/perception/target_3d`
- Ensure `enable_depth: true` in config
- Check camera intrinsics are being received: `ros2 topic echo /camera/camera_info --once`

---

*Last updated: January 2026 - Perception Lead*
