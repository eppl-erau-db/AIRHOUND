# AIRHOUND Flight Testing Guide

> **Version:** 1.0  
> **Date:** January 2026  
> **Platform:** Jetson Orin Nano 16GB + Intel RealSense D455

---

## Quick Start (TL;DR)

```bash
# One-command flight test
cd ~/AIRHOUND
./scripts/flight_launch.sh

# That's it. The script handles everything:
# - Pre-flight hardware checks
# - ROS2 workspace sourcing
# - Automatic rosbag recording
# - Clean shutdown on Ctrl+C
```

---

## Table of Contents

1. [Hardware Setup](#1-hardware-setup)
2. [Pre-Flight Checklist](#2-pre-flight-checklist)
3. [Running Flight Tests](#3-running-flight-tests)
4. [Post-Flight Data](#4-post-flight-data)
5. [Troubleshooting](#5-troubleshooting)
6. [Emergency Procedures](#6-emergency-procedures)
7. [Configuration Reference](#7-configuration-reference)

---

## 1. Hardware Setup

### Required Hardware
| Component | Model | Connection |
|-----------|-------|------------|
| Compute | Jetson Orin Nano 16GB | - |
| Camera | Intel RealSense D455 | USB 3.2 (use blue port) |
| Flight Controller | Pixhawk 6C | USB or UART |
| Power | 5V 4A min | Barrel jack |

### Physical Connections

```
┌─────────────────────────────────────────────────────────────┐
│                    JETSON ORIN NANO                          │
├──────────────┬──────────────┬──────────────┬────────────────┤
│  USB 3.2 (1) │  USB 3.2 (2) │  USB 2.0     │   UART/GPIO    │
│  ┌────────┐  │  (Available) │  (Available) │   ┌────────┐   │
│  │  D455  │  │              │              │   │Pixhawk │   │
│  └────────┘  │              │              │   │TELEM2  │   │
└──────────────┴──────────────┴──────────────┴────────────────┘
```

**IMPORTANT:**
- D455 **must** use USB 3.x port (check with `lsusb -t` for 5000M speed)
- Secure all cables with strain relief before flight

---

## 2. Pre-Flight Checklist

### Automated Checks
Run the automated pre-flight check:

```bash
python3 ~/AIRHOUND/scripts/flight_preflight_check.py
```

Expected output:
```
============================================================
  AIRHOUND PRE-FLIGHT CHECK
  2026-01-17 14:30:00 | jetson-orin
============================================================

  [PASS] RealSense Camera: D455 detected (USB 3.x)
  [PASS] TensorRT Engine: drone_rfdetr.engine (56.1MB)
  [PASS] ROS2 Workspace: All 3 packages installed
  [PASS] GPU Status: Orin GPU @ 45C, 12000MB free
  [WARN] PX4 Connection: No USB serial device found
  [WARN] DDS Agent: MicroXRCE-DDS agent not found
  [PASS] Disk Space: 50G free (35% used)
  [PASS] CPU Load: Load: 0.50 / 6 cores
  [PASS] Inference Test: RF-DETR inference OK

============================================================
  PASSED WITH 2 WARNING(S) - PROCEED WITH CAUTION
============================================================
```

### Manual Checks (Before Every Flight)

- [ ] Propellers secure and undamaged
- [ ] Battery fully charged (>95%)
- [ ] D455 lens clean and unobstructed
- [ ] GPS lock acquired (if using)
- [ ] Flight area clear of obstacles
- [ ] Spotter designated and briefed
- [ ] Emergency landing zone identified

---

## 3. Running Flight Tests

### Option A: One-Command Launch (Recommended)

```bash
cd ~/AIRHOUND
./scripts/flight_launch.sh
```

Options:
```bash
# Use YOLOv8 instead of RF-DETR
./scripts/flight_launch.sh --detector yolo

# Skip pre-flight checks (NOT recommended)
./scripts/flight_launch.sh --skip-preflight

# Dry run (check only, don't launch)
./scripts/flight_launch.sh --dry-run

# Simulation mode (no camera needed)
./scripts/flight_launch.sh --sim

# Enable auto-arm (DANGEROUS - controlled tests only!)
./scripts/flight_launch.sh --arm
```

### Option B: Synthetic Mode Testing (No Camera)

Use synthetic mode to test the PX4 control pipeline on real flight hardware **without a camera connected**. This is useful for:
- Validating offboard control and communication
- Testing tracking node behavior
- Ground testing before adding camera hardware

```bash
# Flight mode with synthetic detections (no camera needed)
./launch_airhound.sh flight --synthetic

# Or via ROS2 launch directly
ros2 launch launch/e2e_flight.launch.py camera_source:=synthetic
```

In synthetic mode:
- Mock detector publishes simulated `/detections` at 30 Hz
- Simulated camera_info is published for tracking node
- Tracking node uses default intrinsics if needed
- PX4 control pipeline works identically to real camera mode

### Option C: Manual Launch (Advanced)

```bash
# Terminal 1: Source workspace
cd ~/AIRHOUND/ws_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 2: Start DDS agent (if using PX4)
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Launch pipeline
ros2 launch airhound_perception full_pipeline.launch.py detector:=rfdetr

# Terminal 4: Monitor detections
ros2 topic echo /detections

# Terminal 5: Record rosbag
ros2 bag record -a -o flight_test_001
```

### Monitoring During Flight

```bash
# Watch detection rate
ros2 topic hz /detections

# Monitor yaw commands
ros2 topic echo /target_yaw

# Check PX4 setpoints
ros2 topic echo /fmu/in/trajectory_setpoint

# View camera feed (if display available)
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

---

## 4. Post-Flight Data

### Log Location
All flight data is saved to:
```
~/AIRHOUND/flight_logs/flight_YYYYMMDD_HHMMSS/
├── rosbag/                    # Compressed rosbag2 data
├── pipeline.log               # ROS2 node output
├── rosbag.log                 # Recording status
└── flight_summary.txt         # Session metadata
```

### Extracting Data

```bash
# List topics in rosbag
ros2 bag info ~/AIRHOUND/flight_logs/flight_*/rosbag

# Replay rosbag
ros2 bag play ~/AIRHOUND/flight_logs/flight_*/rosbag

# Export detections to CSV
ros2 bag record --serialization-format cdr \
    -o export ~/AIRHOUND/flight_logs/flight_*/rosbag
```

### Key Metrics to Review
1. **Detection Rate:** Should be ~25 FPS for RF-DETR
2. **Detection Latency:** Check timestamps in `/detections`
3. **Tracking Stability:** Review yaw rate commands for oscillation
4. **Depth Accuracy:** Compare depth readings vs ground truth

---

## 5. Troubleshooting

### Camera Issues

| Problem | Solution |
|---------|----------|
| "D455 not detected" | Reconnect USB, try different port |
| "USB 2.x (SLOW)" | Use blue USB 3.x port |
| "Depth frames empty" | Reset camera: `rs-enumerate-devices --compact` |
| "No image callback" | Check topic: `ros2 topic list \| grep camera` |

### Detection Issues

| Problem | Solution |
|---------|----------|
| "TensorRT engine not found" | Re-export: see RF-DETR setup guide |
| "Low FPS (<15)" | Check GPU temp, close other apps |
| "No detections" | Lower confidence: edit `perception.yaml` |
| "False positives" | Raise confidence threshold |

### PX4 Communication

| Problem | Solution |
|---------|----------|
| "No serial device" | Check USB connection, try `ls /dev/ttyACM*` |
| "DDS agent failed" | Install: `sudo apt install ros-humble-micro-ros-agent` |
| "No PX4 topics" | Verify PX4 has DDS enabled in firmware |

### Quick Fixes

```bash
# Reset RealSense camera
sudo systemctl restart nvargus-daemon
rs-enumerate-devices --compact

# Kill stuck ROS2 nodes
pkill -9 -f ros2
pkill -9 -f detector_node

# Clear GPU memory
sudo fuser -v /dev/nvidia* 2>/dev/null | xargs -r kill -9

# Rebuild workspace
cd ~/AIRHOUND/ws_ros2
rm -rf build install log
colcon build --symlink-install
```

---

## 6. Emergency Procedures

### In-Flight Emergency

1. **IMMEDIATE:** Release sticks to neutral (if RC connected)
2. **PRIORITY:** Switch to MANUAL or STABILIZE mode
3. **IF UNRESPONSIVE:** Kill switch / Emergency motor stop
4. **AFTER LANDING:** Do not approach until props stopped

### Software Emergency

```bash
# Kill all AIRHOUND processes
pkill -9 -f airhound
pkill -9 -f detector_node
pkill -9 -f ros2

# Emergency PX4 disarm (if DDS working)
ros2 topic pub --once /fmu/in/vehicle_command \
    px4_msgs/msg/VehicleCommand \
    "{command: 400, param1: 0.0, param2: 21196.0}"
```

### Data Recovery
If system crashes, rosbags may be incomplete:
```bash
# Check for partial bags
ls -la ~/AIRHOUND/flight_logs/*/rosbag/

# Attempt repair
ros2 bag reindex ~/AIRHOUND/flight_logs/flight_*/rosbag
```

---

## 7. Configuration Reference

### Detector Configuration

**File:** `ws_ros2/src/airhound_perception/config/perception_rfdetr.yaml`

```yaml
detector_node:
  ros__parameters:
    # Detector settings
    detector_type: "rfdetr"
    model_path: "/home/airhound/AIRHOUND/models/drone_rfdetr.engine"
    confidence_threshold: 0.25
    
    # Depth integration
    enable_depth: true
    depth_scale: 0.001  # D455 default: 1mm per unit
    depth_window_size: 5  # Median filter size
    
    # Performance
    max_detections: 10
    nms_threshold: 0.45
```

### Key Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `confidence_threshold` | 0.25 | 0.1-0.9 | Detection confidence cutoff |
| `depth_scale` | 0.001 | - | Depth units to meters |
| `depth_window_size` | 5 | 3-11 | Median filter kernel (odd) |
| `max_detections` | 10 | 1-100 | Max detections per frame |

### Topic Reference

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/camera/color/image_raw` | Image | 30Hz | RGB camera feed |
| `/camera/depth/image_raw` | Image | 30Hz | Depth image (uint16) |
| `/detections` | Detection2DArray | ~25Hz | Drone detections |
| `/target_yaw` | Float32 | ~25Hz | Yaw rate command |
| `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | 10Hz | PX4 commands |

---

## Support

- **Perception Issues:** Contact perception lead
- **Flight Control:** Contact flight team lead  
- **Hardware:** Check drone maintenance log

**Repository:** `https://github.com/eppl-erau-db/AIRHOUND`

---

*Last updated: January 2026 | AIRHOUND Perception Team*
