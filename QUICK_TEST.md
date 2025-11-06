# Quick Test Guide - Mock Detector for Simulation

**Last Updated**: October 27, 2025  
**Purpose**: Verify mock detector works for SITL testing

---

## ✅ Prerequisites

- Ubuntu 22.04
- ROS 2 Humble installed
- Python 3.10+
- NumPy installed: `pip3 install numpy`

---

## 🚀 Quick Test (5 steps)

### Step 1: Get the code
```bash
cd ~/Documents/school/EPPL/simulation_workspace/AIRHOUND
git checkout feature/role1-perception-integration
```

### Step 2: Build the package
```bash
cd ws_ros2
colcon build --packages-select airhound_perception
```

### Step 3: Source the workspace
```bash
source install/setup.bash
```

### Step 4: Launch mock detector
```bash
ros2 launch airhound_perception sim_test.launch.py
```

**Expected output**:
```
[mock_detector_node]: Mock Detector started: 1280x720 @ 30.0 Hz
[mock_detector_node]: Publishing 1 target(s) on /detections
[mock_detector_node]: Motion: circular, Moving: True
```

### Step 5: Verify detections (in another terminal)
```bash
# Terminal 2
cd ~/Documents/school/EPPL/simulation_workspace/AIRHOUND/ws_ros2
source install/setup.bash
ros2 topic echo /detections
```

**Expected output**:
```yaml
header:
  stamp:
    sec: 1730000000
    nanosec: 123456789
  frame_id: camera_color_optical_frame
detections:
- bbox:
    center:
      x: 640.5
      y: 360.2
      theta: 0.0
    size_x: 120.0
    size_y: 80.0
  results:
  - hypothesis:
      class_id: drone
      score: 0.85
```

---

## ✅ Success Criteria

- [x] Package builds without errors
- [x] Mock detector launches without errors
- [x] `/detections` topic publishes at ~30 Hz
- [x] Detections contain valid bounding boxes
- [x] Target moves (if moving_target:=true)

---

## 🎮 Test Different Modes

### Static target (centered)
```bash
ros2 launch airhound_perception sim_test.launch.py moving_target:=false
```

### Sinusoidal motion (left-right)
```bash
ros2 launch airhound_perception sim_test.launch.py motion_type:=sinusoidal
```

### Multiple targets
```bash
ros2 launch airhound_perception sim_test.launch.py num_targets:=3
```

### With noise
```bash
ros2 launch airhound_perception sim_test.launch.py add_noise:=true
```

---

## 🐛 Troubleshooting

### Error: "Package not found"
```bash
# Make sure you sourced the workspace
source install/setup.bash

# Check package is installed
ros2 pkg list | grep airhound
```

### Error: "No module named 'numpy'"
```bash
pip3 install numpy
```

### Error: "executable 'mock_detector' not found"
```bash
# Rebuild the package
colcon build --packages-select airhound_perception --symlink-install
source install/setup.bash
```

### No output on /detections topic
```bash
# Check if node is running
ros2 node list

# Check if topic exists
ros2 topic list | grep detections

# Check topic info
ros2 topic info /detections
```

---

## 📊 Check Performance

### Check publish rate
```bash
ros2 topic hz /detections
# Expected: ~30 Hz
```

### Check message content
```bash
ros2 topic echo /detections --once
```

### Monitor FPS
```bash
ros2 topic echo /perception/fps
# Should report 30.0
```

---

## ✅ Dependencies Check

**Required for mock detector**:
- `numpy` (Python package)
- `rclpy` (from ROS 2)
- `vision_msgs` (from ROS 2)
- `geometry_msgs` (from ROS 2)
- `std_msgs` (from ROS 2)

**NOT required for mock detector**:
- ❌ ultralytics (YOLO)
- ❌ OpenCV
- ❌ Camera hardware
- ❌ CUDA/TensorRT

---

## 🎯 What You're Testing

```
Mock Detector (sim_test.launch.py)
    ↓
Publishes /detections (fake bounding boxes)
    ↓
Role 2/4 subscribe to test control
    ↓
No camera or YOLO needed!
```

---

## ✅ If Everything Works

You should see:
1. ✅ Mock detector launches
2. ✅ `/detections` publishing at 30 Hz
3. ✅ Valid Detection2DArray messages
4. ✅ Target position changes over time (if moving)
5. ✅ No errors in console

**You're ready to test control algorithms!**

---

## 📞 Next Steps

- Integrate with PX4 SITL
- Build Role 2/4 control node
- Subscribe to `/detections`
- Test yaw control

See `SIMULATION_TESTING.md` for full integration guide.

---

**Document Version**: 1.0  
**Questions?** Check the error message and troubleshooting section above.