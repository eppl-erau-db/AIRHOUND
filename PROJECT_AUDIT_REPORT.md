# AIRHOUND Project - Comprehensive Audit Report
**Date:** November 4, 2025  
**Deadline:** Poster Symposium in 8 days  
**Repository:** eppl-erau-db/AIRHOUND

---

## Executive Summary

### 🚨 CRITICAL FINDINGS:
1. **Role 1 (Perception):** ✅ READY FOR SIM
2. **Role 2 (Tracking):** ⚠️ INCOMPLETE - Missing entry point, has bug
3. **Role 3 (Offboard):** ⚠️ EXISTS but NOT in ROS 2 package format  
4. **Role 4 (Simulation):** ❌ NOT FOUND - No Gazebo/SITL integration
5. **Integration:** ❌ NO END-TO-END LAUNCH FILE

---

## Branch Analysis

### Branches in Repository:
```
origin/main                              [INTEGRATION BRANCH]
origin/feature/role1-perception-integration  
origin/role2-tracking-geometry
origin/middleware-test
origin/px4-converter-demo
origin/test-working-sitl                 [SITL BASE]
origin/master                            [DEPRECATED]
```

### Main Branch Status (origin/main)
**Latest Commit:** `e0be4bef` - Merge PR #4 (perception integration)

**Packages Found:**
- ✅ `airhound_perception` (Role 1)
- ⚠️ `Tracking-Geometry` (Role 2) - incomplete

**Missing:**
- ❌ Offboard control package (Role 3)
- ❌ Simulation/Gazebo package (Role 4)
- ❌ Integration launch files

---

## Role-by-Role Assessment

### ROLE 1 — Perception (RealSense → YOLO → /detections)

**Branch:** `feature/role1-perception-integration` (merged to main)  
**Package:** `airhound_perception`  
**Status:** ✅ **READY FOR SIM TESTING**

#### Deliverables vs Requirements:

| Requirement | Status | Notes |
|-------------|--------|-------|
| RealSense camera launch | ✅ | `perception.launch.py` |
| /detections topic (Detection2DArray) | ✅ | Published at target rate |
| YOLO TensorRT integration | ✅ | Optional via `extras_require` |
| **MOCK detector for SIM** | ✅ | `mock_detector.py` + `sim_test.launch.py` |
| Latency/FPS diagnostics | ✅ | `/perception/latency_ms`, `/perception/fps` |
| Bag file for Role 2 | ✅ | `record_bag.launch.py` |
| Documentation | ⚠️ | Needs cleanup |

#### Entry Points:
```python
detector_node = airhound_perception.detector_node:main
mock_detector = airhound_perception.mock_detector:main  
synthetic_camera = airhound_perception.synthetic_camera:main
```

#### Interface Contract (VERIFIED):
```
Input:  /camera/color/image_raw (sensor_msgs/Image)
        /camera/camera_info (sensor_msgs/CameraInfo)
Output: /detections (vision_msgs/Detection2DArray) @ ≥15 Hz
        /perception/fps, /perception/latency_ms
Frame:  camera_color_optical_frame
```

#### ✅ SIM-READY Features:
- Mock detector works WITHOUT camera/YOLO/Gazebo
- Publishes synthetic Detection2DArray
- Configurable motion patterns (stationary, oscillating, circular)
- Noise injection for realistic testing

---

### ROLE 2 — Tracking & Geometry (bbox → yaw)

**Branch:** `role2-tracking-geometry` (merged to main)  
**Package:** `Tracking-Geometry`  
**Status:** ⚠️ **INCOMPLETE - CRITICAL ISSUES**

#### Deliverables vs Requirements:

| Requirement | Status | Notes |
|-------------|--------|-------|
| Subscribe to /detections | ✅ | Implemented |
| CameraInfo processing | ✅ | fx, fy, cx, cy extracted |
| Pixel → yaw conversion | ✅ | Small-angle approximation |
| P/PID controller | ⚠️ | P controller only, no I/D |
| Deadband | ✅ | Configurable |
| Rate limiting | ✅ | max_rate parameter |
| **Entry point in setup.py** | ❌ | **MISSING!** |
| Launch file | ❌ | **MISSING!** |
| Documentation | ❌ | **MISSING!** |

#### 🚨 CRITICAL BUGS FOUND:

**1. Import Error (Line 1):**
```python
#import rclpy          # ← COMMENTED OUT!
from rclpy.node import Node  # But this needs rclpy!
```
**Fix:** Uncomment `import rclpy`

**2. Missing Entry Point:**
```python
entry_points={
    'console_scripts': [
        # EMPTY! Should have: 'tracking_node = Tracking-Geometry.tracking:main'
    ],
},
```

**3. No Launch File:** Cannot be started via `ros2 launch`

#### Interface Contract (from code):
```
Input:  /detections (vision_msgs/Detection2DArray)
        /camera/camera_info (sensor_msgs/CameraInfo)
        /tf (optional, set up but not used)
Output: /target_yaw_rate (std_msgs/Float32)  
        ⚠️ NOTE: Scope says /target_yaw OR /cmd_yaw_rate
             Code publishes /target_yaw_rate (hybrid name!)
```

#### Implementation Details:
- **Algorithm:** Pick highest confidence detection
- **Math:** `yaw_err = (u - cx) / fx`
- **Control:** P controller with deadband
- **Prediction:** Holds last rate when target lost
- **TF2:** Initialized but not actively used

---

### ROLE 3 — Offboard Control & PX4 Integration

**Branch:** `middleware-test`  
**Package:** ❌ **NO ROS 2 PACKAGE** (loose C++ files)  
**Status:** ⚠️ **EXISTS BUT NOT PACKAGED**

#### What EXISTS:
- `workspace/src/yaw_controller_node.cpp` - Offboard yaw controller
- `workspace/src/px4_simulator.cpp` - PX4 simulator stub
- `workspace/src/dummy_yaw_publisher.cpp` - Test publisher
- `workspace/CMakeLists.txt` - Build system

#### What's MISSING:
- ❌ No `package.xml` (not a ROS 2 package!)
- ❌ Not integrated into `ws_ros2/` workspace
- ❌ No launch file
- ❌ Not merged to main

#### Code Analysis (`yaw_controller_node.cpp`):
```cpp
// Subscribes to: /yaw_command (std_msgs/Float64)
// Publishes to:  /fmu/in/offboard_control_mode
//                /fmu/in/trajectory_setpoint
//                /fmu/in/vehicle_command
```

#### 🚨 INTERFACE MISMATCH:
- Role 2 publishes: `/target_yaw_rate` (Float32)
- Role 3 subscribes: `/yaw_command` (Float64)
- **Topic names AND types don't match!**

#### Deliverables vs Requirements:

| Requirement | Status | Notes |
|-------------|--------|-------|
| Offboard mode streaming | ⚠️ | Code exists, not packaged |
| 10-20 Hz setpoint publishing | ⚠️ | Implemented in C++ |
| Arm + mode switch | ⚠️ | Implemented |
| Watchdog (target loss handling) | ❌ | Not visible in code |
| SITL demo | ❌ | Not tested |
| Parameter documentation | ❌ | Missing |

---

### ROLE 4 — Simulation & QA

**Branch:** `test-working-sitl` (supposedly)  
**Package:** ❌ **NOT FOUND**  
**Status:** ❌ **CRITICAL GAP**

#### What's MISSING:
- ❌ No Gazebo (GZ Sim) world files
- ❌ No `ros_gz_bridge` configuration
- ❌ No SITL integration launch
- ❌ No repeatable target motion plugin
- ❌ No rosbag profiles/scripts
- ❌ No regression tests
- ❌ No QA documentation

#### What EXISTS:
- `ws_ros2/src/scripts/` has example PX4 code (listeners, offboard examples)
- `ws_ros2/src/msg/` has 235 PX4 message definitions

#### 🚨 THIS IS THE BIGGEST GAP
Without this, you cannot:
- Test end-to-end in simulation
- Verify integration before hardware
- Record repeatable test scenarios

---

## Integration Analysis

### End-to-End Pipeline Status

```
[Camera/Mock] → [Perception] → [Tracking] → [Offboard] → [PX4/SITL]
     ✅            ✅             ⚠️           ⚠️           ❌
```

### Interface Contracts - ACTUAL vs EXPECTED

| From → To | Expected | Actual | Match? |
|-----------|----------|--------|--------|
| Perception → Tracking | `/detections` (Detection2DArray) | `/detections` | ✅ |
| Tracking → Offboard | `/target_yaw` (Float32) OR `/cmd_yaw_rate` (Vector3) | `/target_yaw_rate` (Float32) | ⚠️ Name mismatch |
| Offboard listens for | `/target_yaw` or `/cmd_yaw_rate` | `/yaw_command` (Float64) | ❌ BROKEN |

### Launch Files Status:

| Component | Launch File | Status |
|-----------|-------------|--------|
| Perception (real) | `perception.launch.py` | ✅ |
| Perception (mock) | `sim_test.launch.py` | ✅ |
| Tracking | ❌ MISSING | ❌ |
| Offboard | ❌ MISSING | ❌ |
| **FULL SYSTEM** | ❌ MISSING | ❌ |
| SITL+Gazebo | ❌ MISSING | ❌ |

---

## Critical Issues Summary

### 🔴 BLOCKERS (Must fix before sim test):

1. **Role 2 (Tracking):**
   - Fix: Uncomment `import rclpy`
   - Fix: Add entry point to `setup.py`
   - Fix: Create launch file
   - Verify: Topic name `/target_yaw_rate` vs contract

2. **Role 3 (Offboard):**
   - Fix: Convert to proper ROS 2 package
   - Fix: Change subscriber topic from `/yaw_command` to `/target_yaw_rate`
   - Fix: Change message type from Float64 to Float32
   - Fix: Move to `ws_ros2/src/`
   - Create: Launch file

3. **Role 4 (Simulation):**
   - Create: Gazebo world with quad
   - Create: Camera bridge
   - Create: SITL integration launch
   - Create: System-level launch file

4. **Integration:**
   - Create: Master launch file that starts all nodes
   - Verify: All topic names/types match
   - Test: End-to-end in SITL

---

## What Works RIGHT NOW

### ✅ You CAN Test:
```bash
# Terminal 1: Start mock detector
ros2 launch airhound_perception sim_test.launch.py

# Terminal 2: Echo detections
ros2 topic echo /detections
```

### ❌ You CANNOT Test (yet):
- Tracking node (missing entry point)
- Offboard control (not packaged)
- Full pipeline
- Anything with Gazebo/SITL

---

## Recommended Action Plan (8-day timeline)

### 🔥 Day 1-2 (CRITICAL PATH):

**Role 2 Fixes:**
```bash
# 1. Fix import
sed -i 's/#import rclpy/import rclpy/' ws_ros2/src/Tracking-Geometry/Tracking-Geometry/tracking.py

# 2. Fix setup.py
# Add: 'tracking_node = Tracking-Geometry.tracking:main'

# 3. Create launch file
# Create: ws_ros2/src/Tracking-Geometry/launch/tracking.launch.py
```

**Role 3 Packaging:**
- Move `workspace/` code into proper ROS 2 package: `ws_ros2/src/airhound_offboard/`
- Fix topic interface to match Role 2
- Create launch file

### Day 3-4 (INTEGRATION):

**Role 4 - Minimal Sim:**
- Create basic Gazebo world (can use stock PX4 world)
- Create system launch file:
  - Mock detector
  - Tracking node
  - Offboard controller
  - Micro XRCE-DDS Agent
  - PX4 SITL

### Day 5-6 (TESTING):
- End-to-end SITL testing
- Fix bugs found in integration
- Record demo video
- Create rosbags

### Day 7-8 (DOCUMENTATION):
- Create poster content
- Document known issues/limitations
- Prepare demo script
- Safety checklist

---

## Files That Need Creation

### Tracking Package:
```
ws_ros2/src/Tracking-Geometry/launch/tracking.launch.py
```

### Offboard Package (needs creation):
```
ws_ros2/src/airhound_offboard/
  ├── package.xml
  ├── CMakeLists.txt
  ├── src/
  │   └── yaw_controller_node.cpp
  └── launch/
      └── offboard.launch.py
```

### Simulation Package (needs creation):
```
ws_ros2/src/airhound_sim/
  ├── package.xml
  ├── worlds/
  │   └── airhound_world.sdf
  ├── launch/
  │   ├── sitl.launch.py
  │   └── full_system.launch.py
  └── config/
      └── bridge.yaml
```

---

## Summary for Poster

### What You HAVE:
- ✅ Perception system (YOLO + mock)
- ✅ Basic tracking algorithm
- ✅ Offboard controller code
- ✅ PX4 message interfaces

### What You NEED (to claim "working"):
- 🔧 Fix 3 bugs in tracking
- 🔧 Package offboard controller
- 🔧 Create simulation environment
- 🔧 Create integration launch
- 🔧 1 successful end-to-end test

### Honest Assessment:
**Current State:** "Components developed, integration in progress"  
**Feasible in 8 days:** Yes, if you focus on SITL-only demo  
**Hardware flight:** Unlikely in 8 days (need sim validation first)

---

## Contact Points / Ownership

Based on branch commits:
- **Role 1 (Perception):** Rylan (you)
- **Role 2 (Tracking):** castej (based on package.xml)
- **Role 3 (Offboard):** Unknown (middleware-test branch)
- **Role 4 (Simulation):** Missing/Unknown

**RECOMMENDATION:** Have an immediate team meeting to:
1. Assign Role 4 ownership
2. Fix critical bugs (30 min of work)
3. Create integration plan
4. Divide remaining tasks

---

**END OF AUDIT**
