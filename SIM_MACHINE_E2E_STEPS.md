# Simulation Machine E2E Test — Roles 1-2-3 Integration

**Branch:** `integration/roles-1-2-3-e2e`  
**Goal:** Verify the 3-node pipeline works end-to-end:  
`mock_detector` → `tracking_node` → `simple_test_node` (or `yaw_controller_node`)

**Tested in:** Multipass VM (Ubuntu 22.04, ROS 2 Humble) ✅  
**Ready for:** Simulation machine testing

---

## Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble (or newer)
- **Hardware:** Jetson or x86_64 with sufficient resources
- **Access:** SSH or local terminal on simulation machine

---

## Step 1: Clone and checkout integration branch

```bash
# On the simulation machine
cd ~/
git clone https://github.com/eppl-erau-db/AIRHOUND.git
cd AIRHOUND
git checkout integration/roles-1-2-3-e2e
git pull origin integration/roles-1-2-3-e2e
```

**Expected:** Branch `integration/roles-1-2-3-e2e` checked out successfully.

---

## Step 2: Build the workspace

```bash
cd ~/AIRHOUND/ws_ros2

# Source ROS 2
source /opt/ros/humble/setup.bash

# Clean previous builds (optional but recommended)
rm -rf build/ install/ log/

# Build all packages
colcon build --symlink-install

# Expected build order:
# 1. px4_msgs (takes ~5-10 min on first build)
# 2. Tracking-Geometry
# 3. airhound_perception
# 4. offboard_control
```

**Success criteria:**
- No build errors (warnings are OK)
- `install/` directory created
- All 4 packages built successfully

**Troubleshooting:**
- If `px4_msgs` missing: ensure you cloned with submodules or build it separately
- If Python module errors: check Python 3.10+ installed

---

## Step 3: Source the workspace

```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
```

**Verify nodes are available:**
```bash
ros2 pkg executables airhound_perception
# Expected: detector_node, mock_detector, synthetic_camera

ros2 pkg executables Tracking-Geometry
# Expected: tracking_node

ros2 pkg executables offboard_control
# Expected: dummy_yaw_publisher, px4_simulator, simple_test_node, yaw_controller_node
```

---

## Step 4: Run the 3-node E2E test

### Option A: Manual launch (3 separate terminals)

**Terminal 1 — Mock Detector (Role 1: Perception)**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run airhound_perception mock_detector
```
**Expected output:**
```
[INFO] [mock_detector_node]: Mock Detector started: 1280x720 @ 30.0 Hz
[INFO] [mock_detector_node]: Publishing 1 target(s) on /detections
[INFO] [mock_detector_node]: Motion: circular, Moving: True
```

**Terminal 2 — Tracking Node (Role 2: Tracking & Geometry)**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run Tracking-Geometry tracking_node
```
**Expected output:**
```
[INFO] [yaw_error_node]: YawErrorNode initialized.
```

**Terminal 3 — Simple Test Node (Role 3: Offboard stub)**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run offboard_control simple_test_node
```
**Expected output:**
```
[INFO] [simple_test_node]: Enhanced SimpleTestNode started - monitoring yaw commands
```

---

### Option B: Automated script (single terminal)

Use the provided test script:
```bash
cd ~/AIRHOUND/ws_ros2
bash ../test_e2e_integration.sh
```

This launches all 3 nodes in a tmux session. Attach with:
```bash
tmux attach -t e2e_test
```

Navigate panes with `Ctrl+b` then arrow keys.  
Kill all with `Ctrl+b`, type `:kill-session`

---

## Step 5: Verify the pipeline

Open a **4th terminal** (monitoring terminal):

```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
```

### Check topics are publishing:
```bash
ros2 topic list
```
**Expected topics:**
- `/detections` — vision_msgs/Detection2DArray from mock_detector
- `/target_yaw` — std_msgs/Float32 from tracking_node
- `/parameter_events`, `/rosout` (standard)

### Inspect /detections:
```bash
ros2 topic echo /detections --once
```
**Expected:**
- `detections` array with 1 detection
- `bbox.center.position.x` and `.y` values (e.g., 640.0, 360.0)
- `results[0].hypothesis.class_id: "0"`
- `results[0].hypothesis.score: 0.95`

### Inspect /target_yaw:
```bash
ros2 topic echo /target_yaw --once
```
**Expected:**
- `data: <some float value>` (e.g., `0.0`, `0.523`, `-1.2`, etc.)
- This is the yaw command computed from bbox center

### Check message rates:
```bash
ros2 topic hz /detections
# Expected: ~30 Hz (mock detector publishes at 30 FPS)

ros2 topic hz /target_yaw
# Expected: ~30 Hz (tracking publishes whenever it receives detections)
```

---

## Step 6: Success criteria

✅ **PASS if all of the following are true:**

1. All 3 nodes start without crashes
2. `/detections` published at ~30 Hz with valid bbox data
3. `/target_yaw` published at ~30 Hz with Float32 values
4. `simple_test_node` logs show it's receiving yaw commands
5. No Python tracebacks or segfaults in any terminal

❌ **FAIL if any of:**
- Nodes crash on startup
- `/detections` or `/target_yaw` missing or silent
- AttributeError, ImportError, or build errors
- Rates significantly lower than 30 Hz (< 10 Hz indicates problem)

---

## Step 7: Next steps (after E2E passes)

### A. Test with PX4 SITL (Gazebo)

If you want to integrate with actual PX4 simulation:

1. Start PX4 SITL + Gazebo:
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gz_x500
   ```

2. Start MicroXRCE-DDS Agent:
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. Replace `simple_test_node` with `yaw_controller_node`:
   ```bash
   ros2 run offboard_control yaw_controller_node
   ```

4. Verify PX4 receives `/fmu/in/offboard_control_mode` and `/fmu/in/trajectory_setpoint`

---

### B. Merge to main (if E2E passes)

Once verified on simulation machine:

```bash
# Create PR from integration branch to main
git checkout main
git pull origin main
git merge integration/roles-1-2-3-e2e
git push origin main
```

**OR** open a GitHub PR for review.

---

## Troubleshooting

### Issue: `mock_detector` crashes with `AttributeError: 'Pose2D' object has no attribute 'x'`

**Cause:** Old version of code before bugfix  
**Fix:** 
```bash
cd ~/AIRHOUND
git pull origin integration/roles-1-2-3-e2e
cd ws_ros2
colcon build --packages-select airhound_perception
```

The latest commit (93fe6849) fixes this.

---

### Issue: `tracking_node` publishes `inf` values on `/target_yaw`

**Cause:** Mock detector has zero or invalid intrinsics (divide-by-zero in tracking geometry)  
**Expected behavior:** This is known; mock_detector uses placeholder intrinsics.  
**Fix (optional):** Edit `mock_detector.py` to set realistic `fx`, `fy`, `cx`, `cy` in CameraInfo.

---

### Issue: Nodes can't find each other / no topics

**Cause:** ROS domain mismatch or network issues  
**Fix:**
```bash
# Check ROS domain
echo $ROS_DOMAIN_ID

# Set to same value on all terminals (e.g., 0)
export ROS_DOMAIN_ID=0

# Restart all nodes
```

---

### Issue: Build fails with "package 'px4_msgs' not found"

**Cause:** Submodule not initialized  
**Fix:**
```bash
cd ~/AIRHOUND
git submodule update --init --recursive
cd ws_ros2
colcon build
```

---

## Logs and Evidence

After successful test, collect evidence:

```bash
# Record a bag for 10 seconds
ros2 bag record -a -o ~/e2e_test_bag --duration 10

# Screenshot or log terminal outputs
# Take note of:
# - Node startup logs
# - Topic hz measurements
# - Sample messages from /detections and /target_yaw
```

Attach logs to PR or issue for review.

---

## Contact / Support

- **Integration lead:** See repo contributors
- **Issues:** Open GitHub issue with label `integration-test`
- **Scope reference:** `scopework.md` (Roles 1-3 definitions)

---

**Document version:** 1.0  
**Last updated:** 2025-01-25  
**Tested on:** Multipass VM (Ubuntu 22.04, ROS 2 Humble)