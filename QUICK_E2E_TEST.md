# Quick E2E Test — Simulation Machine

**Branch:** `integration/roles-1-2-3-e2e`  
**Time:** ~10 minutes  
**Goal:** Verify mock_detector → tracking_node → simple_test_node pipeline

---

## Fast Setup (Copy-Paste)

```bash
# 1. Clone and checkout
cd ~/
git clone https://github.com/eppl-erau-db/AIRHOUND.git
cd AIRHOUND
git checkout integration/roles-1-2-3-e2e

# 2. Build
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 3. Source workspace
source install/setup.bash
```

---

## Run Test (3 Terminals)

**Terminal 1:**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run airhound_perception mock_detector
```

**Terminal 2:**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run Tracking-Geometry tracking_node
```

**Terminal 3:**
```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash
ros2 run offboard_control simple_test_node
```

---

## Verify (Terminal 4)

```bash
source ~/AIRHOUND/ws_ros2/install/setup.bash

# Check topics exist
ros2 topic list
# Expected: /detections, /target_yaw

# Check rates
ros2 topic hz /detections    # Should be ~30 Hz
ros2 topic hz /target_yaw    # Should be ~30 Hz

# Sample messages
ros2 topic echo /detections --once
ros2 topic echo /target_yaw --once
```

---

## Success Criteria

✅ All 3 nodes running without crashes  
✅ `/detections` publishing at ~30 Hz  
✅ `/target_yaw` publishing at ~30 Hz  
✅ No Python errors or segfaults

---

## Quick Troubleshooting

**Node crashes on start?**
```bash
# Update to latest fix
git pull origin integration/roles-1-2-3-e2e
colcon build --packages-select airhound_perception
```

**No topics visible?**
```bash
# Check ROS domain
export ROS_DOMAIN_ID=0
# Restart all nodes
```

**`px4_msgs` not found?**
```bash
git submodule update --init --recursive
colcon build
```

---

**Full guide:** See `SIM_MACHINE_E2E_STEPS.md`
