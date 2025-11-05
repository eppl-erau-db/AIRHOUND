# Quick Start Guide - Sim Computer Testing

**Branch:** `integration/roles-1-2-3-e2e`  
**Purpose:** Test complete E2E integration of Roles 1-3

---

## One-Command Test

```bash
# On the sim computer
cd ~/path/to/AIRHOUND
git checkout integration/roles-1-2-3-e2e
./test_e2e_integration.sh
```

That's it! The script will:
- ✅ Check you're on the right branch
- ✅ Source ROS 2 (Humble or Jazzy)
- ✅ Build the workspace
- ✅ Validate all nodes
- ✅ Launch everything in tmux with monitoring

---

## What Gets Tested

```
Mock Detector → Tracking Node → Offboard Converter
   (Role 1)        (Role 2)          (Role 3)
```

**Topics:**
- `/detections` (vision_msgs/Detection2DArray) - Role 1 → Role 2
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Role 1 → Role 2
- `/target_yaw` (std_msgs/Float32) - Role 2 → Role 3

---

## Using tmux (automatically launched)

**Switch between windows:**
- `Ctrl+b` then `0` - Mock Detector
- `Ctrl+b` then `1` - Tracking Node  
- `Ctrl+b` then `2` - Offboard Converter
- `Ctrl+b` then `3` - Monitor/Commands

**Other tmux commands:**
- `Ctrl+b` then `[` - Scroll mode (press `q` to exit)
- `Ctrl+b` then `d` - Detach (leave running in background)
- `Ctrl+C` in any window - Stop that node
- Exit script - Kills entire session

---

## Manual Testing (without tmux)

If you don't have tmux or prefer manual control:

**Terminal 1:**
```bash
cd ws_ros2
source install/setup.bash
ros2 run airhound_perception mock_detector
```

**Terminal 2:**
```bash
cd ws_ros2
source install/setup.bash
ros2 run Tracking-Geometry tracking_node
```

**Terminal 3:**
```bash
cd ws_ros2
source install/setup.bash
ros2 run offboard_control px4_converter_node_simple
```

**Terminal 4 (monitoring):**
```bash
ros2 topic list
ros2 topic echo /target_yaw
ros2 topic hz /detections
```

---

## Success Criteria

✅ **Pass if you see:**
- Mock detector publishing at ~30 Hz
- `/detections` topic active
- `/target_yaw` topic active with Float32 messages
- Tracking node receiving detections
- No Python import errors for `tracking_geometry`

⚠️ **Expected warnings (OK to ignore):**
- Camera info contains zeros → tracking outputs `inf` (non-blocking)
- Package name warning about `Tracking-Geometry` hyphens (cosmetic only)

❌ **Fail if you see:**
- `ModuleNotFoundError: No module named 'Tracking-Geometry'` (should be FIXED)
- Build errors
- No topics publishing

---

## Next: Full SITL Test

After no-sim test passes, proceed to full PX4 SITL test:

```bash
# See detailed instructions in:
cat E2E_INTEGRATION_TESTING.md
```

You'll need:
- PX4 SITL running (`make px4_sitl gz_x500`)
- MicroXRCE Agent (`./middleware/start_microxrce_agent.sh`)
- Then run the nodes to connect to real PX4

---

## Troubleshooting

**Build fails:**
```bash
./test_e2e_integration.sh --clean
```

**Wrong branch:**
```bash
git checkout integration/roles-1-2-3-e2e
git pull origin integration/roles-1-2-3-e2e
```

**Nodes don't start:**
```bash
# Check ROS 2 is sourced
echo $ROS_DISTRO

# Should show: humble or jazzy
# If empty:
source /opt/ros/humble/setup.bash  # or jazzy
```

**No tmux:**
```bash
sudo apt install tmux
# Then re-run the script
```

---

## Files in This Branch

- `E2E_INTEGRATION_TESTING.md` - Comprehensive testing guide
- `test_e2e_integration.sh` - Automated test script (this uses tmux)
- `middleware/README.md` - PX4 integration details
- `ws_ros2/src/` - All role packages (with fixes)

---

## Questions?

Refer to:
1. This file for quick commands
2. `E2E_INTEGRATION_TESTING.md` for detailed testing scenarios
3. `middleware/README.md` for PX4 SITL setup

Or check the audit thread for background on fixes applied.