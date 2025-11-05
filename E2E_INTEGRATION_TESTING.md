# E2E Integration Testing Guide
**Branch:** `integration/roles-1-2-3-e2e`

This branch contains the complete integration of Roles 1-3 for end-to-end testing on the sim computer.

---

## What's in this Branch

### ✅ Role 1 - Perception (`airhound_perception`)
- Mock detector for SITL testing (no camera/YOLO needed)
- Publishes: `/detections` (vision_msgs/Detection2DArray)
- Location: `ws_ros2/src/airhound_perception/`

### ✅ Role 2 - Tracking (`Tracking-Geometry`)
- **FIXED:** Python module renamed to `tracking_geometry/` (was causing import errors)
- **FIXED:** Entry point now uses `tracking_geometry.tracking:main`
- Subscribes: `/detections`, `/camera/camera_info`
- Publishes: `/target_yaw` (std_msgs/Float32)
- Location: `ws_ros2/src/Tracking-Geometry/`

### ✅ Role 3 - Offboard Control (`offboard_control`)
- Subscribes: `/target_yaw` (std_msgs/Float32)
- Publishes: PX4 offboard control messages
- Location: `ws_ros2/src/offboard_control/`

### ✅ Middleware - PX4 Integration Tools
- README with setup instructions
- `setup_dependencies.sh` - installs MicroXRCE-DDS Agent
- `start_microxrce_agent.sh` - starts the ROS 2 ↔ PX4 bridge
- Workspace with additional converter nodes and launch files
- Location: `middleware/`

---

## Quick Start on Sim Computer

### 1. Clone and Checkout Integration Branch
```bash
cd ~/path/to/AIRHOUND
git fetch origin
git checkout integration/roles-1-2-3-e2e
```

### 2. Build Workspace
```bash
cd ws_ros2
colcon build
source install/setup.bash
```

### 3. Build Middleware (if using PX4 SITL)
```bash
cd ../middleware/DDS_to_PX4_middleware/workspace

# Clone px4_msgs if not already present
# (it should be in the workspace/src/ already)

colcon build
source install/setup.bash
```

---

## Testing Scenarios

### Option A: No-Sim Test (3 ROS 2 Nodes Only)

Test the perception → tracking → offboard pipeline **without** PX4.

**Terminal 1: Mock Detector**
```bash
cd ws_ros2
source install/setup.bash
ros2 run airhound_perception mock_detector
```

**Terminal 2: Tracking Node**
```bash
cd ws_ros2
source install/setup.bash
ros2 run Tracking-Geometry tracking_node
```

**Terminal 3: Offboard Converter (simple test mode)**
```bash
cd ws_ros2
source install/setup.bash
ros2 run offboard_control px4_converter_node_simple
```

**Terminal 4: Monitor Topics**
```bash
# Watch detections
ros2 topic echo /detections

# Watch yaw output from tracking
ros2 topic echo /target_yaw

# List all topics
ros2 topic list
```

---

### Option B: Full SITL Test (with PX4 + Gazebo)

Complete integration test with PX4 simulation.

**Terminal 1: PX4 SITL + Gazebo**
```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_x500
# Wait for "Commander: Ready for takeoff!"
```

**Terminal 2: MicroXRCE Agent**
```bash
cd middleware
./start_microxrce_agent.sh
# Should see "Micro XRCE-DDS Agent running..."
```

**Terminal 3: Mock Detector**
```bash
cd ws_ros2
source install/setup.bash
ros2 run airhound_perception mock_detector
```

**Terminal 4: Tracking Node**
```bash
cd ws_ros2
source install/setup.bash
ros2 run Tracking-Geometry tracking_node
```

**Terminal 5: PX4 Converter**
```bash
cd middleware/DDS_to_PX4_middleware/workspace
source install/setup.bash
ros2 run offboard_control px4_converter_node
```

**Monitor PX4 Topics:**
```bash
# In another terminal
ros2 topic list | grep px4
ros2 topic echo /fmu/out/vehicle_status
```

---

## Known Issues / Notes

### Mock Detector Camera Info
The mock detector publishes zeros for camera intrinsics by default. This causes divide-by-zero in tracking (produces `inf` values). For real testing, either:
1. Use a real camera node that publishes valid `/camera/camera_info`
2. Modify mock_detector to publish realistic intrinsics
3. Accept that tracking outputs will be `inf` (non-blocking for message flow testing)

### Package Naming Warning
You may see a warning about `Tracking-Geometry` package name using hyphens. This is cosmetic - the internal Python module is now correctly named `tracking_geometry`, so the node will work.

### Middleware Build Artifacts
The middleware workspace has a `.gitignore` to exclude `build/`, `install/`, and `log/` directories. If you see these in git status, they should already be ignored.

---

## Expected Data Flow

```
┌─────────────────┐
│  mock_detector  │
│   (Role 1)      │
└────────┬────────┘
         │ /detections (Detection2DArray)
         │ /camera/camera_info (CameraInfo)
         ↓
┌─────────────────┐
│  tracking_node  │
│   (Role 2)      │
└────────┬────────┘
         │ /target_yaw (Float32)
         ↓
┌─────────────────┐
│ px4_converter   │
│   (Role 3)      │
└────────┬────────┘
         │ PX4 offboard msgs
         ↓
┌─────────────────┐
│ MicroXRCE Agent │
│      Bridge     │
└────────┬────────┘
         │ uXRCE-DDS
         ↓
┌─────────────────┐
│   PX4 SITL      │
│   + Gazebo      │
└─────────────────┘
```

---

## Success Criteria

- [ ] Mock detector publishes `/detections` at ~30 Hz
- [ ] Tracking node receives detections and publishes `/target_yaw`
- [ ] `/target_yaw` is type `std_msgs/Float32` (not Float64)
- [ ] Offboard node receives `/target_yaw` and processes it
- [ ] (SITL only) PX4 receives offboard control messages
- [ ] (SITL only) Drone responds to yaw commands in Gazebo

---

## Next Steps After Testing

1. If no-sim test works: ✅ Message flow is correct
2. If SITL test works: ✅ Full integration is working
3. Replace mock_detector with real YOLO detector
4. Tune tracking and offboard controller gains
5. Test with real camera on hardware

---

## Troubleshooting

### "ModuleNotFoundError: No module named 'Tracking-Geometry'"
- ✅ **FIXED in this branch** - module is now `tracking_geometry`
- Rebuild the workspace: `colcon build --packages-select Tracking-Geometry`

### "No topic /detections"
- Check mock_detector is running: `ros2 node list`
- Check topic name: `ros2 topic list`

### "QoS mismatch" warnings
- These are expected with some PX4 topics
- Should not prevent message flow

### MicroXRCE Agent connection issues
- Ensure PX4 SITL is running first
- Check port 8888 is not in use: `netstat -tulpn | grep 8888`
- Try restarting the agent script

---

## Questions?

Check the middleware README for PX4 setup details:
```bash
cat middleware/README.md
```

Or review the original E2E test report:
```bash
cat ../E2E_TEST_REPORT.md  # (if present in simulation_workspace/)
```
