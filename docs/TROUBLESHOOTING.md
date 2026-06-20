# AIRHOUND Troubleshooting

## "MicroXRCEAgent not found"

```bash
./middleware/setup_dependencies.sh
```

## "Package not found" errors

```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
cd ..  # Return to AIRHOUND root before running launch scripts
```

Or use the launcher with `--build` (from the repo root):
```bash
./launch_airhound.sh sim --build
```

## CMake cache errors (wrong directory paths)

If you see errors like `The source directory "/home/other_user/..." does not exist`:

```bash
# Clean stale build artifacts and rebuild
cd ws_ros2
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
cd ..  # Return to AIRHOUND root
```

## Build hangs at 0%

This is normal for the first build. The `px4_msgs` package generates 170+ message type headers before compilation starts. Wait ~15 minutes on Jetson. You can verify progress:

```bash
# Check if files are being generated
ls ws_ros2/build/px4_msgs/rosidl_generator_cpp/px4_msgs/msg/ | wc -l
```

## Drone doesn't move

1. Check PX4 SITL shows "Ready for takeoff!"
2. Verify MicroXRCE Agent connection
3. Check topics:
   ```bash
   ros2 topic echo /yaw_command
   ros2 topic echo /fmu/in/trajectory_setpoint
   ```

## Camera issues

```bash
# Validate camera
python3 scripts/validate_camera.py

# Check RealSense streams
# NOTE: realsense2_camera publishes under /camera/camera/ (nested namespace)
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_raw

# If no camera available, use synthetic mode
./launch_airhound.sh flight --synthetic
```

> **RealSense topic namespace:** The `realsense2_camera` driver publishes under
> `/camera/camera/` by default (e.g. `/camera/camera/color/image_raw`), not `/camera/`.
> Scripts that check `/camera/color/image_raw` will show a false negative — verify with
> `ros2 topic list | grep camera`.

## Depth values are NaN

1. Verify target is within range (0.4m - 6.0m for D455)
2. Check depth stream: `ros2 topic hz /camera/camera/depth/image_raw`
3. Increase `depth_window_size` for noisy environments

## PX4 / Ethernet / HITL

| Problem | Solution |
|---------|----------|
| `mavlink_shell.py` hangs | `sudo systemctl stop ModemManager` then retry |
| `NET_IP0 not found` in NSH | fmu-v6x uses `netman` + SD card `net.cfg`, not NET_IP params |
| Can't ping Pixhawk (192.168.0.4) | Add Jetson route, see [HITL_SETUP.md](HITL_SETUP.md) |
| No PX4 ROS2 topics | Start DDS agent: `MicroXRCEAgent udp4 -p 8888` on Jetson |
| HITL loop not active | Confirm `SYS_AUTOSTART=1001`, `MAV_2_CONFIG=1000`, Pixhawk reachable |
