# AIRHOUND Agent Instructions

AIRHOUND is an autonomous drone yaw-to-target tracking system running on NVIDIA Jetson Orin.

## Data Flow

```
RealSense D455 / Synthetic
        |
  detector_node (RF-DETR or YOLOv8, TensorRT FP16)
        | /detections (Detection2DArray)
  yaw_error_node (pixel → yaw error, Kalman filter, PINN dropout recovery)
        | /yaw_command (Float32)
  px4_converter → PX4 Offboard via uXRCE-DDS
```

## Repository Layout

```
config/airhound.yaml          # Single config file for all nodes
launch/
  e2e_sim.launch.py           # SITL mode
  e2e_flight.launch.py        # Hardware flight mode
ws_ros2/src/
  airhound_perception/        # detector_node, yolo/rfdetr/mock wrappers
  tracking_geometry/          # yaw_error_node, Kalman filter, PINN node
  offboard_control/           # px4_converter
  msg/                        # Custom message definitions
models/                       # Weights (LFS) — .engine files built on Jetson
docs/HITL_SETUP.md            # HITL procedure
manuscript/                   # SPIE paper (LaTeX)
```

## Build

```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Always run colcon from `ws_ros2/`, not the repo root.**

## Critical Pitfalls

1. **Camera topic namespace** — `realsense2_camera` publishes under `/camera/camera/` (double), not `/camera/`. Real topics: `/camera/camera/color/image_raw`, `/camera/camera/depth/image_raw`.
2. **TensorRT engines are platform-specific** — must be built on the Jetson, never on x86.
3. **colcon from root** — creates `build/`, `install/`, `log/` at repo root (gitignored). Always build from `ws_ros2/`.
4. **DO NOT search/grep in `build/`, `install/`, `log/`** — thousands of generated files, causes context blowout.
5. **ModemManager conflicts** — stop it before using Pixhawk over USB: `sudo systemctl stop ModemManager`.
6. **PX4 MAVLink UDP is unicast** — send a heartbeat to `192.168.0.4:14550` before expecting replies.
7. **fmu-v6x Ethernet config** — uses `netman` + `/fs/microsd/net.cfg`, NOT `NET_IP0`–`NET_IP3` parameters. See `docs/HITL_SETUP.md`.
8. **px4_msgs must match PX4 firmware version** — resync if DDS bridge drops.

## Hardware

- **Companion**: Jetson Orin Nano 16GB, `eno1` at `192.168.0.3`, JetPack 6.2.1
- **Flight Controller**: Auterion PX4 FMU v6X.x, Ethernet `192.168.0.4`
- **Camera**: Intel RealSense D455
- **HITL**: `docs/HITL_SETUP.md`
