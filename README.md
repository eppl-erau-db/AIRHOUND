# AIRHOUND

Autonomous drone yaw-to-target tracking system using computer vision and PX4 offboard control.

## Overview

AIRHOUND is a ROS2-based system that enables a drone to autonomously track and follow a target using:
- **YOLOv8** object detection (TensorRT optimized for Jetson)
- **Tracking geometry** for pixel-to-yaw conversion  
- **PX4 offboard control** via DDS (XRCE-DDS)

```
Camera --> Detection --> Tracking --> PX4 Offboard --> Drone Yaw Control
```

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble (or Jazzy)
- Python 3.10+

### Installation

```bash
# Clone the repository
git clone https://github.com/eppl-erau-db/AIRHOUND.git
cd AIRHOUND

# Install MicroXRCE-DDS Agent and build workspace
./middleware/setup_dependencies.sh

# (Optional) Install PX4-Autopilot for simulation
./scripts/start_px4_sitl.sh --install
```

### Running Simulation Mode

Simulation mode uses synthetic detections - no camera or YOLO model needed!

```bash
# Full simulation (starts PX4 SITL + Gazebo + all ROS2 nodes)
./launch_airhound.sh sim

# If PX4 SITL is already running in another terminal:
./launch_airhound.sh sim --no-px4
```

### Running Flight Mode

Flight mode uses real camera input and YOLO detection.

```bash
# Flight mode (RealSense camera + YOLO + real PX4)
./launch_airhound.sh flight
```

## Architecture

### Data Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Perception    │────▶│    Tracking     │────▶│  PX4 Converter  │────▶│      PX4        │
│  (mock/YOLO)    │     │  (YawErrorNode) │     │                 │     │  (SITL/Real)    │
└─────────────────┘     └─────────────────┘     └─────────────────┘     └─────────────────┘
        │                       │                       │                       │
   /detections           /target_yaw             /fmu/in/*              Drone moves!
   /camera/camera_info     (Float32)           (px4_msgs)
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/detections` | `vision_msgs/Detection2DArray` | Object detection results |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/target_yaw` | `std_msgs/Float32` | Yaw rate command (rad/s) |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | PX4 control mode |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Yaw/position setpoint |

### Packages

| Package | Description |
|---------|-------------|
| `airhound_perception` | Detection nodes (YOLO and mock detector) |
| `Tracking-Geometry` | Pixel-to-yaw conversion |
| `offboard_control` | PX4 offboard interface |

## Configuration

All tunable parameters are in `config/airhound.yaml`:

```yaml
# Mode: "sim" or "flight"
mode: "sim"

# Mock detector (sim mode)
mock_detector:
  moving_target: true
  motion_type: "circular"  # circular, sinusoidal, figure8, static
  motion_speed: 0.5        # rad/s

# Tracking
tracking:
  max_rate: 1.0      # Max yaw rate (rad/s)
  deadband: 0.01     # Deadband threshold (rad)

# PX4
px4:
  auto_arm: true
  publish_rate: 10.0  # Hz
```

## Project Structure

```
AIRHOUND/
├── launch_airhound.sh          # Main entry point
├── config/
│   └── airhound.yaml           # Unified configuration
├── launch/
│   ├── e2e_sim.launch.py       # Simulation launch file
│   └── e2e_flight.launch.py    # Flight launch file
├── scripts/
│   └── start_px4_sitl.sh       # PX4 SITL helper
├── middleware/
│   ├── setup_dependencies.sh   # Install dependencies
│   └── start_microxrce_agent.sh
└── ws_ros2/
    └── src/
        ├── airhound_perception/   # Detection package
        ├── Tracking-Geometry/     # Tracking package
        └── offboard_control/      # PX4 interface package
```

## Step-by-Step: Running Simulation

### Terminal 1: Start PX4 SITL + Gazebo

```bash
# Option A: Use the helper script
./scripts/start_px4_sitl.sh

# Option B: Manual (if PX4 is installed elsewhere)
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Wait for "Ready for takeoff!" message.

### Terminal 2: Start MicroXRCE-DDS Agent

```bash
./middleware/start_microxrce_agent.sh
```

### Terminal 3: Launch AIRHOUND

```bash
./launch_airhound.sh sim --no-px4 --no-agent
```

Watch Gazebo - the drone should arm, takeoff, and start rotating to track the simulated target!

## Troubleshooting

### "MicroXRCEAgent not found"

Run the dependency setup script:
```bash
./middleware/setup_dependencies.sh
```

### "Package not found" errors

Build the workspace:
```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Or use the launcher with `--build` flag:
```bash
./launch_airhound.sh sim --build
```

### Drone doesn't move

1. Check PX4 SITL shows "Ready for takeoff!"
2. Check MicroXRCE Agent shows connection messages
3. Verify topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic echo /target_yaw
   ros2 topic echo /fmu/in/trajectory_setpoint
   ```

### ROS2 not found

Install ROS2 Humble:
```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions
source /opt/ros/humble/setup.bash
```

## Development

### Manual Launch (for debugging)

```bash
# Terminal 1: Mock detector
ros2 run airhound_perception mock_detector

# Terminal 2: Tracking node
ros2 run Tracking-Geometry tracking_node

# Terminal 3: PX4 converter
ros2 run offboard_control px4_converter_node

# Terminal 4: Monitor
ros2 topic echo /target_yaw
```

### Rebuilding Packages

```bash
cd ws_ros2
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

## Hardware Requirements

### Simulation
- Any modern Linux PC
- GPU recommended for Gazebo

### Flight
- NVIDIA Jetson (Orin/Xavier)
- Intel RealSense D455 camera
- PX4 flight controller (Pixhawk)
- Companion computer connection (Ethernet/UART)

## Contributing

1. Create a feature branch from `integration/roles-1-2-3-e2e`
2. Make changes and test in simulation
3. Submit PR to merge into `main`

## License

[Add license here]

## Authors

EPPL - Embry-Riddle Aeronautical University
