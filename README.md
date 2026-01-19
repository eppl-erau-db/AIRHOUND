# AIRHOUND

**Autonomous Intelligent Real-time Hunting Of Uncrewed Navigating Drones**

A ROS2-based autonomous drone pursuit system using transformer-based object detection, depth fusion, and PX4 offboard control. Developed for the **SPIE Defense + Security 2026** conference (DS112: Machine Learning from Challenging Data).

---

## Key Results

| Metric | RF-DETR | YOLOv8n |
|--------|---------|---------|
| **Inference Speed** | 275.7 FPS | 43.0 FPS |
| **Latency** | 3.6 ms | 23.2 ms |
| **mAP@0.5 (clean)** | 0.953 | 0.957 |
| **mAP@0.5 (severe noise)** | 0.716 | 0.318 |

RF-DETR achieves **+40% mAP advantage** under severe image degradation while running **6.4x faster** on Jetson Orin Nano.

---

## Overview

AIRHOUND enables a drone to autonomously detect, track, and pursue a target drone using:

- **RF-DETR** - Transformer-based detector with superior degradation robustness
- **Intel RealSense D455** - RGB + depth fusion for 3D position estimation
- **3D Kalman Filter** - State estimation and prediction during detection dropouts
- **PX4 Offboard Control** - Direct yaw control via DDS (XRCE-DDS)

```
RealSense D455 --> RF-DETR Detection --> 3D Kalman Filter --> PX4 Offboard --> Drone Pursuit
     (RGB+Depth)      (TensorRT)           (Prediction)         (XRCE-DDS)
```

---

## Hardware Requirements

### Flight Platform (Tested Configuration)

| Component | Specification |
|-----------|---------------|
| Companion Computer | NVIDIA Jetson Orin Nano 16GB |
| JetPack | 6.2.1 |
| CUDA | 12.6 |
| TensorRT | 10.3.0 |
| Camera | Intel RealSense D455 |
| Flight Controller | Pixhawk 6C (PX4 v1.15+) |
| Connection | Ethernet (Jetson <-> Pixhawk via MAVLink Router) |

### Development/Simulation

- Ubuntu 22.04
- ROS2 Humble (or Jazzy)
- Python 3.10+
- GPU recommended for Gazebo simulation

---

## Installation

### 1. Clone Repository

```bash
git clone https://github.com/eppl-erau-db/AIRHOUND.git
cd AIRHOUND

# Install Git LFS (required for model files)
sudo apt install git-lfs
git lfs install
git lfs pull
```

### 2. Install Dependencies

```bash
# Install MicroXRCE-DDS Agent and ROS2 dependencies
./middleware/setup_dependencies.sh

# (Optional) Install PX4-Autopilot for simulation
./scripts/start_px4_sitl.sh --install
```

### 3. Build Workspace

```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Quick Start

### Simulation Mode

Simulation uses synthetic detections - no camera or GPU needed:

```bash
# Full simulation (starts PX4 SITL + Gazebo + all ROS2 nodes)
./launch_airhound.sh sim

# If PX4 SITL is already running:
./launch_airhound.sh sim --no-px4
```

### Flight Mode

Flight mode uses RealSense camera and TensorRT-optimized RF-DETR:

```bash
# Pre-flight validation
python3 scripts/validate_camera.py
python3 scripts/flight_preflight_check.py

# Launch flight mode
./launch_airhound.sh flight
```

---

## Architecture

### Data Flow

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   RealSense     │     │   Perception    │     │    Tracking     │     │      PX4        │
│   D455 Camera   │────▶│   (RF-DETR)     │────▶│  (Kalman + Yaw) │────▶│   Offboard      │
└─────────────────┘     └─────────────────┘     └─────────────────┘     └─────────────────┘
   RGB + Depth             /detections            /target_yaw           /fmu/in/*
                      /perception/target_3d
                      /perception/depth_quality
```

### Detector Options

| Detector | Architecture | Strengths | Model File |
|----------|--------------|-----------|------------|
| **RF-DETR** (default) | Transformer | Noise/occlusion robustness, faster on TensorRT | `drone_rfdetr.engine` |
| YOLOv8 | CNN | Slightly higher clean mAP | `yolov8Detector.pt` |

### Depth Integration

The RealSense D455 provides hardware-synchronized RGB and depth. Depth is:
- Fused with detections at the bounding box center
- Filtered with configurable min/max range (default: 0.4m - 6.0m)
- Published as 3D position via `/perception/target_3d`
- Quality metric published to `/perception/depth_quality`

---

## ROS2 Topics

### Perception (Published)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/detections` | `vision_msgs/Detection2DArray` | 30 Hz | 2D bboxes with depth in `pose.z` |
| `/perception/target_3d` | `geometry_msgs/PointStamped` | 30 Hz | 3D position (camera frame) |
| `/perception/depth_quality` | `std_msgs/Float32` | 30 Hz | Depth validity ratio [0-1] |
| `/perception/latency_ms` | `std_msgs/Float32` | 30 Hz | Detection latency |
| `/perception/fps` | `std_msgs/Float32` | 1 Hz | Inference FPS |

### Tracking (Published)

| Topic | Type | Description |
|-------|------|-------------|
| `/target_yaw` | `std_msgs/Float32` | Yaw rate command (rad/s) |

### PX4 Interface

| Topic | Type | Description |
|-------|------|-------------|
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Control mode flags |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position/yaw setpoint |
| `/fmu/out/vehicle_status` | `px4_msgs/VehicleStatus` | Vehicle state |

See [docs/PERCEPTION_INTERFACE.md](docs/PERCEPTION_INTERFACE.md) for detailed message formats.

---

## Configuration

All parameters in `config/airhound.yaml`:

```yaml
mode: "sim"  # "sim" or "flight"

perception:
  detector_type: "rfdetr"           # "rfdetr" or "yolo"
  model_path: "models/drone_rfdetr.engine"
  confidence_threshold: 0.25
  depth:
    enable: true
    min_range: 0.4                  # meters
    max_range: 6.0                  # meters

tracking:
  max_rate: 1.0                     # rad/s
  deadband: 0.01                    # rad

px4:
  auto_arm: true
  publish_rate: 10.0                # Hz
```

---

## Scripts

### Validation & Testing

| Script | Purpose |
|--------|---------|
| `scripts/validate_camera.py` | Verify RealSense D455 connectivity and streams |
| `scripts/integration_test.py` | End-to-end pipeline verification |
| `scripts/flight_preflight_check.py` | Pre-flight checklist for hardware validation |

### Benchmarking & Evaluation

| Script | Purpose |
|--------|---------|
| `scripts/benchmark_comparison.py` | Compare RF-DETR vs YOLOv8 performance |
| `scripts/degradation_evaluation.py` | Evaluate robustness under noise/blur/occlusion |
| `scripts/plot_degradation_results.py` | Generate publication-quality plots |
| `scripts/synthetic_degradation.py` | Apply synthetic degradations to images |

### Data Pipeline

| Script | Purpose |
|--------|---------|
| `scripts/extract_trajectories.py` | Extract trajectory data from rosbags for PINN training |

### Deployment

| Script | Purpose |
|--------|---------|
| `scripts/flight_launch.sh` | Production flight launch script |
| `scripts/start_px4_sitl.sh` | Start PX4 SITL for simulation |

---

## Project Structure

```
AIRHOUND/
├── README.md                       # This file
├── launch_airhound.sh              # Main entry point
├── config/
│   ├── airhound.yaml               # Unified configuration
│   └── realsense_profile.yaml      # Camera profile
├── docs/
│   ├── PERCEPTION_INTERFACE.md     # Perception API documentation
│   ├── PINN_DATA_FORMAT.md         # Trajectory data format for PINN
│   ├── FLIGHT_TESTING_GUIDE.md     # Flight procedures
│   ├── DEGRADATION_EVAL_RESULTS.md # Benchmarking results
│   └── SPIE-Conference-Roadmap.md  # Project timeline
├── models/
│   ├── drone_rfdetr.engine         # TensorRT optimized (Git LFS)
│   └── drone_rfdetr_best.onnx      # ONNX export (Git LFS)
├── scripts/                        # Utility scripts (see above)
├── launch/
│   ├── e2e_sim.launch.py           # Simulation launch
│   └── e2e_flight.launch.py        # Flight launch
├── middleware/
│   ├── setup_dependencies.sh       # Dependency installer
│   └── start_microxrce_agent.sh    # DDS agent launcher
└── ws_ros2/src/
    ├── airhound_perception/        # Detection + depth fusion
    ├── tracking_geometry/          # Kalman filter + yaw control
    └── offboard_control/           # PX4 interface
```

---

## Packages

| Package | Description |
|---------|-------------|
| `airhound_perception` | RF-DETR/YOLO detection with D455 depth fusion |
| `tracking_geometry` | 3D Kalman filter, pixel-to-yaw conversion |
| `offboard_control` | PX4 offboard mode interface |

---

## Benchmarking Results

### TensorRT Performance (Jetson Orin Nano 16GB)

| Model | FPS | Latency | mAP@0.5 |
|-------|-----|---------|---------|
| RF-DETR (TensorRT) | 275.7 | 3.6 ms | 0.953 |
| YOLOv8n (TensorRT) | 43.0 | 23.2 ms | 0.957 |

### Degradation Robustness

| Condition | YOLOv8 mAP | RF-DETR mAP | Advantage |
|-----------|------------|-------------|-----------|
| Clean | 0.957 | 0.953 | -0.4% |
| Gaussian Noise (severe) | 0.318 | 0.716 | **+39.8%** |
| Low Light (severe) | 0.275 | 0.670 | **+39.4%** |
| Motion Blur (severe) | 0.076 | 0.094 | +1.9% |

See [docs/DEGRADATION_EVAL_RESULTS.md](docs/DEGRADATION_EVAL_RESULTS.md) for complete analysis.

---

## Documentation

| Document | Description |
|----------|-------------|
| [PERCEPTION_INTERFACE.md](docs/PERCEPTION_INTERFACE.md) | Topic formats, depth integration, code examples |
| [PINN_DATA_FORMAT.md](docs/PINN_DATA_FORMAT.md) | Trajectory extraction for PINN training |
| [FLIGHT_TESTING_GUIDE.md](docs/FLIGHT_TESTING_GUIDE.md) | Flight procedures and safety |
| [DEGRADATION_EVAL_RESULTS.md](docs/DEGRADATION_EVAL_RESULTS.md) | Full benchmarking analysis |
| [SPIE-Conference-Roadmap.md](docs/SPIE-Conference-Roadmap.md) | Project timeline and milestones |

---

## Troubleshooting

### "MicroXRCEAgent not found"

```bash
./middleware/setup_dependencies.sh
```

### "Package not found" errors

```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Or use the launcher with `--build`:
```bash
./launch_airhound.sh sim --build
```

### Drone doesn't move

1. Check PX4 SITL shows "Ready for takeoff!"
2. Verify MicroXRCE Agent connection
3. Check topics:
   ```bash
   ros2 topic echo /target_yaw
   ros2 topic echo /fmu/in/trajectory_setpoint
   ```

### Camera issues

```bash
# Validate camera
python3 scripts/validate_camera.py

# Check RealSense streams
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

### Depth values are NaN

1. Verify target is within range (0.4m - 6.0m for D455)
2. Check depth stream: `ros2 topic hz /camera/depth/image_raw`
3. Increase `depth_window_size` for noisy environments

---

## Development

### Manual Node Launch (Debugging)

```bash
# Terminal 1: Detection
ros2 run airhound_perception detector_node

# Terminal 2: Tracking
ros2 run tracking_geometry tracking_node

# Terminal 3: PX4 interface
ros2 run offboard_control px4_converter_node

# Terminal 4: Monitor
ros2 topic echo /perception/target_3d
```

### Rebuilding Packages

```bash
cd ws_ros2
colcon build --packages-select <package_name> --symlink-install
source install/setup.bash
```

---

## Contributing

1. Create a feature branch from `main`
2. Make changes and test in simulation
3. Run integration tests: `python3 scripts/integration_test.py`
4. Submit PR with description of changes

### Team Roles

| Role | Responsibility |
|------|----------------|
| Role 1 (Perception Lead) | Detection, depth integration, repo oversight |
| Role 2 (Tracking) | Kalman filter, trajectory prediction |
| Role 3 (Control) | PX4 interface, offboard control |
| Role 4 (PINN) | Physics-informed neural network for prediction |

---

## Citation

If you use AIRHOUND in your research, please cite:

```bibtex
@inproceedings{airhound2026,
  title={AIRHOUND: Autonomous Drone Pursuit with Transformer-Based Detection 
         Under Challenging Visual Conditions},
  author={EPPL Team},
  booktitle={SPIE Defense + Security 2026 (DS112: Machine Learning from Challenging Data)},
  year={2026},
  organization={SPIE}
}
```

---

## License

[MIT License](LICENSE)

---

## Authors

**EPPL Lab** - Embry-Riddle Aeronautical University

*Autonomous drone pursuit research for SPIE Defense + Security 2026*
