# AIRHOUND Agent Instructions

This document provides context for AI coding agents (OpenCode, Claude, etc.) working on the AIRHOUND codebase.

## Project Overview

AIRHOUND is an autonomous drone yaw-to-target tracking system using:
- **Computer Vision**: RF-DETR or YOLOv8 object detection (TensorRT optimized for Jetson)
- **Tracking Geometry**: Pixel-to-yaw conversion for camera-based tracking
- **PX4 Offboard Control**: DDS/XRCE-DDS bridge for drone control

### Data Flow
```
Camera --> Detection --> Tracking --> PX4 Offboard --> Drone Yaw Control
   |           |             |              |
RealSense   RF-DETR/     YawErrorNode   px4_converter
  or        YOLOv8 or
Synthetic   MockDetector
```

### Camera Source Modes

The `camera_source` parameter controls where detections come from (independent of sim/flight mode):

| camera_source | Description | Use Case |
|---------------|-------------|----------|
| `realsense` | Intel RealSense D455 + real detector | Production flight with real perception |
| `synthetic` | Mock detector (no camera needed) | Testing control pipeline on real hardware |

This allows testing the PX4 control pipeline on real flight hardware without needing a camera connected.

## Repository Structure

```
AIRHOUND/
├── AGENTS.md                 # This file - agent instructions
├── README.md                 # User-facing documentation
├── config/
│   └── airhound.yaml         # Unified configuration for all nodes
├── launch/
│   ├── e2e_sim.launch.py     # Simulation mode launch
│   └── e2e_flight.launch.py  # Flight mode launch
├── launch_airhound.sh        # Main entry script
├── scripts/
│   └── start_px4_sitl.sh     # PX4 SITL helper
├── models/                   # Model weights (not in git)
│   ├── *.engine              # TensorRT engines (platform-specific)
│   └── *.onnx                # ONNX models (portable)
├── middleware/
│   └── DDS_to_PX4_middleware/  # PX4 integration (legacy structure)
└── ws_ros2/
    └── src/
        ├── airhound_perception/    # Detection package
        │   └── airhound_perception/
        │       ├── detector_node.py      # Main ROS2 node
        │       ├── yolo_detector.py      # YOLOv8 wrapper
        │       ├── rfdetr_detector.py    # RF-DETR wrapper
        │       ├── mock_detector.py      # Synthetic detections for sim
        │       └── synthetic_camera.py   # Fake camera for testing
        ├── tracking_geometry/      # Yaw error computation
        ├── offboard_control/       # PX4 offboard interface
        └── msg/                    # Custom message definitions
```

## Development Environment

### Target Platforms
- **Development**: Ubuntu 22.04 (x86_64) with ROS2 Humble
- **Deployment**: Jetson Orin Nano 16GB (aarch64, `airhound` user) with JetPack 6.2.1, ROS2 Humble
- **Flight Controller**: Auterion PX4 FMU v6X.x, Ethernet IP `192.168.0.4`
- **Companion**: Jetson `eno1` at `192.168.0.3` (connected via Auterion baseboard Ethernet)
- **HITL**: See `docs/HITL_SETUP.md` for full procedure

### Key Dependencies
- ROS2 Humble
- Python 3.10+
- PyTorch (for model inference)
- TensorRT (for optimized inference on Jetson)
- OpenCV
- rfdetr package (`pip install rfdetr`)
- ultralytics package (`pip install ultralytics`)

### Building the Workspace

```bash
cd ws_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**IMPORTANT**: Do NOT search or grep in `ws_ros2/install/`, `ws_ros2/build/`, or `ws_ros2/log/` directories - they contain generated files and will cause token/memory issues.

## Coding Standards

### Python Style
- Follow PEP 8
- Use type hints for function signatures
- Docstrings: Google style
- Max line length: 100 characters

### ROS2 Conventions
- Node names: `snake_case` (e.g., `perception_node`)
- Topic names: `snake_case` with `/` separators (e.g., `/camera/color/image_raw`)
- Parameter names: `snake_case`
- Use QoS profiles appropriate for sensor data (BEST_EFFORT for cameras)

### File Naming
- Python modules: `snake_case.py`
- Launch files: `descriptive_name.launch.py`
- Config files: `snake_case.yaml`

## Key Components

### 1. Detector Wrappers (`airhound_perception`)

All detectors must implement a common interface:

```python
class Detection:
    """Detection result container."""
    xyxy: Tuple[float, float, float, float]  # Bounding box (x1, y1, x2, y2)
    conf: float                               # Confidence score
    cls: int                                  # Class ID
    label: str                                # Class name

class BaseDetector:
    """Base interface for detectors."""
    def infer(self, image_bgr: np.ndarray) -> List[Detection]:
        """Run inference on BGR image, return detections."""
        raise NotImplementedError
```

#### YOLOv8 Detector (`yolo_detector.py`)
- Uses Ultralytics YOLO
- Supports `.pt` (PyTorch) and `.engine` (TensorRT)
- Auto-prefers TensorRT if `.engine` exists alongside `.pt`

#### RF-DETR Detector (`rfdetr_detector.py`)
- Uses `rfdetr` package
- Supports `.pth` (PyTorch), `.onnx`, and `.engine` (TensorRT)
- Requires TensorRT conversion ON the Jetson for `.engine` files

### 2. Perception Node (`detector_node.py`)

Main ROS2 node that:
- Subscribes to camera images (raw or compressed)
- Runs detection inference
- Publishes `Detection2DArray` messages
- Publishes latency/FPS metrics

**Parameters:**
- `detector_type`: "yolo" or "rfdetr"
- `model_path`: Path to model file
- `confidence_threshold`: Detection confidence threshold
- `device`: "0" for GPU, "cpu" for CPU

### 3. Configuration (`config/airhound.yaml`)

Unified config file for all nodes. Key sections:
- `mode`: "sim" or "flight"
- `perception`: Detector settings (model path, thresholds)
- `tracking`: Yaw control parameters
- `px4`: Offboard control settings

## Common Tasks

### Adding a New Detector

1. Create `new_detector.py` in `ws_ros2/src/airhound_perception/airhound_perception/`
2. Implement the `Detection` class and detector wrapper
3. Update `detector_node.py` to support the new detector type
4. Add configuration options to `config/airhound.yaml`
5. Update this AGENTS.md with the new detector info

### Testing on Jetson

1. Build the workspace: `colcon build --symlink-install`
2. Source: `source install/setup.bash`
3. Run simulation mode: `./launch_airhound.sh sim`
4. Run flight mode: `./launch_airhound.sh flight`

### Converting Models to TensorRT

**Must be done ON the Jetson** (engines are platform-specific):

```bash
# For ONNX models
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16 --workspace=4096

# For RF-DETR specifically
python -c "
from rf_detr_drone.export import export_to_tensorrt
from pathlib import Path
export_to_tensorrt(Path('weights/checkpoint_best_ema.pth'), fp16=True)
"
```

## Warnings and Pitfalls

1. **DO NOT** search/grep in build directories (`install/`, `build/`, `log/`) - causes OOM
2. **TensorRT engines are platform-specific** - must regenerate on each platform
3. **px4_msgs must match PX4 version** - sync with PX4-Autopilot if issues arise
4. **Camera topic namespace** - `realsense2_camera` publishes under `/camera/camera/` (nested),
   not `/camera/`. Actual topics: `/camera/camera/color/image_raw`, `/camera/camera/depth/image_raw`.
   Scripts checking `/camera/color/image_raw` will get a false negative.
5. **Jetson power mode matters** - use `sudo nvpmodel -m 0` for max performance
6. **ModemManager conflicts** - on any machine with `/dev/ttyACM0` (Pixhawk USB), stop ModemManager
   before using `mavlink_shell.py`: `sudo systemctl stop ModemManager`
7. **PX4 MAVLink UDP is unicast** - PX4 only responds after receiving a packet from the GCS.
   Always send at least one heartbeat to `192.168.0.4:14550` before expecting replies.
8. **fmu-v6x Ethernet config** - uses `netman` + `/fs/microsd/net.cfg`, NOT `NET_IP0`–`NET_IP3`
   parameters. See `docs/HITL_SETUP.md` for details.

## Testing

### Unit Tests
```bash
cd ws_ros2
colcon test --packages-select airhound_perception
colcon test-result --verbose
```

### Integration Tests
```bash
# Run E2E simulation with mock detector
./launch_airhound.sh sim

# In another terminal, check topics
ros2 topic list
ros2 topic echo /detections
ros2 topic echo /yaw_command
```

## Contact

- Repository: https://github.com/eppl-erau-db/AIRHOUND
- Maintainer: EPPL Lab, Embry-Riddle Aeronautical University
