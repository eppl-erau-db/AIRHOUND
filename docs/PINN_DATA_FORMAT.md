# PINN Training Data Format

This document describes the data format for training the Physics-Informed Neural Network (PINN) used in AIRHOUND's trajectory prediction system.

## Overview

The PINN requires timestamped 3D trajectories with velocity information. Data is extracted from ROS2 rosbags recorded during flight tests using the `scripts/extract_trajectories.py` script.

## CSV Format

The output CSV file contains 7 columns:

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `t` | float | seconds | Time relative to trajectory start (t=0 at first point) |
| `x` | float | meters | X position in camera optical frame |
| `y` | float | meters | Y position in camera optical frame |
| `z` | float | meters | Z position (depth) in camera optical frame |
| `vx` | float | m/s | X velocity |
| `vy` | float | m/s | Y velocity |
| `vz` | float | m/s | Z velocity |

### Example Data

```csv
t,x,y,z,vx,vy,vz
0.000000,0.523400,0.145200,3.245000,0.123456,-0.045678,0.234567
0.033333,0.527512,0.143673,3.252812,0.125123,-0.044234,0.231234
0.066667,0.531789,0.142098,3.260534,0.124890,-0.043567,0.228901
...
```

## Coordinate System

Data is recorded in the **camera optical frame** (standard ROS convention):

```
        Z (depth/forward)
       /
      /
     /_________ X (right)
     |
     |
     | Y (down)
```

- **X-axis**: Points right in the image plane
- **Y-axis**: Points down in the image plane  
- **Z-axis**: Points forward (depth direction)

### Coordinate Origin

The origin is at the camera's optical center (Intel RealSense D455).

### Valid Ranges

For the D455 camera:
- **Z (depth)**: 0.4m - 6.0m (reliable range)
- **X, Y**: Depends on field of view and depth

## Velocity Computation

Velocities are computed using **finite differences** from the position data:

1. **First point**: Forward difference
   ```
   v[0] = (pos[1] - pos[0]) / (t[1] - t[0])
   ```

2. **Interior points**: Central difference (more accurate)
   ```
   v[i] = (pos[i+1] - pos[i-1]) / (t[i+1] - t[i-1])
   ```

3. **Last point**: Backward difference
   ```
   v[-1] = (pos[-1] - pos[-2]) / (t[-1] - t[-2])
   ```

### Velocity Smoothing

For noisy data, apply moving average smoothing with `--smooth-window N`:

```bash
python3 scripts/extract_trajectories.py bag_dir -o traj.csv --smooth-window 5
```

This averages velocities over N neighboring points to reduce noise from position measurement errors.

## Data Requirements for Training

### Minimum Requirements

| Parameter | Minimum | Recommended |
|-----------|---------|-------------|
| Points per trajectory | 10 | 100+ |
| Total training points | 1,000 | 10,000+ |
| Duration per trajectory | 0.5s | 3-10s |
| Sample rate | 10 Hz | 30 Hz |

### Quality Criteria

Good training data should have:

1. **Smooth trajectories**: No teleportation or sudden jumps
2. **Consistent sample rate**: Minimal dropped frames
3. **Varied maneuvers**: Straight flight, turns, acceleration, deceleration
4. **Multiple speeds**: 0.5 - 5.0 m/s typical drone speeds
5. **Different depths**: Cover the operational range (1-6m)

### Data Filtering

The extraction script automatically filters:
- Points with NaN coordinates
- Points outside depth sensor range

Additional filtering may be needed for:
- Occlusions (detection gaps > 0.5s)
- Tracking ID switches
- Extreme velocities (> 10 m/s, likely errors)

## Extraction Script Usage

### Basic Usage

```bash
# Single rosbag
python3 scripts/extract_trajectories.py /path/to/rosbag_dir -o trajectory.csv

# Multiple rosbags (combined)
python3 scripts/extract_trajectories.py bag1 bag2 bag3 -o combined.csv

# With velocity smoothing and statistics
python3 scripts/extract_trajectories.py bag_dir -o traj.csv --smooth-window 5 --stats
```

### Script Options

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output` | (required) | Output CSV file path |
| `--topic` | `/perception/target_3d` | ROS topic with 3D positions |
| `--smooth-window` | 1 | Velocity smoothing window (1=no smoothing) |
| `--min-points` | 10 | Minimum points required |
| `--stats` | false | Print trajectory statistics |

### Alternative Topics

The script searches these topics in order:
1. `/perception/target_3d` (primary)
2. `/target_3d`
3. `/tracking/target_position`

## ROS Message Types

The extraction script supports:

1. **`geometry_msgs/PointStamped`** (preferred)
   ```
   header:
     stamp: {sec, nanosec}
     frame_id: "camera_color_optical_frame"
   point:
     x: float64
     y: float64
     z: float64
   ```

2. **`geometry_msgs/Point`**
   ```
   x: float64
   y: float64
   z: float64
   ```

## File Organization

Recommended directory structure:

```
AIRHOUND/
├── data/
│   ├── rosbags/
│   │   ├── flight_2026-01-20_indoor/
│   │   ├── flight_2026-01-21_outdoor/
│   │   └── ...
│   └── pinn_training/
│       ├── indoor_trajectories.csv
│       ├── outdoor_trajectories.csv
│       └── combined_training.csv
└── scripts/
    └── extract_trajectories.py
```

## Integration with PINN Training

After extracting trajectories, the PINN training pipeline expects:

1. CSV file path passed as training data
2. Physics constraints (gravity, drag model) specified separately
3. Train/validation split handled by training script

Example PINN training invocation (placeholder):
```bash
python3 train_pinn.py \
    --data data/pinn_training/combined_training.csv \
    --epochs 1000 \
    --physics gravity,drag
```

## Troubleshooting

### Common Issues

**No points extracted:**
- Check if the rosbag contains the expected topic
- Use `ros2 bag info <bag_dir>` to list available topics
- Try specifying `--topic` with the correct topic name

**Noisy velocities:**
- Increase `--smooth-window` (try 3, 5, or 7)
- Check if position data has outliers

**Invalid depth values:**
- Ensure depth filtering is enabled in detector_node.py
- Check camera depth quality and lighting conditions

### Validating Data

Print statistics to verify data quality:

```bash
python3 scripts/extract_trajectories.py bag_dir -o traj.csv --stats
```

Expected output:
```
--- Trajectory Statistics ---
  Points:     1523
  Duration:   50.77s
  FPS:        30.0
  X range:    [-1.23, 2.45]m
  Y range:    [-0.89, 1.12]m
  Z range:    [1.50, 5.23]m
  Velocity:   mean=1.45, max=4.23, std=0.89 m/s
```

## References

- Intel RealSense D455 Datasheet: Depth accuracy specifications
- AIRHOUND Perception Interface: `docs/PERCEPTION_INTERFACE.md`
- ROS2 Coordinate Frames: REP-103, REP-105
