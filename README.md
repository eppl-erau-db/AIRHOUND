# DDS to PX4 Middleware

A ROS2-based middleware for communicating with PX4 autopilot via ethernet/DDS. This middleware enables yaw control commands and can be extended for full offboard control.

## Architecture

The middleware consists of three main components:

1. **Yaw Controller Node** (`yaw_controller_node.cpp`) - Main middleware that:
   - Receives yaw commands via `/yaw_command` topic
   - Sends PX4 offboard control mode commands
   - Sends trajectory setpoints to PX4
   - Handles arming/disarming

2. **Dummy Yaw Publisher** (`dummy_yaw_publisher.cpp`) - Test component that:
   - Publishes simulated yaw commands (90° every 500ms)
   - Used for testing without external input

3. **PX4 Simulator** (`px4_simulator.cpp`) - Mock PX4 that:
   - Simulates PX4 responses
   - Publishes vehicle status and odometry
   - Used for testing without real hardware

## Quick Start - Docker Testing

### Prerequisites
- Docker and Docker Compose
- macOS/Linux system

### 1. Build and Run Tests

```bash
# Build the middleware in Docker
./docker_test.sh 1

# Run the simulation test
./docker_test.sh 2

# Stop containers when done
./docker_test.sh 3
```

### 2. Expected Output

When running the simulation test, you should see:
```
[dummy_yaw_publisher-1] [INFO] DummyYawPublisher has been started.
[simple_test_node-2] [INFO] SimpleTestNode started - testing middleware communication
[simple_test_node-2] [INFO] Received yaw command: 1.57 radians (90.0 degrees)
[simple_test_node-2] [INFO] Published: Middleware active - Last yaw: 1.570000 rad
```

This confirms the middleware is receiving and processing yaw commands correctly.

## Testing Options

### Option 1: Simple Test (No PX4 Dependencies)
- Uses `simple_test.launch.py`
- Tests basic ROS2 communication
- Good for verifying middleware connectivity

### Option 2: Full PX4 Simulation Test
- Uses `test_with_dummy.launch.py`  
- Includes PX4 message simulation
- Requires `px4_msgs` package

### Option 3: Local Test (No Docker)
- Uses `test_local_no_px4.launch.py`
- Run directly with `./test_middleware.sh`
- Requires local ROS2 Humble installation

## Real Hardware Setup

### 1. Install PX4 Messages Package

The middleware requires the `px4_msgs` package for real PX4 communication:

```bash
# Option A: Install from source
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs

# Option B: Check if available in your ROS distribution
sudo apt install ros-humble-px4-msgs
```

### 2. Ethernet Configuration

Follow the setup in `ETHERNET_SETUP.md` to configure ethernet communication between your computer and PX4.

Key steps:
- Configure static IP on your computer (e.g., 192.168.1.100)
- Configure PX4 ethernet parameters
- Ensure network connectivity

### 3. Build with PX4 Support

Replace the simplified CMakeLists.txt with the full version:

```bash
cp CMakeLists.txt CMakeLists_full.txt  # backup current
# Edit CMakeLists.txt to include px4_msgs dependency
colcon build
```

### 4. Run with Real Hardware

```bash
# Source your workspace
source install/setup.bash

# Launch the full middleware (adjust launch file as needed)
ros2 launch offboard_control offboard_yaw_demo.launch.py
```

## Configuration Files

- `config/px4_params.yaml` - PX4 parameter recommendations for safe offboard operation
- `config/ethernet_config.yaml` - Network configuration (currently empty, add your settings)

## Development and Debugging

### Monitor Topics

```bash
# List active topics
ros2 topic list

# Monitor yaw commands
ros2 topic echo /yaw_command

# Monitor PX4 vehicle status
ros2 topic echo /fmu/out/vehicle_status

# Send manual yaw command
ros2 topic pub /yaw_command std_msgs/msg/Float64 "{data: 1.57}"
```

### Logs and Debugging

- Docker logs: `docker logs px4_ros2_dev`
- ROS logs: Check `~/.ros/log/` directory
- Enable debug logging in launch files with `output='screen'`

## Next Steps for Real Hardware

1. **Install px4_msgs Package**: Essential for PX4 communication
2. **Network Setup**: Configure ethernet connection per `ETHERNET_SETUP.md`
3. **PX4 Configuration**: Set required parameters from `config/px4_params.yaml`
4. **Safety Testing**: Start with simulation, then tethered testing
5. **Extend Functionality**: Add position control, waypoint navigation, etc.

## Troubleshooting

### Common Issues

**Docker Build Fails**:
- Clean macOS metadata files: `find . -name "._*" -delete`
- Restart Docker containers: `./docker_test.sh 3 && ./docker_test.sh 1`

**Permission Errors**:
- Usually caused by macOS metadata files in Docker
- Clean files and rebuild

**No PX4 Messages**:
- For real hardware, install `px4_msgs` package
- For testing, use simplified launch files

**Network Issues**:
- Verify ethernet configuration
- Check PX4 DDS/ROS2 bridge is running
- Confirm IP addresses and firewall settings

## Package Structure

```
├── src/                     # Source code
│   ├── yaw_controller_node.cpp    # Main middleware
│   ├── dummy_yaw_publisher.cpp    # Test input generator
│   ├── px4_simulator.cpp          # Mock PX4
│   └── simple_test_node.cpp       # Simple test node
├── launch/                  # Launch files
│   ├── test_with_dummy.launch.py  # Full simulation test
│   ├── test_local_no_px4.launch.py # Local test
│   ├── simple_test.launch.py      # Simple test
│   └── offboard_yaw_demo.launch.py # Hardware demo
├── config/                  # Configuration files
├── docker-compose.yml       # Docker configuration
├── docker_test.sh          # Docker test script  
├── test_middleware.sh      # Local test script
└── README.md               # This file
```

## Safety Notice

⚠️ **Always test in simulation first!** ⚠️

This middleware can control aircraft. Ensure:
- Proper safety procedures are followed
- Testing in safe environment first
- Emergency stop procedures are in place
- Compliance with local regulations