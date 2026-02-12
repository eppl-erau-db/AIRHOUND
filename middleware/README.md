# Airhound - PX4 ROS2 Yaw Control System

A ROS2-based system for controlling PX4 drone yaw commands in simulation. This project provides a clean interface for publishing yaw commands that get converted to PX4 offboard control messages.

## System Overview

```
Demo Publisher → /yaw_command → PX4 Converter → MicroXRCE Agent → PX4 SITL (Gazebo)
```

**Components:**
- **demo_publisher_enhanced**: Publishes yaw commands (Float64) to `/yaw_command`
- **px4_converter_gazebo**: Converts yaw commands (/yaw_command Float64) to PX4 offboard control messages
- **MicroXRCE Agent**: Bridges ROS2 DDS with PX4's uXRCE-DDS client
- **PX4 SITL**: PX4 autopilot running in Gazebo simulation

## Prerequisites

- Ubuntu 22.04 or later
- ROS2 (Jazzy or Humble)
- Python 3.10+
- Git

## Fresh Installation (New Machine)

### 1. Install ROS2

**For ROS2 Jazzy (Ubuntu 22.04+):**
```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install build tools
sudo apt install python3-colcon-common-extensions -y

# Source ROS2 (add to ~/.bashrc for persistence)
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

**For ROS2 Humble (Ubuntu 22.04):**
```bash
# Follow official installation: https://docs.ros.org/en/humble/Installation.html
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install python3-colcon-common-extensions -y

# Source ROS2
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2. Install PX4 Dependencies

```bash
cd /home/lucca/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

### 3. Install Project Dependencies

```bash
cd /home/lucca/PX4-Autopilot/airhound
./setup_dependencies.sh
```

This script installs:
- MicroXRCE-DDS Agent (ROS2 ↔ PX4 bridge)
- Additional ROS2 packages
- Python dependencies

### 4. Build ROS2 Workspace

```bash
cd DDS_to_PX4_middleware/workspace
colcon build --packages-select px4_msgs
colcon build --packages-select offboard_control
source install/setup.bash
```

**Add to ~/.bashrc for convenience:**
```bash
echo "source ~/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace/install/setup.bash" >> ~/.bashrc
```

### 5. Build PX4 SITL

```bash
cd /home/lucca/PX4-Autopilot
make px4_sitl
```

## Running the System

You need **4 separate terminals** to run the complete system.

### Terminal 1: Start PX4 SITL + Gazebo

```bash
cd /home/lucca/PX4-Autopilot
make px4_sitl gz_x500
```

Wait for Gazebo to load and show "Commander: Ready for takeoff!"

### Terminal 2: Start MicroXRCE Agent

```bash
cd /home/lucca/PX4-Autopilot/airhound
./start_microxrce_agent.sh
```

You should see:
```
[INFO] MicroXRCE Agent running...
```

### Terminal 3: Start PX4 Converter Node

```bash
cd /home/lucca/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace
source install/setup.bash
ros2 run offboard_control px4_converter_gazebo
```

You should see:
```
[INFO] PX4 Converter Node (Python) started
[INFO] Publishing at 10.0 Hz, auto_arm: True
```

### Terminal 4: Start Demo Publisher

```bash
cd /home/lucca/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace
source install/setup.bash
ros2 run offboard_control demo_publisher_enhanced
```

**Expected Behavior:**
1. After ~1 second, the drone arms automatically
2. Drone takes off and hovers at 5 meters altitude
3. Drone rotates through test yaw angles
4. You'll see log messages showing yaw conversions

## Demo Publisher Parameters

The demo publisher is highly customizable via ROS2 parameters:

```bash
ros2 run offboard_control demo_publisher_enhanced --ros-args \
  -p test_mode:=cardinals \
  -p publish_rate:=5.0 \
  -p enable_cycling:=false \
  -p start_delay:=5.0 \
  -p test_duration:=60
```

**Available Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `test_mode` | string | `"full"` | Test sequence: `"full"`, `"basic"`, or `"cardinals"` |
| `publish_rate` | float | `2.0` | Publishing frequency in Hz |
| `enable_cycling` | bool | `true` | Loop the sequence after completion |
| `start_delay` | float | `0.0` | Delay in seconds before starting |
| `test_duration` | int | `30` | Total test duration in seconds |

**Test Modes:**
- **`cardinals`**: North, East, South, West (5 positions)
- **`basic`**: Cardinals + diagonals (9 positions)
- **`full`**: Comprehensive test with micro-movements (25 positions)

## Custom Yaw Publisher

To create your own yaw command publisher, publish Float64 messages to `/yaw_command`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class MyYawPublisher(Node):
    def __init__(self):
        super().__init__('my_yaw_publisher')
        self.pub = self.create_publisher(Float64, '/yaw_command', 10)
        self.timer = self.create_timer(0.5, self.publish_yaw)
        self.yaw = 0.0

    def publish_yaw(self):
        msg = Float64()
        msg.data = self.yaw  # radians, range [-π, π]
        self.pub.publish(msg)
        self.yaw += 0.1

def main():
    rclpy.init()
    node = MyYawPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting

### "MicroXRCEAgent not found"
Run the setup script:
```bash
cd /home/lucca/PX4-Autopilot/airhound
./setup_dependencies.sh
```

### "No module named 'px4_msgs'"
Build the workspace:
```bash
cd /home/lucca/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace
colcon build
source install/setup.bash
```

### Drone doesn't respond
1. Check MicroXRCE Agent is running (Terminal 2 should show connection messages)
2. Verify PX4 shows "Commander: Ready for takeoff!"
3. Ensure px4_converter_gazebo is publishing messages (check Terminal 3)
4. Verify demo publisher is running (check Terminal 4)

### "Package not found" errors
Make sure you sourced both ROS2 and the workspace:
```bash
# For Jazzy:
source /opt/ros/jazzy/setup.bash
# For Humble:
source /opt/ros/humble/setup.bash

# Then source the workspace:
source ~/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace/install/setup.bash
```

## Project Structure

```
airhound/
├── README.md                           # This file
├── setup_dependencies.sh               # Install dependencies
├── start_microxrce_agent.sh           # Start DDS bridge
├── DDS_to_PX4_middleware/
│   └── workspace/
│       └── src/
│           ├── px4_msgs/               # PX4 message definitions
│           └── offboard_control/
│               ├── CMakeLists.txt
│               ├── package.xml
│               └── src/
│                   ├── demo_publisher_enhanced.cpp        # Customizable yaw publisher
│                   ├── px4_converter_gazebo.cpp           # Yaw → PX4 converter (sim)
│                   └── dummy_yaw_publisher.cpp           # Simple test publisher
└── PX4-Autopilot/                     # PX4 firmware (SITL)
```

## Development

### Rebuilding After Code Changes

After editing C++ files:
```bash
cd /home/lucca/PX4-Autopilot/airhound/DDS_to_PX4_middleware/workspace
colcon build --packages-select offboard_control
source install/setup.bash
```

C++ nodes require rebuilding with `colcon build` after changes.

### Testing Without PX4

You can test the demo publisher without PX4:
```bash
ros2 run offboard_control demo_publisher_enhanced

# In another terminal, watch the output:
ros2 topic echo /yaw_command
```

## References

- [PX4 Documentation](https://docs.px4.io/)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [PX4-ROS2 Bridge](https://docs.px4.io/main/en/ros/ros2_comm.html)
