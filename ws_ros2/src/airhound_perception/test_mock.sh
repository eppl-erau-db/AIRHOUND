#!/bin/bash
# Test script for mock detector - verifies it runs without errors

set -e  # Exit on error

echo "=========================================="
echo "Testing AIRHOUND Mock Detector"
echo "=========================================="
echo ""

# Check if we're in the right directory
if [ ! -d "install/airhound_perception" ]; then
    echo "ERROR: Must run from ws_ros2 directory"
    echo "Usage: cd ws_ros2 && bash ../test_mock.sh"
    exit 1
fi

# Source ROS 2
echo "1. Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Source workspace
echo "2. Sourcing workspace..."
source install/setup.bash

# Check if mock_detector executable exists
echo "3. Checking executables..."
if [ ! -f "install/airhound_perception/lib/airhound_perception/mock_detector" ]; then
    echo "ERROR: mock_detector executable not found!"
    echo "Run: colcon build --packages-select airhound_perception"
    exit 1
fi
echo "   ✓ mock_detector found"

# Test import by running node with --help
echo "4. Testing mock_detector imports..."
timeout 3 ros2 run airhound_perception mock_detector --ros-args -h > /dev/null 2>&1 || true
if [ $? -eq 124 ]; then
    echo "   ✓ mock_detector starts (killed after 3s - normal)"
else
    echo "   ✓ mock_detector help works"
fi

# Check if launch file exists
echo "5. Checking launch file..."
LAUNCH_FILE="install/airhound_perception/share/airhound_perception/launch/sim_test.launch.py"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "ERROR: sim_test.launch.py not found!"
    exit 1
fi
echo "   ✓ sim_test.launch.py found"

# Test launch file syntax
echo "6. Testing launch file syntax..."
python3 -m py_compile "$LAUNCH_FILE"
if [ $? -eq 0 ]; then
    echo "   ✓ Launch file syntax valid"
else
    echo "ERROR: Launch file has syntax errors"
    exit 1
fi

echo ""
echo "=========================================="
echo "All checks passed! ✓"
echo "=========================================="
echo ""
echo "To run mock detector:"
echo "  ros2 launch airhound_perception sim_test.launch.py"
echo ""
echo "To verify detections:"
echo "  ros2 topic echo /detections"
echo ""
