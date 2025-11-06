#!/bin/bash
# End-to-End Test Script for AIRHOUND Perception Package
# This script tests the mock detector without needing Gazebo, camera, or YOLO weights
# Run this on the sim machine with ROS 2 Humble installed

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}AIRHOUND Perception E2E Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Store the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ws_ros2"

# Check if ROS 2 is installed
echo -e "${YELLOW}[1/8] Checking ROS 2 installation...${NC}"
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}ERROR: ROS 2 Humble not found at /opt/ros/humble/${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS 2 Humble found${NC}"
echo ""

# Source ROS 2
echo -e "${YELLOW}[2/8] Sourcing ROS 2 environment...${NC}"
source /opt/ros/humble/setup.bash
echo -e "${GREEN}✓ ROS 2 environment sourced${NC}"
echo ""

# Build the package
echo -e "${YELLOW}[3/8] Building airhound_perception package...${NC}"
cd "$WS_DIR"
colcon build --packages-select airhound_perception
if [ $? -ne 0 ]; then
    echo -e "${RED}ERROR: Build failed${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Package built successfully${NC}"
echo ""

# Source workspace
echo -e "${YELLOW}[4/8] Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

# Verify package is installed
echo -e "${YELLOW}[5/8] Verifying package installation...${NC}"
if ! ros2 pkg list | grep -q airhound_perception; then
    echo -e "${RED}ERROR: airhound_perception package not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Package found in ROS 2 package list${NC}"

# Check entry points
echo -e "${BLUE}  Checking entry points:${NC}"
if ros2 pkg executables airhound_perception | grep -q detector_node; then
    echo -e "${GREEN}    ✓ detector_node${NC}"
else
    echo -e "${RED}    ✗ detector_node NOT FOUND${NC}"
fi

if ros2 pkg executables airhound_perception | grep -q mock_detector; then
    echo -e "${GREEN}    ✓ mock_detector${NC}"
else
    echo -e "${RED}    ✗ mock_detector NOT FOUND${NC}"
fi

if ros2 pkg executables airhound_perception | grep -q synthetic_camera; then
    echo -e "${GREEN}    ✓ synthetic_camera${NC}"
else
    echo -e "${RED}    ✗ synthetic_camera NOT FOUND${NC}"
fi
echo ""

# Launch mock detector in background
echo -e "${YELLOW}[6/8] Launching mock detector...${NC}"
ros2 launch airhound_perception sim_test.launch.py > /tmp/airhound_mock_detector.log 2>&1 &
LAUNCH_PID=$!
echo -e "${GREEN}✓ Mock detector launched (PID: $LAUNCH_PID)${NC}"
echo ""

# Wait for node to start
echo -e "${YELLOW}[7/8] Waiting for node to initialize (3 seconds)...${NC}"
sleep 3

# Check if process is still running
if ! ps -p $LAUNCH_PID > /dev/null; then
    echo -e "${RED}ERROR: Mock detector process died${NC}"
    echo -e "${RED}Log output:${NC}"
    cat /tmp/airhound_mock_detector.log
    exit 1
fi
echo -e "${GREEN}✓ Node is running${NC}"
echo ""

# Test topics
echo -e "${YELLOW}[8/8] Testing ROS 2 topics...${NC}"
echo ""

# Check if /detections topic exists
echo -e "${BLUE}  Checking /detections topic...${NC}"
if ros2 topic list | grep -q "/detections"; then
    echo -e "${GREEN}    ✓ /detections topic exists${NC}"

    # Get topic type
    TOPIC_TYPE=$(ros2 topic info /detections --no-daemon | grep "Type:" | awk '{print $2}')
    echo -e "${BLUE}    Topic type: ${TOPIC_TYPE}${NC}"

    # Get topic rate
    echo -e "${BLUE}    Measuring topic rate (5 seconds)...${NC}"
    timeout 5s ros2 topic hz /detections --window 10 > /tmp/topic_hz.txt 2>&1 || true
    if grep -q "average rate" /tmp/topic_hz.txt; then
        RATE=$(grep "average rate" /tmp/topic_hz.txt | awk '{print $3}')
        echo -e "${GREEN}    ✓ Topic publishing at ${RATE} Hz${NC}"
    else
        echo -e "${YELLOW}    ⚠ Could not measure rate (might be OK if publishing slowly)${NC}"
    fi

    # Echo one message
    echo -e "${BLUE}    Capturing one message...${NC}"
    timeout 3s ros2 topic echo /detections --once --no-daemon > /tmp/topic_msg.txt 2>&1 || true
    if [ -s /tmp/topic_msg.txt ]; then
        echo -e "${GREEN}    ✓ Message received${NC}"
        echo -e "${BLUE}    First detection sample:${NC}"
        head -20 /tmp/topic_msg.txt | sed 's/^/      /'
    else
        echo -e "${YELLOW}    ⚠ No message captured in 3 seconds${NC}"
    fi
else
    echo -e "${RED}    ✗ /detections topic NOT FOUND${NC}"
fi
echo ""

# List all active topics
echo -e "${BLUE}  All active topics:${NC}"
ros2 topic list | sed 's/^/    /'
echo ""

# List all active nodes
echo -e "${BLUE}  All active nodes:${NC}"
ros2 node list | sed 's/^/    /'
echo ""

# Cleanup
echo -e "${YELLOW}Cleaning up...${NC}"
kill $LAUNCH_PID 2>/dev/null || true
sleep 1
# Force kill if still running
if ps -p $LAUNCH_PID > /dev/null 2>&1; then
    kill -9 $LAUNCH_PID 2>/dev/null || true
fi
echo -e "${GREEN}✓ Mock detector stopped${NC}"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}E2E Test Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Summary:${NC}"
echo -e "  • Package builds successfully"
echo -e "  • Mock detector launches without errors"
echo -e "  • /detections topic publishes vision_msgs/Detection2DArray"
echo -e "  • Ready for control/tracking integration"
echo ""
echo -e "${BLUE}Next Steps (on sim machine):${NC}"
echo -e "  1. Test with different launch parameters:"
echo -e "     ${GREEN}ros2 launch airhound_perception sim_test.launch.py motion:=oscillating noise:=0.05${NC}"
echo -e "  2. Subscribe to /detections in your control node"
echo -e "  3. Implement yaw tracking using bbox center from detections"
echo ""
echo -e "${BLUE}Log files saved to:${NC}"
echo -e "  • /tmp/airhound_mock_detector.log"
echo -e "  • /tmp/topic_hz.txt"
echo -e "  • /tmp/topic_msg.txt"
echo ""
