#!/bin/bash
# E2E Integration Test Script for integration/roles-1-2-3-e2e branch
# Run this on the sim computer to test all 3 roles

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="${SCRIPT_DIR}/ws_ros2"

echo "============================================"
echo "E2E Integration Test - Roles 1-3"
echo "Branch: integration/roles-1-2-3-e2e"
echo "============================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're on the right branch
CURRENT_BRANCH=$(git -C "${SCRIPT_DIR}" branch --show-current)
if [ "$CURRENT_BRANCH" != "integration/roles-1-2-3-e2e" ]; then
    echo -e "${YELLOW}Warning: Not on integration/roles-1-2-3-e2e branch!${NC}"
    echo "Current branch: $CURRENT_BRANCH"
    echo "Switch with: git checkout integration/roles-1-2-3-e2e"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Source ROS 2
echo -e "${GREEN}[1/5] Sourcing ROS 2...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "  ✓ ROS 2 Humble sourced"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "  ✓ ROS 2 Jazzy sourced"
else
    echo -e "${RED}  ✗ ROS 2 not found! Install ROS 2 first.${NC}"
    exit 1
fi

# Build workspace
echo -e "${GREEN}[2/5] Building workspace...${NC}"
cd "${WS_DIR}"

# Clean build if requested
if [ "$1" == "--clean" ]; then
    echo "  Cleaning previous build..."
    rm -rf build/ install/ log/
fi

echo "  Building Tracking-Geometry, airhound_perception, offboard_control..."
colcon build --packages-select Tracking-Geometry airhound_perception offboard_control

if [ $? -ne 0 ]; then
    echo -e "${RED}  ✗ Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}  ✓ Build successful${NC}"

# Source workspace
echo -e "${GREEN}[3/5] Sourcing workspace...${NC}"
source install/setup.bash
echo "  ✓ Workspace sourced"

# Check nodes are available
echo -e "${GREEN}[4/5] Checking node executables...${NC}"

# Check tracking node
if ! ros2 pkg executables Tracking-Geometry | grep -q "tracking_node"; then
    echo -e "${RED}  ✗ tracking_node not found!${NC}"
    echo "  Available executables:"
    ros2 pkg executables Tracking-Geometry
    exit 1
fi
echo "  ✓ tracking_node found"

# Check mock detector
if ! ros2 pkg executables airhound_perception | grep -q "mock_detector"; then
    echo -e "${RED}  ✗ mock_detector not found!${NC}"
    echo "  Available executables:"
    ros2 pkg executables airhound_perception
    exit 1
fi
echo "  ✓ mock_detector found"

# Check offboard nodes
if ! ros2 pkg executables offboard_control | grep -q "px4_converter_node_simple"; then
    echo -e "${RED}  ✗ px4_converter_node_simple not found!${NC}"
    echo "  Available executables:"
    ros2 pkg executables offboard_control
    exit 1
fi
echo "  ✓ px4_converter_node_simple found"

# Start nodes (Option A: No-sim test)
echo -e "${GREEN}[5/5] Starting nodes (no-sim test)...${NC}"
echo ""
echo "Starting 3-node pipeline in separate terminals using tmux..."
echo "Press Ctrl+C to stop all nodes and exit."
echo ""

# Check if tmux is available
if ! command -v tmux &> /dev/null; then
    echo -e "${YELLOW}tmux not installed. Install with: sudo apt install tmux${NC}"
    echo ""
    echo "Alternatively, manually run these commands in separate terminals:"
    echo ""
    echo "Terminal 1 (Mock Detector):"
    echo "  cd ${WS_DIR} && source install/setup.bash"
    echo "  ros2 run airhound_perception mock_detector"
    echo ""
    echo "Terminal 2 (Tracking Node):"
    echo "  cd ${WS_DIR} && source install/setup.bash"
    echo "  ros2 run Tracking-Geometry tracking_node"
    echo ""
    echo "Terminal 3 (Offboard Converter):"
    echo "  cd ${WS_DIR} && source install/setup.bash"
    echo "  ros2 run offboard_control px4_converter_node_simple"
    echo ""
    echo "Terminal 4 (Monitor):"
    echo "  ros2 topic list"
    echo "  ros2 topic echo /detections"
    echo "  ros2 topic echo /target_yaw"
    exit 0
fi

# Create tmux session
SESSION_NAME="airhound_e2e_test"

# Kill existing session if it exists
tmux kill-session -t ${SESSION_NAME} 2>/dev/null || true

# Create new session
tmux new-session -d -s ${SESSION_NAME} -n "E2E Test"

# Window 0: Mock Detector
tmux send-keys -t ${SESSION_NAME}:0 "cd ${WS_DIR} && source install/setup.bash" C-m
tmux send-keys -t ${SESSION_NAME}:0 "echo '=== Mock Detector (Role 1) ===' && sleep 1" C-m
tmux send-keys -t ${SESSION_NAME}:0 "ros2 run airhound_perception mock_detector" C-m

# Window 1: Tracking Node
tmux new-window -t ${SESSION_NAME}:1 -n "Tracking"
tmux send-keys -t ${SESSION_NAME}:1 "cd ${WS_DIR} && source install/setup.bash" C-m
tmux send-keys -t ${SESSION_NAME}:1 "echo '=== Tracking Node (Role 2) ===' && sleep 2" C-m
tmux send-keys -t ${SESSION_NAME}:1 "ros2 run Tracking-Geometry tracking_node" C-m

# Window 2: Offboard Converter
tmux new-window -t ${SESSION_NAME}:2 -n "Offboard"
tmux send-keys -t ${SESSION_NAME}:2 "cd ${WS_DIR} && source install/setup.bash" C-m
tmux send-keys -t ${SESSION_NAME}:2 "echo '=== Offboard Converter (Role 3) ===' && sleep 3" C-m
tmux send-keys -t ${SESSION_NAME}:2 "ros2 run offboard_control px4_converter_node_simple" C-m

# Window 3: Monitor
tmux new-window -t ${SESSION_NAME}:3 -n "Monitor"
tmux send-keys -t ${SESSION_NAME}:3 "cd ${WS_DIR} && source install/setup.bash" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '=== Topic Monitor ===' && sleep 4" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo 'Waiting for nodes to start...' && sleep 3" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '' && echo 'Active topics:' && ros2 topic list" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '' && echo 'Checking /detections:' && timeout 2 ros2 topic hz /detections || echo 'No messages on /detections'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '' && echo 'Checking /target_yaw:' && timeout 2 ros2 topic hz /target_yaw || echo 'No messages on /target_yaw'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '' && echo 'Sample /target_yaw message:' && timeout 2 ros2 topic echo /target_yaw --once || echo 'No message received'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo '' && echo '--- Available commands ---'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo 'ros2 topic list           - List all topics'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo 'ros2 topic echo /target_yaw - Watch target yaw'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo 'ros2 node list            - List all nodes'" C-m
tmux send-keys -t ${SESSION_NAME}:3 "echo 'ros2 topic hz /detections - Check detection rate'" C-m

# Select first window
tmux select-window -t ${SESSION_NAME}:0

echo ""
echo -e "${GREEN}✓ All nodes started in tmux session: ${SESSION_NAME}${NC}"
echo ""
echo "Tmux commands:"
echo "  - Switch windows: Ctrl+b then 0/1/2/3"
echo "  - Scroll in window: Ctrl+b then [ (q to exit scroll mode)"
echo "  - Detach from session: Ctrl+b then d"
echo "  - Kill all nodes: tmux kill-session -t ${SESSION_NAME}"
echo ""
echo "Attaching to tmux session in 3 seconds..."
sleep 3

# Attach to session
tmux attach-session -t ${SESSION_NAME}

# Cleanup on exit
echo ""
echo "Cleaning up..."
tmux kill-session -t ${SESSION_NAME} 2>/dev/null || true
echo "Done!"
