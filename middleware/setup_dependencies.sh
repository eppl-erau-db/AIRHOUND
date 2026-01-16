#!/bin/bash
# Setup script for Airhound dependencies
# Installs MicroXRCE-DDS Agent and other required tools

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Airhound Dependencies Setup${NC}"
echo -e "${GREEN}========================================${NC}"

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}Please do not run this script as root${NC}"
    exit 1
fi

# Install MicroXRCE-DDS Agent
install_microxrce_agent() {
    echo -e "${YELLOW}Installing MicroXRCE-DDS Agent...${NC}"

    # Create directory for tools
    TOOLS_DIR="$HOME/.local/airhound_tools"
    mkdir -p "$TOOLS_DIR"
    cd "$TOOLS_DIR"

    # Clone and build MicroXRCE-DDS-Agent
    if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
        echo "Cloning Micro-XRCE-DDS-Agent..."
        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    fi

    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build

    echo "Building MicroXRCE-DDS Agent..."
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/

    echo -e "${GREEN}✓ MicroXRCE-DDS Agent installed${NC}"
}

# Check if MicroXRCE-DDS Agent is already installed
if command -v MicroXRCEAgent &> /dev/null; then
    echo -e "${GREEN}✓ MicroXRCE-DDS Agent already installed${NC}"
else
    install_microxrce_agent
fi

# Check ROS2 installation
echo ""
if command -v ros2 &> /dev/null; then
    echo -e "${GREEN}✓ ROS2 is installed${NC}"
    ros2 --version
else
    echo -e "${YELLOW}ROS2 not found.${NC}"
    echo "Please install ROS2 (Jazzy or Humble):"
    echo "https://docs.ros.org/en/jazzy/Installation.html"
fi

# Build the workspace
echo ""
echo -e "${YELLOW}Building ROS2 workspace...${NC}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR/DDS_to_PX4_middleware/workspace"

# Source ROS2 if available (try multiple distros)
ROS2_SOURCED=false
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ Sourced ROS2 Jazzy${NC}"
    ROS2_SOURCED=true
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ Sourced ROS2 Humble${NC}"
    ROS2_SOURCED=true
fi

if [ "$ROS2_SOURCED" = true ]; then
    cd "$WORKSPACE_DIR"
    echo "Building px4_msgs package..."
    colcon build --packages-select px4_msgs
    echo "Building offboard_control package..."
    colcon build --packages-select offboard_control
    echo -e "${GREEN}✓ Workspace built successfully${NC}"
else
    echo -e "${YELLOW}Skipping workspace build - ROS2 not available${NC}"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Setup Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "To add the workspace to your shell automatically:"
echo -e "  ${GREEN}echo 'source $WORKSPACE_DIR/install/setup.bash' >> ~/.bashrc${NC}"
echo ""
echo "See README.md for full instructions on running the system."
echo ""
