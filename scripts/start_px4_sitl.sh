#!/bin/bash
# =============================================================================
# PX4 SITL + Gazebo Starter
# =============================================================================
# Helper script to start PX4 SITL simulation with Gazebo
#
# Usage:
#   ./scripts/start_px4_sitl.sh                    # Start with defaults
#   ./scripts/start_px4_sitl.sh --target gz_x500   # Specific vehicle
#   ./scripts/start_px4_sitl.sh --headless         # No GUI
#   ./scripts/start_px4_sitl.sh --install          # Install PX4 first
# =============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Defaults
PX4_PATH="${HOME}/PX4-Autopilot"
GAZEBO_TARGET="gz_x500"
HEADLESS=false
INSTALL_PX4=false

# =============================================================================
# Functions
# =============================================================================

print_help() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  --target TARGET   Gazebo target (default: gz_x500)"
    echo "                    Options: gz_x500, gz_x500_depth, gz_x500_vision"
    echo "  --px4-path PATH   PX4-Autopilot path (default: ~/PX4-Autopilot)"
    echo "  --headless        Run without Gazebo GUI"
    echo "  --install         Install PX4-Autopilot if not found"
    echo "  --help            Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                          # Default x500 quadcopter"
    echo "  $0 --target gz_x500_depth   # With depth camera"
    echo "  $0 --headless               # CI/testing mode"
    echo ""
}

install_px4() {
    echo -e "${BLUE}Installing PX4-Autopilot...${NC}"
    echo "This will take 20-30 minutes."
    echo ""
    
    cd "${HOME}"
    
    # Clone if not exists
    if [ ! -d "PX4-Autopilot" ]; then
        echo "Cloning PX4-Autopilot..."
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    else
        echo "PX4-Autopilot directory exists, updating..."
        cd PX4-Autopilot
        git pull
        git submodule update --init --recursive
        cd ..
    fi
    
    cd PX4-Autopilot
    
    # Run setup script
    echo ""
    echo -e "${BLUE}Running PX4 setup script...${NC}"
    echo "This installs required dependencies (may ask for sudo password)"
    bash ./Tools/setup/ubuntu.sh
    
    # Initial build
    echo ""
    echo -e "${BLUE}Building PX4 SITL...${NC}"
    make px4_sitl_default
    
    echo ""
    echo -e "${GREEN}PX4-Autopilot installed successfully!${NC}"
    echo "Location: ${HOME}/PX4-Autopilot"
}

check_px4() {
    if [ ! -d "$PX4_PATH" ]; then
        echo -e "${RED}ERROR: PX4-Autopilot not found at ${PX4_PATH}${NC}"
        echo ""
        
        if [ "$INSTALL_PX4" = true ]; then
            install_px4
        else
            echo "Options:"
            echo "  1. Run with --install flag to install PX4"
            echo "  2. Specify path with --px4-path /your/path"
            echo "  3. Install manually: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html"
            exit 1
        fi
    fi
    
    echo -e "${GREEN}Found PX4-Autopilot at ${PX4_PATH}${NC}"
}

start_sitl() {
    echo -e "${BLUE}Starting PX4 SITL with Gazebo...${NC}"
    echo "Target: ${GAZEBO_TARGET}"
    echo "Headless: ${HEADLESS}"
    echo ""
    
    cd "$PX4_PATH"
    
    # Set headless mode if requested
    if [ "$HEADLESS" = true ]; then
        export HEADLESS=1
        echo "Running in headless mode (no GUI)"
    fi
    
    echo -e "${YELLOW}Starting PX4 SITL...${NC}"
    echo "Wait for 'Ready for takeoff!' message"
    echo "Press Ctrl+C to stop"
    echo ""
    
    # Start PX4 SITL
    make px4_sitl "$GAZEBO_TARGET"
}

# =============================================================================
# Main
# =============================================================================

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --target)
            GAZEBO_TARGET="$2"
            shift 2
            ;;
        --px4-path)
            PX4_PATH="$2"
            shift 2
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --install)
            INSTALL_PX4=true
            shift
            ;;
        --help|-h)
            print_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_help
            exit 1
            ;;
    esac
done

echo "=============================================="
echo "     PX4 SITL + Gazebo Starter"
echo "=============================================="
echo ""

# Check/install PX4
check_px4

# Start SITL
start_sitl
