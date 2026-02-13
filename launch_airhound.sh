#!/bin/bash
# =============================================================================
# AIRHOUND E2E Launcher
# =============================================================================
# Single entry point for the complete AIRHOUND drone tracking pipeline
#
# Usage:
#   ./launch_airhound.sh sim              # Full simulation mode
#   ./launch_airhound.sh flight           # Real flight mode
#   ./launch_airhound.sh sim --no-px4     # Sim without starting PX4 SITL
#   ./launch_airhound.sh sim --build      # Rebuild workspace first
#   ./launch_airhound.sh --help           # Show help
#
# Prerequisites:
#   - ROS2 (Humble or Jazzy) installed
#   - MicroXRCE-DDS Agent (run middleware/setup_dependencies.sh to install)
#   - For sim mode: PX4-Autopilot with Gazebo
#   - For flight mode: RealSense camera + YOLO model weights
# =============================================================================

set -e

# =============================================================================
# PATH sanitization
# =============================================================================
# Linuxbrew ships Python 3.14+ and protobuf versions that conflict with
# ROS2 (needs system Python 3.12) and PX4 builds. Strip it from PATH so
# system toolchain is used consistently.
if echo "$PATH" | grep -q "linuxbrew"; then
    PATH=$(echo "$PATH" | tr ':' '\n' | grep -v linuxbrew | tr '\n' ':' | sed 's/:$//')
    export PATH
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Get script directory (where this script lives)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Default values
MODE=""
START_PX4=true
START_AGENT=true
BUILD_WORKSPACE=false
CONFIG_FILE="${SCRIPT_DIR}/config/airhound.yaml"
PX4_PATH="${HOME}/PX4-Autopilot"
GAZEBO_TARGET="gz_x500"
CAMERA_SOURCE=""  # Empty = use default for mode (synthetic for sim, realsense for flight)

# Track background processes for cleanup
PX4_PID=""
AGENT_PID=""
ROS_PID=""

# =============================================================================
# Functions
# =============================================================================

print_header() {
    echo -e "${CYAN}"
    echo "=============================================="
    echo "     AIRHOUND E2E Launcher"
    echo "=============================================="
    echo -e "${NC}"
}

print_help() {
    echo "Usage: $0 <mode> [options]"
    echo ""
    echo "Modes:"
    echo "  sim       Simulation mode (mock detector + PX4 SITL + Gazebo)"
    echo "  flight    Flight mode (RealSense camera + YOLO + real PX4)"
    echo ""
    echo "Options:"
    echo "  --build           Rebuild ROS2 workspace before launching"
    echo "  --no-px4          Don't start PX4 SITL (assume already running)"
    echo "  --no-agent        Don't start MicroXRCE Agent (assume already running)"
    echo "  --synthetic       Use synthetic/mock detector instead of real camera"
    echo "                    (allows testing control pipeline without camera hardware)"
    echo "  --realsense       Use RealSense camera (default for flight mode)"
    echo "  --config FILE     Use custom config file (default: config/airhound.yaml)"
    echo "  --px4-path PATH   PX4-Autopilot installation path (default: ~/PX4-Autopilot)"
    echo "  --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 sim                          # Full simulation"
    echo "  $0 sim --no-px4                 # Sim (PX4 already running)"
    echo "  $0 sim --build                  # Rebuild and run sim"
    echo "  $0 flight                       # Real flight mode with camera"
    echo "  $0 flight --synthetic           # Flight mode without camera (synthetic detections)"
    echo "  $0 flight --no-agent            # Flight (agent already running)"
    echo ""
}

cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    
    # Kill ROS2 launch and its child nodes
    if [ -n "$ROS_PID" ] && kill -0 "$ROS_PID" 2>/dev/null; then
        echo "Stopping ROS2 nodes..."
        kill -SIGINT "$ROS_PID" 2>/dev/null || true
        wait "$ROS_PID" 2>/dev/null || true
    fi
    pkill -f "mock_detector\|tracking_node\|px4_converter" 2>/dev/null || true
    
    # Kill MicroXRCE Agent
    if [ -n "$AGENT_PID" ] && kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "Stopping MicroXRCE Agent..."
        kill -SIGINT "$AGENT_PID" 2>/dev/null || true
        wait "$AGENT_PID" 2>/dev/null || true
    fi
    
    # Kill PX4 SITL and all its child processes (make -> ninja -> px4 -> gz sim)
    if [ -n "$PX4_PID" ] && kill -0 "$PX4_PID" 2>/dev/null; then
        echo "Stopping PX4 SITL..."
        kill -SIGINT "$PX4_PID" 2>/dev/null || true
        wait "$PX4_PID" 2>/dev/null || true
    fi
    pkill -f "bin/px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    
    # Remove PX4 lock files so next launch starts clean
    rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
    
    echo -e "${GREEN}Cleanup complete.${NC}"
}

check_ros2() {
    # Check which distro is available and source it
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        ROS_DISTRO="humble"
        ROS_SETUP="/opt/ros/humble/setup.bash"
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        ROS_DISTRO="jazzy"
        ROS_SETUP="/opt/ros/jazzy/setup.bash"
    else
        echo -e "${RED}ERROR: No ROS2 installation found in /opt/ros/${NC}"
        echo "Please install ROS2 Humble or Jazzy first."
        echo "See: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi

    # Source ROS2 so ros2 CLI is available for the rest of the script
    source "$ROS_SETUP"

    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}ERROR: ROS2 sourced but ros2 command not found!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}Found ROS2 ${ROS_DISTRO}${NC}"
}

check_microxrce_agent() {
    if command -v MicroXRCEAgent &> /dev/null; then
        AGENT_CMD="MicroXRCEAgent"
        return 0
    fi
    
    # Check local build
    LOCAL_AGENT="${HOME}/.local/airhound_tools/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
    if [ -f "$LOCAL_AGENT" ]; then
        AGENT_CMD="$LOCAL_AGENT"
        return 0
    fi
    
    echo -e "${RED}ERROR: MicroXRCE-DDS Agent not found!${NC}"
    echo "Please run: ${GREEN}./middleware/setup_dependencies.sh${NC}"
    exit 1
}

check_px4() {
    PX4_PATH_EXPANDED="${PX4_PATH/#\~/$HOME}"
    
    if [ ! -d "$PX4_PATH_EXPANDED" ]; then
        echo -e "${YELLOW}WARNING: PX4-Autopilot not found at ${PX4_PATH_EXPANDED}${NC}"
        echo ""
        echo "Would you like to install PX4-Autopilot? (y/n)"
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            install_px4
        else
            echo "Please install PX4-Autopilot or use --no-px4 flag"
            exit 1
        fi
    fi
    
    echo -e "${GREEN}Found PX4-Autopilot at ${PX4_PATH_EXPANDED}${NC}"
}

install_px4() {
    echo -e "${BLUE}Installing PX4-Autopilot...${NC}"
    echo "This may take 20-30 minutes."
    echo ""
    
    cd "${HOME}"
    
    if [ ! -d "PX4-Autopilot" ]; then
        git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    fi
    
    cd PX4-Autopilot
    
    echo "Running PX4 setup script..."
    bash ./Tools/setup/ubuntu.sh
    
    echo "Building PX4 SITL (first build)..."
    make px4_sitl_default
    
    echo -e "${GREEN}PX4-Autopilot installed successfully!${NC}"
}

configure_px4_sitl() {
    # PX4 SITL defaults reject offboard arming because there is no RC
    # transmitter or GCS heartbeat. Patch the x500 airframe to allow
    # offboard operation without RC and suppress datalink-loss failsafes.
    # This only touches the local build and is idempotent.
    
    PX4_PATH_EXPANDED="${PX4_PATH/#\~/$HOME}"
    local AIRFRAME="${PX4_PATH_EXPANDED}/ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500"
    
    if [ ! -f "$AIRFRAME" ]; then
        echo -e "${YELLOW}WARNING: PX4 airframe file not found, skipping SITL param config${NC}"
        return
    fi
    
    local NEEDS_PATCH=false
    
    # Check if our params are already present
    grep -q "COM_RCL_EXCEPT" "$AIRFRAME" || NEEDS_PATCH=true
    
    if [ "$NEEDS_PATCH" = true ]; then
        echo -e "${BLUE}Configuring PX4 SITL parameters for offboard operation...${NC}"
        
        # Append offboard-friendly params (idempotent; only added if missing)
        cat >> "$AIRFRAME" << 'EOF'

# --- AIRHOUND offboard SITL params ---
# Allow offboard mode without RC transmitter
param set-default COM_RCL_EXCEPT 4
# Disable RC loss and datalink loss failsafe actions for SITL
param set-default NAV_RCL_ACT 0
param set-default NAV_DLL_ACT 0
EOF
        
        echo -e "${GREEN}PX4 SITL configured for offboard arming${NC}"
        
        # Clear cached parameters so new defaults take effect on next boot
        rm -rf "${PX4_PATH_EXPANDED}/build/px4_sitl_default/rootfs/eeprom" 2>/dev/null
        rm -f "${PX4_PATH_EXPANDED}/build/px4_sitl_default/rootfs/parameters.bson" 2>/dev/null
        rm -f "${PX4_PATH_EXPANDED}/build/px4_sitl_default/rootfs/parameters_backup.bson" 2>/dev/null
    else
        echo -e "${GREEN}PX4 SITL already configured for offboard${NC}"
    fi
}

source_workspace() {
    echo -e "${BLUE}Sourcing ROS2 environment...${NC}"
    
    # Source ROS2 base
    source "$ROS_SETUP"
    
    # Source workspace if built
    WS_SETUP="${SCRIPT_DIR}/ws_ros2/install/setup.bash"
    if [ -f "$WS_SETUP" ]; then
        source "$WS_SETUP"
        echo -e "${GREEN}Workspace sourced${NC}"
    else
        echo -e "${YELLOW}Workspace not built. Run with --build flag or build manually.${NC}"
        BUILD_WORKSPACE=true
    fi
}

build_workspace() {
    echo -e "${BLUE}Building ROS2 workspace...${NC}"
    
    cd "${SCRIPT_DIR}/ws_ros2"
    
    # Source ROS2 first
    source "$ROS_SETUP"
    
    # Build packages in order
    echo "Building px4_msgs..."
    colcon build --packages-select px4_msgs --symlink-install
    source install/setup.bash
    
    echo "Building airhound_perception..."
    colcon build --packages-select airhound_perception --symlink-install
    source install/setup.bash
    
    echo "Building tracking_geometry..."
    colcon build --packages-select tracking_geometry --symlink-install
    source install/setup.bash
    
    echo "Building offboard_control..."
    colcon build --packages-select offboard_control --symlink-install
    source install/setup.bash
    
    echo -e "${GREEN}Workspace built successfully!${NC}"
}

clean_px4_locks() {
    # PX4 SITL uses lock files that persist after unclean shutdowns,
    # preventing the next launch. Clean them proactively.
    if [ -f "/tmp/px4_lock-0" ]; then
        echo -e "${YELLOW}Cleaning stale PX4 lock files...${NC}"
        pkill -9 -f "bin/px4" 2>/dev/null || true
        pkill -9 -f "gz sim" 2>/dev/null || true
        sleep 1
        rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null
    fi
}

start_px4_sitl() {
    echo -e "${BLUE}Starting PX4 SITL + Gazebo...${NC}"
    
    PX4_PATH_EXPANDED="${PX4_PATH/#\~/$HOME}"
    
    # Clean stale locks from previous runs
    clean_px4_locks
    
    # Configure PX4 parameters for offboard SITL (idempotent)
    configure_px4_sitl
    
    cd "$PX4_PATH_EXPANDED"
    
    # Start PX4 SITL in background
    make px4_sitl "$GAZEBO_TARGET" &
    PX4_PID=$!
    
    echo "PX4 SITL starting (PID: $PX4_PID)"
    echo "Waiting for 'Ready for takeoff'..."
    
    # Wait for PX4 to be ready (check for mavlink heartbeat or timeout)
    sleep 15
    
    echo -e "${GREEN}PX4 SITL should be ready${NC}"
}

start_microxrce_agent() {
    echo -e "${BLUE}Starting MicroXRCE-DDS Agent...${NC}"
    
    $AGENT_CMD udp4 -p 8888 &
    AGENT_PID=$!
    
    echo "MicroXRCE Agent started (PID: $AGENT_PID)"
    sleep 2
    
    echo -e "${GREEN}MicroXRCE Agent running${NC}"
}

launch_sim_mode() {
    echo -e "${CYAN}Launching AIRHOUND in SIMULATION mode${NC}"
    echo ""
    
    cd "${SCRIPT_DIR}"
    source_workspace
    
    if [ "$BUILD_WORKSPACE" = true ]; then
        build_workspace
        source_workspace
    fi
    
    # Start PX4 SITL if requested
    if [ "$START_PX4" = true ]; then
        check_px4
        start_px4_sitl
    fi
    
    # Start MicroXRCE Agent if requested
    if [ "$START_AGENT" = true ]; then
        check_microxrce_agent
        start_microxrce_agent
    fi
    
    echo ""
    echo -e "${GREEN}Starting ROS2 E2E simulation launch...${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    
    # Launch the E2E simulation
    ros2 launch "${SCRIPT_DIR}/launch/e2e_sim.launch.py" &
    ROS_PID=$!
    
    # Wait for ROS2 launch to finish
    wait $ROS_PID
}

launch_flight_mode() {
    echo -e "${CYAN}Launching AIRHOUND in FLIGHT mode${NC}"
    echo ""
    echo -e "${RED}WARNING: This mode controls a REAL drone!${NC}"
    echo -e "${RED}Ensure safety protocols are in place!${NC}"
    echo ""
    
    cd "${SCRIPT_DIR}"
    source_workspace
    
    if [ "$BUILD_WORKSPACE" = true ]; then
        build_workspace
        source_workspace
    fi
    
    # Start MicroXRCE Agent if requested
    if [ "$START_AGENT" = true ]; then
        check_microxrce_agent
        start_microxrce_agent
    fi
    
    # Determine camera source (default to realsense for flight mode)
    local camera_arg=""
    if [ -n "$CAMERA_SOURCE" ]; then
        camera_arg="camera_source:=${CAMERA_SOURCE}"
        echo -e "${BLUE}Using camera source: ${CAMERA_SOURCE}${NC}"
    else
        echo -e "${BLUE}Using camera source: realsense (default)${NC}"
    fi
    
    echo ""
    echo -e "${GREEN}Starting ROS2 E2E flight launch...${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    
    # Launch the E2E flight mode with optional camera_source argument
    if [ -n "$camera_arg" ]; then
        ros2 launch "${SCRIPT_DIR}/launch/e2e_flight.launch.py" "$camera_arg" &
    else
        ros2 launch "${SCRIPT_DIR}/launch/e2e_flight.launch.py" &
    fi
    ROS_PID=$!
    
    # Wait for ROS2 launch to finish
    wait $ROS_PID
}

# =============================================================================
# Main
# =============================================================================

# Set up signal handlers for cleanup
trap cleanup EXIT SIGINT SIGTERM

print_header

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        sim)
            MODE="sim"
            shift
            ;;
        flight)
            MODE="flight"
            shift
            ;;
        --build)
            BUILD_WORKSPACE=true
            shift
            ;;
        --no-px4)
            START_PX4=false
            shift
            ;;
        --no-agent)
            START_AGENT=false
            shift
            ;;
        --synthetic)
            CAMERA_SOURCE="synthetic"
            shift
            ;;
        --realsense)
            CAMERA_SOURCE="realsense"
            shift
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --px4-path)
            PX4_PATH="$2"
            shift 2
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

# Validate mode
if [ -z "$MODE" ]; then
    echo -e "${RED}ERROR: No mode specified!${NC}"
    echo ""
    print_help
    exit 1
fi

# Check ROS2 installation
check_ros2

# Launch based on mode
case $MODE in
    sim)
        launch_sim_mode
        ;;
    flight)
        launch_flight_mode
        ;;
    *)
        echo -e "${RED}Invalid mode: $MODE${NC}"
        exit 1
        ;;
esac
