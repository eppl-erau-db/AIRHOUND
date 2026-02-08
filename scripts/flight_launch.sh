#!/bin/bash
#
# AIRHOUND Flight Test Launcher
#
# One-command launcher for flight testing. Handles:
# - Pre-flight checks
# - ROS2 workspace sourcing
# - Automatic rosbag recording
# - Signal handling for clean shutdown
# - Post-flight log organization
#
# Usage:
#   ./scripts/flight_launch.sh                    # Default: RF-DETR, no auto-arm
#   ./scripts/flight_launch.sh --detector yolo    # Use YOLOv8
#   ./scripts/flight_launch.sh --arm              # Enable auto-arm (DANGEROUS)
#   ./scripts/flight_launch.sh --skip-preflight   # Skip pre-flight checks
#   ./scripts/flight_launch.sh --dry-run          # Check only, don't launch
#
set -euo pipefail

# ===== Configuration =====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AIRHOUND_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${AIRHOUND_ROOT}/ws_ros2"
LOG_DIR="${AIRHOUND_ROOT}/flight_logs"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# ===== Defaults =====
DETECTOR="rfdetr"
AUTO_ARM="false"
SKIP_PREFLIGHT="false"
DRY_RUN="false"
RECORD_BAG="true"
SIM_MODE="false"

# ===== Parse Arguments =====
while [[ $# -gt 0 ]]; do
    case $1 in
        --detector)
            DETECTOR="$2"
            shift 2
            ;;
        --arm|--auto-arm)
            AUTO_ARM="true"
            shift
            ;;
        --skip-preflight)
            SKIP_PREFLIGHT="true"
            shift
            ;;
        --dry-run)
            DRY_RUN="true"
            shift
            ;;
        --no-record)
            RECORD_BAG="false"
            shift
            ;;
        --sim)
            SIM_MODE="true"
            shift
            ;;
        -h|--help)
            echo "AIRHOUND Flight Test Launcher"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --detector TYPE    Detector type: rfdetr (default) or yolo"
            echo "  --arm              Enable auto-arm (DANGEROUS - controlled tests only)"
            echo "  --skip-preflight   Skip pre-flight checks"
            echo "  --dry-run          Run checks only, don't launch"
            echo "  --no-record        Don't record rosbag"
            echo "  --sim              Simulation mode (no real camera)"
            echo "  -h, --help         Show this help"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# ===== Functions =====
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

# Track PIDs for cleanup
PIDS=()
BAG_PID=""
LAUNCH_PID=""

cleanup() {
    echo ""
    log_info "Shutting down AIRHOUND..."
    
    # Stop rosbag recording gracefully
    if [[ -n "$BAG_PID" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
        log_info "Stopping rosbag recording..."
        kill -INT "$BAG_PID" 2>/dev/null || true
        sleep 2
    fi
    
    # Stop ROS2 launch
    if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        log_info "Stopping ROS2 nodes..."
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        sleep 3
        kill -KILL "$LAUNCH_PID" 2>/dev/null || true
    fi
    
    # Kill any remaining processes
    for pid in "${PIDS[@]}"; do
        kill -KILL "$pid" 2>/dev/null || true
    done
    
    # Organize logs
    if [[ -d "$FLIGHT_LOG_DIR" ]]; then
        log_info "Flight logs saved to: ${FLIGHT_LOG_DIR}"
        
        # Create summary
        cat > "${FLIGHT_LOG_DIR}/flight_summary.txt" <<EOF
AIRHOUND Flight Test Summary
=============================
Session ID: ${SESSION_ID}
Start Time: ${START_TIME}
End Time:   $(date '+%Y-%m-%d %H:%M:%S')
Detector:   ${DETECTOR}
Auto-Arm:   ${AUTO_ARM}
Sim Mode:   ${SIM_MODE}

Files:
$(ls -lh "${FLIGHT_LOG_DIR}" 2>/dev/null | tail -n +2)
EOF
    fi
    
    log_success "Shutdown complete"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ===== Main =====
echo -e "${BOLD}"
echo "  ___  _____ ____  _   _  ___  _   _ _   _ ____  "
echo " / _ \|_   _|  _ \| | | |/ _ \| | | | \ | |  _ \ "
echo "| |_| | | | | |_) | |_| | | | | | | |  \| | | | |"
echo "| ___ | | | |  _ <|  _  | |_| | |_| | |\  | |_| |"
echo "|_| |_| |_| |_| \_\_| |_|\___/ \___/|_| \_|____/ "
echo -e "${NC}"
echo "         Autonomous Drone Pursuit System"
echo ""

# Session ID for this flight
SESSION_ID="flight_$(date +%Y%m%d_%H%M%S)"
START_TIME="$(date '+%Y-%m-%d %H:%M:%S')"
FLIGHT_LOG_DIR="${LOG_DIR}/${SESSION_ID}"

log_info "Session: ${SESSION_ID}"
log_info "Detector: ${DETECTOR}"
log_info "Auto-Arm: ${AUTO_ARM}"

# ===== Pre-Flight Checks =====
if [[ "$SKIP_PREFLIGHT" == "false" ]]; then
    log_info "Running pre-flight checks..."
    
    if python3 "${SCRIPT_DIR}/flight_preflight_check.py" --skip-inference; then
        log_success "Pre-flight checks passed"
    else
        exit_code=$?
        if [[ $exit_code -eq 2 ]]; then
            log_warn "Pre-flight checks have warnings"
            read -p "Continue anyway? [y/N] " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        else
            log_error "Pre-flight checks failed"
            exit 1
        fi
    fi
else
    log_warn "Skipping pre-flight checks"
fi

if [[ "$DRY_RUN" == "true" ]]; then
    log_info "Dry run complete"
    exit 0
fi

# ===== Setup Log Directory =====
mkdir -p "$FLIGHT_LOG_DIR"
log_info "Logs: ${FLIGHT_LOG_DIR}"

# ===== Source ROS2 Workspace =====
log_info "Sourcing ROS2 workspace..."
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"

# ===== Start Rosbag Recording =====
if [[ "$RECORD_BAG" == "true" ]]; then
    log_info "Starting rosbag recording..."
    
    BAG_TOPICS=(
        "/camera/color/image_raw"
        "/camera/depth/image_raw"
        "/camera/camera_info"
        "/detections"
        "/perception/target_3d"
        "/perception/depth_quality"
        "/perception/fps"
        "/target_yaw"
        "/fmu/in/trajectory_setpoint"
        "/fmu/in/offboard_control_mode"
        "/fmu/out/vehicle_local_position"
        "/fmu/out/vehicle_attitude"
    )
    
    ros2 bag record \
        -o "${FLIGHT_LOG_DIR}/rosbag" \
        --compression-mode file \
        --compression-format zstd \
        "${BAG_TOPICS[@]}" \
        > "${FLIGHT_LOG_DIR}/rosbag.log" 2>&1 &
    BAG_PID=$!
    PIDS+=($BAG_PID)
    
    sleep 1
    if kill -0 "$BAG_PID" 2>/dev/null; then
        log_success "Rosbag recording started (PID: $BAG_PID)"
    else
        log_warn "Rosbag recording failed to start"
        BAG_PID=""
    fi
fi

# ===== Launch Pipeline =====
log_info "Launching AIRHOUND pipeline..."
echo ""
echo -e "${BOLD}========================================${NC}"
echo -e "${GREEN}  Pipeline Starting - Press Ctrl+C to stop${NC}"
echo -e "${BOLD}========================================${NC}"
echo ""

ros2 launch airhound_perception full_pipeline.launch.py \
    detector:=${DETECTOR} \
    auto_arm:=${AUTO_ARM} \
    sim_mode:=${SIM_MODE} \
    start_agent:=false \
    2>&1 | tee "${FLIGHT_LOG_DIR}/pipeline.log" &
LAUNCH_PID=$!
PIDS+=($LAUNCH_PID)

# Wait for launch to complete or be interrupted
wait $LAUNCH_PID
