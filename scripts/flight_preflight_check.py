#!/usr/bin/env python3
"""
AIRHOUND Pre-Flight Check Script

Validates all system components before flight testing.
Run this BEFORE every flight session.

Exit codes:
  0 - All checks passed, safe to fly
  1 - Critical failure, DO NOT FLY
  2 - Warnings present, proceed with caution

Usage:
  python3 scripts/flight_preflight_check.py
  python3 scripts/flight_preflight_check.py --json  # Machine-readable output
"""

import subprocess
import sys
import os
import json
import argparse
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import List, Optional
import time

# ANSI colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"
BOLD = "\033[1m"


@dataclass
class CheckResult:
    name: str
    passed: bool
    critical: bool
    message: str
    details: Optional[str] = None


@dataclass
class PreflightReport:
    timestamp: str
    hostname: str
    checks: List[CheckResult] = field(default_factory=list)
    passed: bool = True
    warnings: int = 0
    failures: int = 0
    
    def add(self, result: CheckResult):
        self.checks.append(result)
        if not result.passed:
            if result.critical:
                self.failures += 1
                self.passed = False
            else:
                self.warnings += 1


def run_cmd(cmd: str, timeout: int = 10) -> tuple:
    """Run command and return (success, stdout, stderr)"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode == 0, result.stdout.strip(), result.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"
    except Exception as e:
        return False, "", str(e)


def check_realsense_camera() -> CheckResult:
    """Check Intel RealSense D455 camera is connected and accessible"""
    success, stdout, _ = run_cmd("lsusb | grep -i 'Intel.*RealSense'")
    if not success or not stdout:
        return CheckResult(
            name="RealSense Camera",
            passed=False,
            critical=True,
            message="D455 camera not detected on USB",
            details="Connect D455 to USB 3.x port. Check cable and power."
        )
    
    # Check for correct USB 3.x speed
    success2, stdout2, _ = run_cmd("lsusb -t | grep -A1 -i realsense || lsusb -t | grep 5000M")
    usb_speed = "USB 3.x" if "5000M" in stdout2 else "USB 2.x (SLOW)"
    
    return CheckResult(
        name="RealSense Camera",
        passed=True,
        critical=True,
        message=f"D455 detected ({usb_speed})",
        details=stdout.strip()
    )


def check_tensorrt_engine() -> CheckResult:
    """Check TensorRT engine file exists and is valid"""
    engine_path = Path("/home/airhound/AIRHOUND/models/drone_rfdetr.engine")
    
    if not engine_path.exists():
        return CheckResult(
            name="TensorRT Engine",
            passed=False,
            critical=True,
            message="RF-DETR engine not found",
            details=f"Expected at: {engine_path}"
        )
    
    size_mb = engine_path.stat().st_size / (1024 * 1024)
    if size_mb < 10:
        return CheckResult(
            name="TensorRT Engine",
            passed=False,
            critical=True,
            message=f"Engine file too small ({size_mb:.1f}MB), may be corrupted"
        )
    
    return CheckResult(
        name="TensorRT Engine",
        passed=True,
        critical=True,
        message=f"drone_rfdetr.engine ({size_mb:.1f}MB)"
    )


def check_ros2_workspace() -> CheckResult:
    """Check ROS2 workspace is built and packages available"""
    ws_path = Path("/home/airhound/AIRHOUND/ws_ros2/install")
    
    if not ws_path.exists():
        return CheckResult(
            name="ROS2 Workspace",
            passed=False,
            critical=True,
            message="Workspace not built",
            details="Run: cd ~/AIRHOUND/ws_ros2 && colcon build"
        )
    
    required_pkgs = ["airhound_perception", "offboard_control", "px4_msgs"]
    missing = [p for p in required_pkgs if not (ws_path / p).exists()]
    
    if missing:
        return CheckResult(
            name="ROS2 Workspace",
            passed=False,
            critical=True,
            message=f"Missing packages: {', '.join(missing)}",
            details="Run: cd ~/AIRHOUND/ws_ros2 && colcon build"
        )
    
    return CheckResult(
        name="ROS2 Workspace",
        passed=True,
        critical=True,
        message=f"All {len(required_pkgs)} packages installed"
    )


def check_px4_connection() -> CheckResult:
    """Check PX4 serial/network connection"""
    # Check for USB serial
    success, stdout, _ = run_cmd("ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null")
    
    if stdout:
        return CheckResult(
            name="PX4 Connection",
            passed=True,
            critical=True,
            message=f"Serial device: {stdout.split()[0]}"
        )
    
    # Check for network connection (common for SITL or ethernet)
    success, stdout, _ = run_cmd("ping -c 1 -W 1 192.168.1.1 2>/dev/null || ping -c 1 -W 1 127.0.0.1 2>/dev/null")
    
    return CheckResult(
        name="PX4 Connection",
        passed=False,
        critical=False,  # Non-critical for bench testing
        message="No USB serial device found",
        details="Connect Pixhawk via USB or configure ethernet. For bench test, can proceed."
    )


def check_micro_xrce_agent() -> CheckResult:
    """Check MicroXRCE-DDS agent is available"""
    success, stdout, _ = run_cmd("which MicroXRCEAgent 2>/dev/null || which micro_ros_agent 2>/dev/null")
    
    if success and stdout:
        return CheckResult(
            name="DDS Agent",
            passed=True,
            critical=False,
            message=f"Found: {stdout}"
        )
    
    # Check if it's running
    success2, stdout2, _ = run_cmd("pgrep -f 'MicroXRCE|micro_ros_agent'")
    if success2 and stdout2:
        return CheckResult(
            name="DDS Agent",
            passed=True,
            critical=False,
            message="Agent already running"
        )
    
    return CheckResult(
        name="DDS Agent",
        passed=False,
        critical=False,
        message="MicroXRCE-DDS agent not found",
        details="Install: sudo apt install ros-humble-micro-ros-agent"
    )


def check_gpu_status() -> CheckResult:
    """Check NVIDIA GPU and CUDA availability"""
    success, stdout, _ = run_cmd("nvidia-smi --query-gpu=name,temperature.gpu,memory.used,memory.total --format=csv,noheader,nounits")
    
    if not success:
        return CheckResult(
            name="GPU Status",
            passed=False,
            critical=True,
            message="nvidia-smi failed",
            details="Check NVIDIA driver installation"
        )
    
    parts = stdout.split(',')
    if len(parts) >= 4:
        name = parts[0].strip()
        temp_str = parts[1].strip()
        mem_used_str = parts[2].strip()
        mem_total_str = parts[3].strip()
        
        # Handle N/A values (common on Jetson)
        try:
            temp = int(temp_str) if temp_str not in ('[N/A]', 'N/A', '') else None
        except ValueError:
            temp = None
        try:
            mem_used = int(mem_used_str) if mem_used_str not in ('[N/A]', 'N/A', '') else None
        except ValueError:
            mem_used = None
        try:
            mem_total = int(mem_total_str) if mem_total_str not in ('[N/A]', 'N/A', '') else None
        except ValueError:
            mem_total = None
        
        # Build status message
        status_parts = [name]
        if temp is not None:
            status_parts.append(f"@ {temp}C")
            # Warn if GPU is too hot
            if temp > 80:
                return CheckResult(
                    name="GPU Status",
                    passed=False,
                    critical=False,
                    message=f"{name} @ {temp}C (OVERHEATING!)",
                    details="Let GPU cool before flight"
                )
        
        if mem_used is not None and mem_total is not None:
            mem_free = mem_total - mem_used
            status_parts.append(f"{mem_free}MB free")
            if mem_free < 2000:  # Less than 2GB free
                return CheckResult(
                    name="GPU Status",
                    passed=False,
                    critical=False,
                    message=f"{name}, only {mem_free}MB free",
                    details="Close other applications to free GPU memory"
                )
        
        return CheckResult(
            name="GPU Status",
            passed=True,
            critical=True,
            message=", ".join(status_parts)
        )
    
    return CheckResult(
        name="GPU Status",
        passed=True,
        critical=True,
        message="GPU available"
    )


def check_disk_space() -> CheckResult:
    """Check sufficient disk space for logging"""
    success, stdout, _ = run_cmd("df -h /home | tail -1")
    
    if not success:
        return CheckResult(
            name="Disk Space",
            passed=False,
            critical=False,
            message="Could not check disk space"
        )
    
    parts = stdout.split()
    if len(parts) >= 4:
        available = parts[3]
        percent_used = parts[4].replace('%', '')
        
        if int(percent_used) > 90:
            return CheckResult(
                name="Disk Space",
                passed=False,
                critical=False,
                message=f"Only {available} free ({percent_used}% used)",
                details="Clear old logs: rm -rf ~/AIRHOUND/flight_logs/old_*"
            )
        
        return CheckResult(
            name="Disk Space",
            passed=True,
            critical=False,
            message=f"{available} free ({percent_used}% used)"
        )
    
    return CheckResult(name="Disk Space", passed=True, critical=False, message="OK")


def check_cpu_load() -> CheckResult:
    """Check CPU load is reasonable"""
    success, stdout, _ = run_cmd("uptime | awk -F'load average:' '{print $2}' | awk -F',' '{print $1}'")
    
    if success and stdout:
        load = float(stdout.strip())
        cpus = os.cpu_count() or 4
        
        if load > cpus * 0.8:
            return CheckResult(
                name="CPU Load",
                passed=False,
                critical=False,
                message=f"High load: {load:.2f} (cores: {cpus})",
                details="Close unnecessary applications"
            )
        
        return CheckResult(
            name="CPU Load",
            passed=True,
            critical=False,
            message=f"Load: {load:.2f} / {cpus} cores"
        )
    
    return CheckResult(name="CPU Load", passed=True, critical=False, message="OK")


def check_inference_test() -> CheckResult:
    """Quick inference test to verify detector works"""
    test_script = '''
import sys
sys.path.insert(0, '/home/airhound/AIRHOUND/ws_ros2/src/airhound_perception/airhound_perception')
from rfdetr_detector import RFDETRDetector
import numpy as np
det = RFDETRDetector('/home/airhound/AIRHOUND/models/drone_rfdetr.engine', conf=0.25)
img = np.zeros((720, 1280, 3), dtype=np.uint8)
_ = det.infer(img)
print("OK")
'''
    success, stdout, stderr = run_cmd(f'python3 -c "{test_script}"', timeout=30)
    
    if success and "OK" in stdout:
        return CheckResult(
            name="Inference Test",
            passed=True,
            critical=True,
            message="RF-DETR inference OK"
        )
    
    return CheckResult(
        name="Inference Test",
        passed=False,
        critical=True,
        message="Inference test failed",
        details=stderr[:200] if stderr else "Unknown error"
    )


def run_preflight_checks(skip_inference: bool = False) -> PreflightReport:
    """Run all preflight checks"""
    report = PreflightReport(
        timestamp=time.strftime("%Y-%m-%d %H:%M:%S"),
        hostname=os.uname().nodename
    )
    
    checks = [
        check_realsense_camera,
        check_tensorrt_engine,
        check_ros2_workspace,
        check_gpu_status,
        check_px4_connection,
        check_micro_xrce_agent,
        check_disk_space,
        check_cpu_load,
    ]
    
    if not skip_inference:
        checks.append(check_inference_test)
    
    for check_fn in checks:
        result = check_fn()
        report.add(result)
    
    return report


def print_report(report: PreflightReport):
    """Print formatted report to terminal"""
    print(f"\n{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}  AIRHOUND PRE-FLIGHT CHECK{RESET}")
    print(f"  {report.timestamp} | {report.hostname}")
    print(f"{'='*60}\n")
    
    for check in report.checks:
        if check.passed:
            icon = f"{GREEN}[PASS]{RESET}"
        elif check.critical:
            icon = f"{RED}[FAIL]{RESET}"
        else:
            icon = f"{YELLOW}[WARN]{RESET}"
        
        print(f"  {icon} {check.name}: {check.message}")
        if check.details and not check.passed:
            print(f"         {BLUE}{check.details}{RESET}")
    
    print(f"\n{'='*60}")
    
    if report.passed and report.warnings == 0:
        print(f"  {GREEN}{BOLD}ALL CHECKS PASSED - READY FOR FLIGHT{RESET}")
        exit_code = 0
    elif report.passed:
        print(f"  {YELLOW}{BOLD}PASSED WITH {report.warnings} WARNING(S) - PROCEED WITH CAUTION{RESET}")
        exit_code = 2
    else:
        print(f"  {RED}{BOLD}FAILED - {report.failures} CRITICAL ISSUE(S) - DO NOT FLY{RESET}")
        exit_code = 1
    
    print(f"{'='*60}\n")
    return exit_code


def main():
    parser = argparse.ArgumentParser(description="AIRHOUND Pre-Flight Check")
    parser.add_argument("--json", action="store_true", help="Output as JSON")
    parser.add_argument("--skip-inference", action="store_true", help="Skip inference test")
    args = parser.parse_args()
    
    report = run_preflight_checks(skip_inference=args.skip_inference)
    
    if args.json:
        output = {
            "timestamp": report.timestamp,
            "hostname": report.hostname,
            "passed": report.passed,
            "warnings": report.warnings,
            "failures": report.failures,
            "checks": [asdict(c) for c in report.checks]
        }
        print(json.dumps(output, indent=2))
        sys.exit(0 if report.passed else 1)
    else:
        exit_code = print_report(report)
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
