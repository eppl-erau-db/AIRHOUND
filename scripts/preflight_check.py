#!/usr/bin/env python3
"""
AIRHOUND Pre-Flight Checklist

Run before every flight to verify all systems are go.
Checks ROS2 topics, PX4 connection, camera, models, and disk space.

Usage:
    python3 preflight_check.py                    # Full check (realsense mode)
    python3 preflight_check.py --mode synthetic   # Synthetic mode (no camera)
    python3 preflight_check.py --mode sitl        # SITL mode (no camera, no FC)

Exit code 0 = all checks pass. Non-zero = failures found.
"""

import argparse
import subprocess
import os
import sys
import shutil
from pathlib import Path


def run(cmd, timeout=5):
    """Run a command, return (success, stdout)."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=True)
        return r.returncode == 0, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, "TIMEOUT"
    except Exception as e:
        return False, str(e)


def check(name, passed, detail=""):
    status = "✅" if passed else "❌"
    line = f"  {status} {name}"
    if detail:
        line += f"  ({detail})"
    print(line)
    return passed


def main():
    parser = argparse.ArgumentParser(description='AIRHOUND pre-flight checklist')
    parser.add_argument('--mode', choices=['realsense', 'synthetic', 'sitl'],
                        default='realsense')
    args = parser.parse_args()

    print("=" * 55)
    print(f"  AIRHOUND PRE-FLIGHT CHECKLIST  [mode: {args.mode}]")
    print("=" * 55)

    failures = 0

    # ---- ROS2 Environment ----
    print("\n[ROS2 Environment]")
    ok, out = run("ros2 topic list 2>/dev/null | head -1")
    if not check("ROS2 daemon reachable", ok):
        failures += 1

    # ---- PX4 Connection ----
    print("\n[PX4 Connection]")
    if args.mode != 'sitl':
        # Check MicroXRCE-DDS agent
        ok, out = run("pgrep -f MicroXRCEAgent")
        if not check("MicroXRCE-DDS Agent running", ok, "start with: MicroXRCEAgent udp4 -p 8888"):
            failures += 1

    # Check PX4 topics
    ok, out = run("ros2 topic list 2>/dev/null | grep -c fmu")
    fmu_count = int(out) if ok and out.isdigit() else 0
    if not check(f"PX4 topics available ({fmu_count} /fmu/* topics)", fmu_count >= 2):
        failures += 1

    # Check attitude data flowing
    ok, out = run("timeout 2 ros2 topic hz /fmu/out/vehicle_attitude --window 5 2>&1 | head -1")
    if not check("Vehicle attitude publishing", ok and "average rate" in out, out[:60] if out else ""):
        failures += 1

    # ---- Camera (realsense mode only) ----
    if args.mode == 'realsense':
        print("\n[Camera]")
        ok, out = run("ros2 topic list 2>/dev/null | grep -c camera")
        cam_count = int(out) if ok and out.isdigit() else 0
        if not check(f"Camera topics available ({cam_count})", cam_count >= 2):
            failures += 1

        ok, out = run("timeout 2 ros2 topic hz /camera/color/image_raw --window 5 2>&1 | head -1")
        if not check("RGB stream publishing", ok and "average rate" in out, out[:60] if out else ""):
            failures += 1

    # ---- Model Weights ----
    print("\n[Model Weights]")
    # Resolve relative to script location (works on both laptop and Jetson)
    script_dir = Path(__file__).resolve().parent.parent
    model_dir = script_dir / "models"

    # Check for any TensorRT engine
    engines = list(model_dir.glob("*.engine")) + list(model_dir.glob("**/*.engine"))
    if not check(f"TensorRT engine(s) found ({len(engines)})",
                 len(engines) > 0 or args.mode in ('synthetic', 'sitl'),
                 ", ".join(e.name for e in engines[:3]) if engines else "none"):
        failures += 1

    # PINN weights (ONNX preferred for Jetson, PyTorch as fallback)
    pinn_onnx = model_dir / "pinn" / "pinn_best_ir8.onnx"
    pinn_pth = model_dir / "pinn" / "pinn_best.pth"
    pinn_ok = pinn_onnx.exists() or pinn_pth.exists()
    pinn_found = str(pinn_onnx if pinn_onnx.exists() else pinn_pth)
    if not check("PINN weights exist", pinn_ok, pinn_found):
        failures += 1

    pinn_norm = model_dir / "pinn" / "norm_stats.npz"
    if not check("PINN norm stats exist", pinn_norm.exists()):
        failures += 1

    # ---- Disk Space ----
    print("\n[Disk Space]")
    bag_dir = Path(os.path.expanduser("~/airhound_bags"))
    bag_dir.mkdir(exist_ok=True)
    usage = shutil.disk_usage(str(bag_dir))
    free_gb = usage.free / (1024 ** 3)
    if not check(f"Disk space for bags ({free_gb:.1f} GB free)", free_gb > 2.0,
                 "need >2 GB for ~10 min flight"):
        failures += 1

    # Count existing bags
    existing_bags = list(bag_dir.iterdir()) if bag_dir.exists() else []
    check(f"Existing bags: {len(existing_bags)}", True)

    # ---- Workspace Build ----
    print("\n[Workspace]")
    ws_install = script_dir / "ws_ros2" / "install"
    packages = ['airhound_perception', 'tracking_geometry', 'offboard_control']
    for pkg in packages:
        pkg_path = ws_install / pkg
        if not check(f"Package built: {pkg}", pkg_path.exists()):
            failures += 1

    # ---- Topic Connectivity (quick check) ----
    print("\n[Topic Connectivity]")
    ok, out = run("ros2 topic list 2>/dev/null")
    topics = out.split('\n') if ok else []

    expected = ['/fmu/out/vehicle_attitude']
    if args.mode == 'realsense':
        expected.append('/camera/color/image_raw')

    for topic in expected:
        found = topic in topics
        if not check(f"Topic exists: {topic}", found):
            failures += 1

    # ---- Summary ----
    print("\n" + "=" * 55)
    if failures == 0:
        print("  ✅  ALL CHECKS PASSED — READY FOR FLIGHT")
    else:
        print(f"  ❌  {failures} CHECK(S) FAILED — DO NOT FLY")
    print("=" * 55)

    return failures


if __name__ == '__main__':
    sys.exit(main())
