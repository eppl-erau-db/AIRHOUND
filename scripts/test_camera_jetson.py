#!/usr/bin/env python3
"""
Standalone camera + perception test for Jetson.
No PX4, no tracking, no control. Just camera → detector → print results.

Usage:
    # Camera only (verify RealSense driver)
    python3 scripts/test_camera_jetson.py --camera-only

    # Camera + RF-DETR detector
    python3 scripts/test_camera_jetson.py --model models/drone_rfdetr.engine

    # Camera + detector, save sample frame
    python3 scripts/test_camera_jetson.py --model models/drone_rfdetr.engine --save-frame
"""

import argparse
import subprocess
import sys
import time
import signal

def run(cmd, timeout=10):
    """Run shell command, return (success, output)."""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return r.returncode == 0, r.stdout.strip()
    except subprocess.TimeoutExpired:
        return False, "timeout"

def check(label, condition, detail=""):
    icon = "✅" if condition else "❌"
    msg = f"  {icon} {label}"
    if detail:
        msg += f"  ({detail})"
    print(msg)
    return condition

def main():
    parser = argparse.ArgumentParser(description="AIRHOUND Jetson camera test")
    parser.add_argument("--camera-only", action="store_true",
                        help="Only test RealSense, skip detector")
    parser.add_argument("--model", default="models/drone_rfdetr.engine",
                        help="Path to detector model")
    parser.add_argument("--save-frame", action="store_true",
                        help="Save a sample frame to /tmp/airhound_test_frame.jpg")
    parser.add_argument("--duration", type=int, default=15,
                        help="Seconds to run test (default 15)")
    args = parser.parse_args()

    print("=" * 55)
    print("  AIRHOUND CAMERA TEST — Jetson Standalone")
    print("=" * 55)

    # Step 1: Check RealSense device
    print("\n[USB Devices]")
    ok, out = run("lsusb | grep -i 'intel.*real\\|8086:0b07\\|8086:0b5c'")
    check("RealSense USB device detected", ok, out[:80] if out else "not found")
    if not ok:
        print("\n  ⚠  No RealSense detected on USB. Plug it in and retry.")
        sys.exit(1)

    # Step 2: Check realsense2_camera package
    print("\n[ROS2 Packages]")
    ok, _ = run("ros2 pkg list 2>/dev/null | grep realsense2_camera")
    if not check("realsense2_camera installed", ok):
        print("  Install: sudo apt install ros-humble-realsense2-camera")
        sys.exit(1)

    # Step 3: Launch camera
    print(f"\n[Launching RealSense for {args.duration}s...]")
    cam_cmd = (
        "ros2 launch realsense2_camera rs_launch.py "
        "enable_color:=true enable_depth:=false "
        "rgb_camera.color_profile:=1280x720x30 "
        "enable_gyro:=false enable_accel:=false "
    )
    cam_proc = subprocess.Popen(
        cam_cmd, shell=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
    )
    time.sleep(5)  # Wait for camera to initialize

    # Step 4: Check camera topics
    print("\n[Camera Topics]")
    ok, out = run("ros2 topic list 2>/dev/null | grep camera")
    topics = out.split('\n') if ok and out else []
    check(f"Camera topics ({len(topics)})", len(topics) >= 2,
          ", ".join(t for t in topics[:5]))

    ok, out = run("timeout 3 ros2 topic hz /camera/color/image_raw --window 5 2>&1 | head -1")
    check("RGB stream publishing", ok and "average rate" in out, out[:60] if out else "no data")

    ok, out = run("timeout 3 ros2 topic echo /camera/color/camera_info --once 2>&1 | grep 'width\\|height' | head -2")
    check("Camera info available", ok, out.replace('\n', ', ')[:60] if out else "")

    if args.save_frame:
        print("\n[Saving sample frame...]")
        save_ok, _ = run(
            "python3 -c \""
            "import rclpy; from sensor_msgs.msg import Image; import cv2; import numpy as np; "
            "rclpy.init(); node = rclpy.create_node('frame_saver'); "
            "msg = [None]; "
            "sub = node.create_subscription(Image, '/camera/color/image_raw', lambda m: msg.__setitem__(0, m), 10); "
            "rclpy.spin_once(node, timeout_sec=3.0); "
            "rclpy.spin_once(node, timeout_sec=3.0); "
            "img = msg[0]; "
            "frame = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, 3) if img else None; "
            "cv2.imwrite('/tmp/airhound_test_frame.jpg', frame[:,:,::-1]) if frame is not None else None; "
            "print(f'Saved {img.width}x{img.height}' if img else 'No frame'); "
            "node.destroy_node(); rclpy.shutdown()"
            "\"", timeout=10
        )
        check("Frame saved to /tmp/airhound_test_frame.jpg", save_ok)

    if args.camera_only:
        print(f"\n[Camera-only test complete]")
        cam_proc.terminate()
        cam_proc.wait(timeout=5)
        return

    # Step 5: Launch detector
    print(f"\n[Launching RF-DETR detector: {args.model}]")
    det_cmd = (
        f"ros2 run airhound_perception detector_node "
        f"--ros-args "
        f"-p model_path:={args.model} "
        f"-p confidence_threshold:=0.25 "
        f"-p device:=0 "
        f"-p input_image_topic:=/camera/color/image_raw "
        f"-p output_detections_topic:=/detections "
    )
    det_proc = subprocess.Popen(
        det_cmd, shell=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
    )
    time.sleep(8)  # TRT engine load can take a few seconds

    # Step 6: Check detector output
    print("\n[Detector Output]")
    ok, out = run("timeout 3 ros2 topic hz /detections --window 5 2>&1 | head -1")
    check("Detections topic publishing", ok and "average rate" in out, out[:60] if out else "no data")

    ok, out = run("timeout 3 ros2 topic hz /perception/fps --window 5 2>&1 | head -1")
    check("FPS topic publishing", ok and "average rate" in out, out[:60] if out else "")

    # Count detections over a few seconds
    ok, out = run(
        "timeout 5 ros2 topic echo /detections --field detections 2>/dev/null | grep -c 'hypothesis' || echo 0",
        timeout=8
    )
    det_count = int(out) if out.isdigit() else 0
    check(f"Detections in 5s: {det_count}", True,
          "0 is expected if no drone in frame")

    # Check FPS value
    ok, out = run("timeout 3 ros2 topic echo /perception/fps --once 2>/dev/null | grep 'data:'")
    check("Reported FPS", ok, out[:40] if out else "")

    # Cleanup
    print("\n[Cleanup]")
    det_proc.terminate()
    cam_proc.terminate()
    det_proc.wait(timeout=5)
    cam_proc.wait(timeout=5)
    print("  Done.")

    print("\n" + "=" * 55)
    print("  Camera test complete. Review results above.")
    print("=" * 55)

if __name__ == "__main__":
    main()
