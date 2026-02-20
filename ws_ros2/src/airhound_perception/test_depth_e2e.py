#!/usr/bin/env python3
"""
Quick E2E Test for Depth Extraction Logic

Tests the core depth extraction and 3D projection functions
without requiring ROS2 or actual hardware.

Author: Perception Lead (CLI Agent Demo)
"""

import numpy as np
import sys

# Test configuration
DEPTH_SCALE = 0.001  # D455: mm -> meters
DEPTH_WINDOW_SIZE = 5
DEPTH_MIN_RANGE = 0.4  # meters
DEPTH_MAX_RANGE = 6.0  # meters

# Mock camera intrinsics (typical D455 @ 1280x720)
CAMERA_K = [
    909.34,  0.0,    640.0,   # fx, 0, cx
    0.0,     909.34, 360.0,   # 0, fy, cy
    0.0,     0.0,    1.0      # 0, 0, 1
]


def extract_depth_at_point(depth_image: np.ndarray, u: int, v: int) -> float:
    """Extract depth at pixel (u, v) using median filtering."""
    if depth_image is None:
        return float('nan')
    
    h, w = depth_image.shape[:2]
    
    # Clamp to image bounds
    u = max(0, min(int(u), w - 1))
    v = max(0, min(int(v), h - 1))
    
    # Extract window around point for median filtering
    half_win = DEPTH_WINDOW_SIZE // 2
    u_min = max(0, u - half_win)
    u_max = min(w, u + half_win + 1)
    v_min = max(0, v - half_win)
    v_max = min(h, v + half_win + 1)
    
    window = depth_image[v_min:v_max, u_min:u_max]
    
    # Filter out zero/invalid depth values
    valid_depths = window[window > 0]
    
    if len(valid_depths) == 0:
        return float('nan')
    
    # Use median for robustness
    depth_raw = float(np.median(valid_depths))
    
    # Apply scale factor (D455: mm -> m)
    depth_meters = depth_raw * DEPTH_SCALE
    
    # Filter by depth range
    if depth_meters < DEPTH_MIN_RANGE or depth_meters > DEPTH_MAX_RANGE:
        return float('nan')
    
    return depth_meters


def project_to_3d(u: float, v: float, depth_m: float) -> tuple:
    """Project 2D pixel + depth to 3D position in camera frame."""
    if np.isnan(depth_m):
        return (float('nan'), float('nan'), float('nan'))
    
    fx = CAMERA_K[0]
    fy = CAMERA_K[4]
    cx = CAMERA_K[2]
    cy = CAMERA_K[5]
    
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    
    return (x, y, z)


def run_tests():
    """Run E2E tests for depth extraction."""
    print("=" * 60)
    print("AIRHOUND Depth Extraction E2E Test")
    print("=" * 60)
    print()
    
    tests_passed = 0
    tests_failed = 0
    
    # Test 1: Basic depth extraction
    print("Test 1: Basic depth extraction at image center")
    depth_img = np.full((480, 848), 2500, dtype=np.uint16)  # 2.5m everywhere
    depth = extract_depth_at_point(depth_img, 424, 240)
    expected = 2.5
    if abs(depth - expected) < 0.01:
        print(f"  ✓ PASS: depth={depth:.3f}m (expected {expected}m)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (expected {expected}m)")
        tests_failed += 1
    
    # Test 2: Depth with noise (median filter test)
    print("\nTest 2: Depth with noise (median filter robustness)")
    depth_img = np.full((480, 848), 3000, dtype=np.uint16)  # 3.0m base
    # Add noise/outliers
    depth_img[238:242, 422:426] = 3000  # center is 3m
    depth_img[238, 422] = 10000  # one outlier at 10m
    depth_img[239, 423] = 0      # one invalid pixel
    depth = extract_depth_at_point(depth_img, 424, 240)
    expected = 3.0
    if abs(depth - expected) < 0.1:
        print(f"  ✓ PASS: depth={depth:.3f}m (expected ~{expected}m, median filtered)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (expected ~{expected}m)")
        tests_failed += 1
    
    # Test 3: Out of range (too close)
    print("\nTest 3: Out of range - too close (<0.4m)")
    depth_img = np.full((480, 848), 200, dtype=np.uint16)  # 0.2m - too close
    depth = extract_depth_at_point(depth_img, 424, 240)
    if np.isnan(depth):
        print(f"  ✓ PASS: depth=NaN (correctly rejected, below min range)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (should be NaN)")
        tests_failed += 1
    
    # Test 4: Out of range (too far)
    print("\nTest 4: Out of range - too far (>6.0m)")
    depth_img = np.full((480, 848), 8000, dtype=np.uint16)  # 8.0m - too far
    depth = extract_depth_at_point(depth_img, 424, 240)
    if np.isnan(depth):
        print(f"  ✓ PASS: depth=NaN (correctly rejected, above max range)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (should be NaN)")
        tests_failed += 1
    
    # Test 5: All zeros (invalid depth)
    print("\nTest 5: Invalid depth data (all zeros)")
    depth_img = np.zeros((480, 848), dtype=np.uint16)
    depth = extract_depth_at_point(depth_img, 424, 240)
    if np.isnan(depth):
        print(f"  ✓ PASS: depth=NaN (correctly handled invalid data)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (should be NaN)")
        tests_failed += 1
    
    # Test 6: 3D projection - center pixel
    print("\nTest 6: 3D projection - image center (should be on optical axis)")
    depth_img = np.full((480, 848), 2000, dtype=np.uint16)  # 2.0m
    depth = extract_depth_at_point(depth_img, 640, 360)  # at principal point
    x, y, z = project_to_3d(640, 360, depth)
    if abs(x) < 0.01 and abs(y) < 0.01 and abs(z - 2.0) < 0.01:
        print(f"  ✓ PASS: 3D=({x:.3f}, {y:.3f}, {z:.3f})m (center should be ~(0,0,2))")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: 3D=({x:.3f}, {y:.3f}, {z:.3f})m (expected ~(0,0,2))")
        tests_failed += 1
    
    # Test 7: 3D projection - off-center pixel
    print("\nTest 7: 3D projection - off-center (100px right of center)")
    depth = 2.0  # 2 meters
    u, v = 740, 360  # 100 pixels right of center
    x, y, z = project_to_3d(u, v, depth)
    expected_x = (740 - 640) * 2.0 / 909.34  # ~0.22m
    if abs(x - expected_x) < 0.01 and abs(y) < 0.01 and abs(z - 2.0) < 0.01:
        print(f"  ✓ PASS: 3D=({x:.3f}, {y:.3f}, {z:.3f})m (x offset correct)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: 3D=({x:.3f}, {y:.3f}, {z:.3f})m (expected x~{expected_x:.3f})")
        tests_failed += 1
    
    # Test 8: Edge case - corner pixel
    print("\nTest 8: Edge case - top-left corner pixel")
    depth_img = np.full((480, 848), 1500, dtype=np.uint16)  # 1.5m
    depth = extract_depth_at_point(depth_img, 0, 0)
    if abs(depth - 1.5) < 0.01:
        print(f"  ✓ PASS: depth={depth:.3f}m (corner handled correctly)")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (expected 1.5m)")
        tests_failed += 1
    
    # Test 9: Realistic detection scenario
    print("\nTest 9: Realistic scenario - drone at 3.5m with some noise")
    depth_img = np.full((480, 848), 3500, dtype=np.uint16)  # 3.5m base
    # Add realistic sensor noise (±50mm)
    noise = np.random.randint(-50, 50, depth_img.shape).astype(np.int32)
    depth_img = np.clip(depth_img.astype(np.int32) + noise, 0, 65535).astype(np.uint16)
    # Simulate bbox center at (500, 300)
    depth = extract_depth_at_point(depth_img, 500, 300)
    x, y, z = project_to_3d(500, 300, depth)
    if 3.4 < depth < 3.6 and not np.isnan(x):
        print(f"  ✓ PASS: depth={depth:.3f}m, 3D=({x:.3f}, {y:.3f}, {z:.3f})m")
        tests_passed += 1
    else:
        print(f"  ✗ FAIL: depth={depth:.3f}m (expected ~3.5m)")
        tests_failed += 1
    
    # Summary
    print()
    print("=" * 60)
    total = tests_passed + tests_failed
    print(f"Results: {tests_passed}/{total} tests passed")
    if tests_failed == 0:
        print("✓ ALL TESTS PASSED - Depth extraction ready for integration!")
    else:
        print(f"✗ {tests_failed} tests failed")
    print("=" * 60)
    
    return tests_failed == 0


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
