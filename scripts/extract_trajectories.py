#!/usr/bin/env python3
"""
PINN Training Data Extractor for AIRHOUND.

Extracts 3D target trajectories from rosbags and converts them to CSV format
suitable for training the Physics-Informed Neural Network (PINN).

The output format contains timestamped 3D positions with computed velocities:
    t, x, y, z, vx, vy, vz

Usage:
    # Single rosbag
    python3 extract_trajectories.py /path/to/bag -o trajectory.csv
    
    # Multiple rosbags (combined into single dataset)
    python3 extract_trajectories.py bag1 bag2 bag3 -o combined_trajectories.csv
    
    # With velocity smoothing
    python3 extract_trajectories.py /path/to/bag -o trajectory.csv --smooth-window 5

Author: Perception Lead
Date: January 2026
"""

import argparse
import sys
from pathlib import Path
from typing import List, Tuple, Optional
from dataclasses import dataclass
import csv

import numpy as np

# ROS2 bag reading
try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    ROSBAGS_AVAILABLE = True
except ImportError:
    ROSBAGS_AVAILABLE = False
    print("Warning: rosbags package not installed. Install with: pip install rosbags")


@dataclass
class TrajectoryPoint:
    """Single point in a trajectory."""
    t: float  # Time in seconds (relative to start)
    x: float  # X position in meters
    y: float  # Y position in meters
    z: float  # Z position in meters (depth)
    vx: float = 0.0  # X velocity in m/s
    vy: float = 0.0  # Y velocity in m/s
    vz: float = 0.0  # Z velocity in m/s


def read_rosbag_trajectories(
    bag_path: Path,
    topic: str = '/perception/target_3d',
    alt_topics: Optional[List[str]] = None
) -> List[Tuple[float, float, float, float]]:
    """
    Read 3D positions from a rosbag.
    
    Returns list of (timestamp_sec, x, y, z) tuples.
    """
    if not ROSBAGS_AVAILABLE:
        raise ImportError("rosbags package required. Install with: pip install rosbags")
    
    if alt_topics is None:
        alt_topics = [
            '/perception/target_3d',
            '/target_3d',
            '/tracking/target_position',
        ]
    
    topics_to_try = [topic] + [t for t in alt_topics if t != topic]
    
    positions = []
    
    with Reader(bag_path) as reader:
        # Find available topics
        available_topics = {conn.topic for conn in reader.connections}
        
        # Find first matching topic
        target_topic = None
        for t in topics_to_try:
            if t in available_topics:
                target_topic = t
                break
        
        if target_topic is None:
            print(f"Warning: No target position topic found in {bag_path}")
            print(f"  Available topics: {available_topics}")
            return []
        
        print(f"  Reading from topic: {target_topic}")
        
        # Read messages
        connections = [c for c in reader.connections if c.topic == target_topic]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            
            # Handle PointStamped
            if hasattr(msg, 'point'):
                x, y, z = msg.point.x, msg.point.y, msg.point.z
            # Handle Point
            elif hasattr(msg, 'x'):
                x, y, z = msg.x, msg.y, msg.z
            else:
                continue
            
            # Skip invalid points
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue
            
            # Convert timestamp (nanoseconds to seconds)
            t_sec = timestamp * 1e-9
            positions.append((t_sec, x, y, z))
    
    return positions


def compute_velocities(
    positions: List[Tuple[float, float, float, float]],
    smooth_window: int = 1
) -> List[TrajectoryPoint]:
    """
    Compute velocities from positions using finite differences.
    
    Args:
        positions: List of (t, x, y, z) tuples
        smooth_window: Window size for velocity smoothing (1 = no smoothing)
    
    Returns:
        List of TrajectoryPoint with velocities
    """
    if len(positions) < 2:
        return []
    
    # Sort by time
    positions = sorted(positions, key=lambda p: p[0])
    
    # Normalize time to start at 0
    t0 = positions[0][0]
    positions = [(t - t0, x, y, z) for t, x, y, z in positions]
    
    # Compute raw velocities via finite differences
    raw_velocities = []
    for i in range(len(positions)):
        if i == 0:
            # Forward difference for first point
            dt = positions[1][0] - positions[0][0]
            if dt > 0:
                vx = (positions[1][1] - positions[0][1]) / dt
                vy = (positions[1][2] - positions[0][2]) / dt
                vz = (positions[1][3] - positions[0][3]) / dt
            else:
                vx, vy, vz = 0.0, 0.0, 0.0
        elif i == len(positions) - 1:
            # Backward difference for last point
            dt = positions[-1][0] - positions[-2][0]
            if dt > 0:
                vx = (positions[-1][1] - positions[-2][1]) / dt
                vy = (positions[-1][2] - positions[-2][2]) / dt
                vz = (positions[-1][3] - positions[-2][3]) / dt
            else:
                vx, vy, vz = 0.0, 0.0, 0.0
        else:
            # Central difference
            dt = positions[i+1][0] - positions[i-1][0]
            if dt > 0:
                vx = (positions[i+1][1] - positions[i-1][1]) / dt
                vy = (positions[i+1][2] - positions[i-1][2]) / dt
                vz = (positions[i+1][3] - positions[i-1][3]) / dt
            else:
                vx, vy, vz = 0.0, 0.0, 0.0
        
        raw_velocities.append((vx, vy, vz))
    
    # Apply smoothing if requested
    if smooth_window > 1:
        smoothed_velocities = []
        half_window = smooth_window // 2
        for i in range(len(raw_velocities)):
            start = max(0, i - half_window)
            end = min(len(raw_velocities), i + half_window + 1)
            window = raw_velocities[start:end]
            avg_vx = np.mean([v[0] for v in window])
            avg_vy = np.mean([v[1] for v in window])
            avg_vz = np.mean([v[2] for v in window])
            smoothed_velocities.append((avg_vx, avg_vy, avg_vz))
        raw_velocities = smoothed_velocities
    
    # Build trajectory points
    trajectory = []
    for i, (t, x, y, z) in enumerate(positions):
        vx, vy, vz = raw_velocities[i]
        trajectory.append(TrajectoryPoint(t=t, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz))
    
    return trajectory


def write_csv(trajectory: List[TrajectoryPoint], output_path: Path):
    """Write trajectory to CSV file."""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
        for pt in trajectory:
            writer.writerow([
                f'{pt.t:.6f}',
                f'{pt.x:.6f}',
                f'{pt.y:.6f}',
                f'{pt.z:.6f}',
                f'{pt.vx:.6f}',
                f'{pt.vy:.6f}',
                f'{pt.vz:.6f}'
            ])


def generate_statistics(trajectory: List[TrajectoryPoint]) -> dict:
    """Generate statistics about the trajectory."""
    if not trajectory:
        return {}
    
    positions = np.array([[pt.x, pt.y, pt.z] for pt in trajectory])
    velocities = np.array([[pt.vx, pt.vy, pt.vz] for pt in trajectory])
    
    v_magnitudes = np.linalg.norm(velocities, axis=1)
    
    return {
        'num_points': len(trajectory),
        'duration_sec': trajectory[-1].t - trajectory[0].t,
        'fps': len(trajectory) / max(0.001, trajectory[-1].t - trajectory[0].t),
        'x_range': (float(positions[:, 0].min()), float(positions[:, 0].max())),
        'y_range': (float(positions[:, 1].min()), float(positions[:, 1].max())),
        'z_range': (float(positions[:, 2].min()), float(positions[:, 2].max())),
        'v_mean': float(v_magnitudes.mean()),
        'v_max': float(v_magnitudes.max()),
        'v_std': float(v_magnitudes.std()),
    }


def main():
    parser = argparse.ArgumentParser(
        description='Extract PINN training trajectories from rosbags',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python3 extract_trajectories.py flight1.bag -o trajectories/flight1.csv
    python3 extract_trajectories.py data/*.bag -o combined_training.csv --smooth-window 5
        """
    )
    parser.add_argument('bags', nargs='+', type=Path,
                        help='Rosbag directories to process')
    parser.add_argument('-o', '--output', type=Path, required=True,
                        help='Output CSV file path')
    parser.add_argument('--topic', type=str, default='/perception/target_3d',
                        help='Topic to read 3D positions from')
    parser.add_argument('--smooth-window', type=int, default=1,
                        help='Window size for velocity smoothing (1=no smoothing)')
    parser.add_argument('--min-points', type=int, default=10,
                        help='Minimum points required per trajectory segment')
    parser.add_argument('--stats', action='store_true',
                        help='Print trajectory statistics')
    
    args = parser.parse_args()
    
    if not ROSBAGS_AVAILABLE:
        print("Error: rosbags package not installed.")
        print("Install with: pip install rosbags")
        sys.exit(1)
    
    # Process all bags
    all_positions = []
    for bag_path in args.bags:
        if not bag_path.exists():
            print(f"Warning: Bag not found: {bag_path}")
            continue
        
        print(f"Processing: {bag_path}")
        positions = read_rosbag_trajectories(bag_path, args.topic)
        print(f"  Found {len(positions)} points")
        all_positions.extend(positions)
    
    if len(all_positions) < args.min_points:
        print(f"Error: Only {len(all_positions)} points found (minimum: {args.min_points})")
        sys.exit(1)
    
    # Compute velocities
    print(f"\nComputing velocities (smooth_window={args.smooth_window})...")
    trajectory = compute_velocities(all_positions, args.smooth_window)
    
    # Write output
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_csv(trajectory, args.output)
    print(f"Wrote {len(trajectory)} points to {args.output}")
    
    # Statistics
    if args.stats:
        stats = generate_statistics(trajectory)
        print("\n--- Trajectory Statistics ---")
        print(f"  Points:     {stats['num_points']}")
        print(f"  Duration:   {stats['duration_sec']:.2f}s")
        print(f"  FPS:        {stats['fps']:.1f}")
        print(f"  X range:    [{stats['x_range'][0]:.2f}, {stats['x_range'][1]:.2f}]m")
        print(f"  Y range:    [{stats['y_range'][0]:.2f}, {stats['y_range'][1]:.2f}]m")
        print(f"  Z range:    [{stats['z_range'][0]:.2f}, {stats['z_range'][1]:.2f}]m")
        print(f"  Velocity:   mean={stats['v_mean']:.2f}, max={stats['v_max']:.2f}, std={stats['v_std']:.2f} m/s")
    
    print("\nDone!")


if __name__ == '__main__':
    main()
