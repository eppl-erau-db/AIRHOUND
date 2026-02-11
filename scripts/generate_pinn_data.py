#!/usr/bin/env python3
"""
Generate Synthetic Trajectory Data for PINN Training.

Produces CSV files in PINN_DATA_FORMAT.md format: [t, x, y, z, vx, vy, vz]
with configurable motion patterns, noise, and dropout simulation.

This enables PINN development and validation before real rosbag data is
available from SITL or hardware flights.

Motion patterns:
    - circular:    Horizontal circle at fixed depth
    - sinusoidal:  Sinusoidal oscillation in one axis
    - figure8:     Figure-eight pattern in XY plane
    - linear:      Constant velocity straight line
    - approach:    Target moving toward/away from camera (Z-axis)
    - diagonal:    3D diagonal motion (combined axes)
    - random_walk: Smoothed random walk (Ornstein-Uhlenbeck)

Usage:
    python3 scripts/generate_pinn_data.py -o data/pinn_training/synthetic.csv
    python3 scripts/generate_pinn_data.py -o data/pinn_training/synthetic.csv \
        --patterns circular figure8 approach --duration 30 --noise 0.02
"""

import argparse
import csv
import os
import sys
from typing import List, Tuple

import numpy as np


def generate_circular(
    duration: float = 10.0,
    fps: float = 30.0,
    radius: float = 1.5,
    speed: float = 0.8,
    center: Tuple[float, float, float] = (0.0, 0.0, 3.0),
) -> np.ndarray:
    """Circular motion in XY plane at fixed depth."""
    dt = 1.0 / fps
    t = np.arange(0, duration, dt)
    omega = speed  # rad/s

    x = center[0] + radius * np.cos(omega * t)
    y = center[1] + radius * np.sin(omega * t)
    z = np.full_like(t, center[2])

    vx = -radius * omega * np.sin(omega * t)
    vy = radius * omega * np.cos(omega * t)
    vz = np.zeros_like(t)

    return np.column_stack([t, x, y, z, vx, vy, vz])


def generate_sinusoidal(
    duration: float = 10.0,
    fps: float = 30.0,
    amplitude: float = 1.0,
    freq: float = 0.5,
    axis: str = 'x',
    center: Tuple[float, float, float] = (0.0, 0.0, 3.0),
) -> np.ndarray:
    """Sinusoidal oscillation along one axis."""
    dt = 1.0 / fps
    t = np.arange(0, duration, dt)
    omega = 2 * np.pi * freq

    displacement = amplitude * np.sin(omega * t)
    velocity = amplitude * omega * np.cos(omega * t)

    x = np.full_like(t, center[0])
    y = np.full_like(t, center[1])
    z = np.full_like(t, center[2])
    vx = np.zeros_like(t)
    vy = np.zeros_like(t)
    vz = np.zeros_like(t)

    if axis == 'x':
        x += displacement
        vx = velocity
    elif axis == 'y':
        y += displacement
        vy = velocity
    else:
        z += displacement
        vz = velocity

    return np.column_stack([t, x, y, z, vx, vy, vz])


def generate_figure8(
    duration: float = 10.0,
    fps: float = 30.0,
    scale: float = 1.5,
    speed: float = 0.5,
    center: Tuple[float, float, float] = (0.0, 0.0, 3.0),
) -> np.ndarray:
    """Figure-eight (lemniscate) in XY plane."""
    dt = 1.0 / fps
    t = np.arange(0, duration, dt)
    omega = speed

    x = center[0] + scale * np.sin(omega * t)
    y = center[1] + scale * np.sin(omega * t) * np.cos(omega * t)
    z = np.full_like(t, center[2])

    vx = scale * omega * np.cos(omega * t)
    vy = scale * omega * (np.cos(omega * t)**2 - np.sin(omega * t)**2)
    vz = np.zeros_like(t)

    return np.column_stack([t, x, y, z, vx, vy, vz])


def generate_linear(
    duration: float = 10.0,
    fps: float = 30.0,
    velocity: Tuple[float, float, float] = (1.0, 0.5, 0.0),
    start: Tuple[float, float, float] = (-2.0, -1.0, 3.0),
) -> np.ndarray:
    """Constant velocity straight line."""
    dt = 1.0 / fps
    t = np.arange(0, duration, dt)

    x = start[0] + velocity[0] * t
    y = start[1] + velocity[1] * t
    z = start[2] + velocity[2] * t

    vx = np.full_like(t, velocity[0])
    vy = np.full_like(t, velocity[1])
    vz = np.full_like(t, velocity[2])

    return np.column_stack([t, x, y, z, vx, vy, vz])


def generate_approach(
    duration: float = 10.0,
    fps: float = 30.0,
    z_start: float = 5.0,
    z_end: float = 1.5,
    center_xy: Tuple[float, float] = (0.0, 0.0),
) -> np.ndarray:
    """Target approaching/retreating along Z (depth) axis."""
    dt = 1.0 / fps
    t = np.arange(0, duration, dt)

    # Smooth approach (sinusoidal easing)
    progress = 0.5 * (1 - np.cos(np.pi * t / duration))
    z = z_start + (z_end - z_start) * progress
    vz = (z_end - z_start) * np.pi / (2 * duration) * np.sin(np.pi * t / duration)

    x = np.full_like(t, center_xy[0])
    y = np.full_like(t, center_xy[1])
    vx = np.zeros_like(t)
    vy = np.zeros_like(t)

    return np.column_stack([t, x, y, z, vx, vy, vz])


def generate_diagonal(
    duration: float = 10.0,
    fps: float = 30.0,
    speed: float = 1.0,
    direction: Tuple[float, float, float] = (1.0, 0.5, -0.3),
    start: Tuple[float, float, float] = (-1.0, -0.5, 4.0),
) -> np.ndarray:
    """3D diagonal motion (constant velocity in arbitrary direction)."""
    d = np.array(direction, dtype=np.float64)
    d_norm = d / (np.linalg.norm(d) + 1e-8) * speed

    return generate_linear(
        duration=duration, fps=fps,
        velocity=tuple(d_norm), start=start,
    )


def generate_random_walk(
    duration: float = 10.0,
    fps: float = 30.0,
    theta: float = 2.0,
    sigma: float = 1.5,
    center: Tuple[float, float, float] = (0.0, 0.0, 3.0),
) -> np.ndarray:
    """Smoothed random walk (Ornstein-Uhlenbeck process)."""
    dt = 1.0 / fps
    n = int(duration / dt)
    t = np.arange(n) * dt

    # OU process for velocity
    vel = np.zeros((n, 3))
    for i in range(1, n):
        vel[i] = vel[i-1] - theta * vel[i-1] * dt + sigma * np.sqrt(dt) * np.random.randn(3)

    # Integrate velocity to get position
    pos = np.zeros((n, 3))
    pos[0] = center
    for i in range(1, n):
        pos[i] = pos[i-1] + vel[i] * dt

    return np.column_stack([t, pos, vel])


def add_noise(data: np.ndarray, pos_noise: float = 0.02, vel_noise: float = 0.05) -> np.ndarray:
    """Add Gaussian noise to positions and velocities (not time)."""
    noisy = data.copy()
    n = len(data)

    # Position noise (columns 1-3)
    noisy[:, 1:4] += np.random.randn(n, 3) * pos_noise

    # Velocity noise (columns 4-6)
    noisy[:, 4:7] += np.random.randn(n, 3) * vel_noise

    return noisy


def add_dropouts(data: np.ndarray, dropout_rate: float = 0.05, max_gap: int = 15) -> np.ndarray:
    """Simulate detection dropouts by removing random contiguous chunks."""
    mask = np.ones(len(data), dtype=bool)
    i = 0
    while i < len(data):
        if np.random.random() < dropout_rate:
            gap = np.random.randint(1, max_gap + 1)
            mask[i:i+gap] = False
            i += gap
        else:
            i += 1
    return data[mask]


PATTERN_GENERATORS = {
    'circular': generate_circular,
    'sinusoidal': generate_sinusoidal,
    'figure8': generate_figure8,
    'linear': generate_linear,
    'approach': generate_approach,
    'diagonal': generate_diagonal,
    'random_walk': generate_random_walk,
}


def generate_dataset(
    patterns: List[str],
    duration: float = 10.0,
    fps: float = 30.0,
    repeats: int = 3,
    noise: float = 0.02,
    dropout_rate: float = 0.0,
    seed: int = 42,
) -> np.ndarray:
    """Generate a combined dataset from multiple patterns with variations."""
    np.random.seed(seed)
    all_trajectories = []
    time_offset = 0.0

    for pattern in patterns:
        gen_fn = PATTERN_GENERATORS[pattern]

        for rep in range(repeats):
            # Vary parameters slightly for each repeat
            kwargs = {'duration': duration, 'fps': fps}

            if pattern == 'circular':
                kwargs['radius'] = 1.0 + np.random.uniform(-0.3, 0.5)
                kwargs['speed'] = 0.5 + np.random.uniform(-0.2, 0.5)
                kwargs['center'] = (
                    np.random.uniform(-0.5, 0.5),
                    np.random.uniform(-0.3, 0.3),
                    np.random.uniform(2.0, 5.0),
                )
            elif pattern == 'sinusoidal':
                kwargs['amplitude'] = 0.5 + np.random.uniform(0, 1.0)
                kwargs['freq'] = 0.3 + np.random.uniform(0, 0.5)
                kwargs['axis'] = np.random.choice(['x', 'y', 'z'])
                kwargs['center'] = (0, 0, np.random.uniform(2.0, 5.0))
            elif pattern == 'figure8':
                kwargs['scale'] = 1.0 + np.random.uniform(-0.3, 0.5)
                kwargs['speed'] = 0.3 + np.random.uniform(0, 0.4)
            elif pattern == 'linear':
                speed = np.random.uniform(0.5, 3.0)
                direction = np.random.randn(3)
                direction = direction / np.linalg.norm(direction) * speed
                kwargs['velocity'] = tuple(direction)
                kwargs['start'] = (
                    np.random.uniform(-2, 2),
                    np.random.uniform(-1, 1),
                    np.random.uniform(2, 5),
                )
            elif pattern == 'approach':
                kwargs['z_start'] = np.random.uniform(3.0, 5.5)
                kwargs['z_end'] = np.random.uniform(1.0, 2.5)
            elif pattern == 'diagonal':
                kwargs['speed'] = np.random.uniform(0.5, 2.5)
                kwargs['direction'] = tuple(np.random.randn(3))
            elif pattern == 'random_walk':
                kwargs['theta'] = np.random.uniform(1.0, 3.0)
                kwargs['sigma'] = np.random.uniform(0.5, 2.0)
                kwargs['center'] = (0, 0, np.random.uniform(2.5, 4.5))

            traj = gen_fn(**kwargs)

            # Add noise
            if noise > 0:
                traj = add_noise(traj, pos_noise=noise, vel_noise=noise * 2)

            # Add dropouts
            if dropout_rate > 0:
                traj = add_dropouts(traj, dropout_rate=dropout_rate)

            # Offset time for concatenation (each trajectory starts at 0)
            traj[:, 0] += time_offset
            time_offset = traj[-1, 0] + 1.0 / fps  # gap between trajectories

            all_trajectories.append(traj)
            print(f"  {pattern} #{rep+1}: {len(traj)} points, "
                  f"{traj[-1,0]-traj[0,0]:.1f}s")

    combined = np.vstack(all_trajectories)
    return combined


def save_csv(data: np.ndarray, path: str):
    """Save trajectory data to CSV in PINN_DATA_FORMAT.md format."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
        for row in data:
            writer.writerow([f'{v:.6f}' for v in row])
    print(f"\nSaved {len(data)} points to {path}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate synthetic trajectory data for PINN training'
    )
    parser.add_argument('-o', '--output', required=True, help='Output CSV path')
    parser.add_argument(
        '--patterns', nargs='+',
        default=['circular', 'sinusoidal', 'figure8', 'linear', 'approach',
                 'diagonal', 'random_walk'],
        choices=list(PATTERN_GENERATORS.keys()),
        help='Motion patterns to generate',
    )
    parser.add_argument('--duration', type=float, default=10.0,
                        help='Duration per trajectory (s)')
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--repeats', type=int, default=3,
                        help='Repeats per pattern (with variation)')
    parser.add_argument('--noise', type=float, default=0.02,
                        help='Position noise std (m)')
    parser.add_argument('--dropout-rate', type=float, default=0.0,
                        help='Detection dropout probability per frame')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    print(f"Generating synthetic trajectories:")
    print(f"  Patterns: {args.patterns}")
    print(f"  Duration: {args.duration}s × {args.repeats} repeats each")
    print(f"  FPS: {args.fps}, Noise: {args.noise}m")
    print()

    data = generate_dataset(
        patterns=args.patterns,
        duration=args.duration,
        fps=args.fps,
        repeats=args.repeats,
        noise=args.noise,
        dropout_rate=args.dropout_rate,
        seed=args.seed,
    )

    save_csv(data, args.output)

    # Print stats
    print(f"\n--- Dataset Statistics ---")
    print(f"  Total points: {len(data)}")
    print(f"  Total duration: {data[-1, 0]:.1f}s")
    print(f"  X range: [{data[:,1].min():.2f}, {data[:,1].max():.2f}] m")
    print(f"  Y range: [{data[:,2].min():.2f}, {data[:,2].max():.2f}] m")
    print(f"  Z range: [{data[:,3].min():.2f}, {data[:,3].max():.2f}] m")
    v_mag = np.sqrt(data[:,4]**2 + data[:,5]**2 + data[:,6]**2)
    print(f"  Velocity: mean={v_mag.mean():.2f}, max={v_mag.max():.2f}, std={v_mag.std():.2f} m/s")


if __name__ == '__main__':
    main()
