#!/usr/bin/env python3
"""
Generate SPIE publication-quality system architecture diagram for AIRHOUND.

Shows the full pipeline:
  D455 Camera → Detection (RF-DETR/YOLOv8) → Depth Fusion → 3D Kalman Filter
  → PINN Prediction → 3D Control → PX4 Offboard

Output: manuscript/figures/system_architecture.{pdf,png}

Run with: /home/rylan/dev/research/EPPL/code/rf-detr-training/.venv/bin/python3 scripts/generate_architecture_diagram.py
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np
import os

# SPIE style
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 9,
    'text.usetex': False,
})

# Colorblind-safe palette (Okabe-Ito inspired, muted for architecture diagram)
COLORS = {
    'sensor':     '#4477AA',  # Blue - hardware/sensors
    'perception': '#EE6677',  # Red - detection
    'tracking':   '#228833',  # Green - tracking/estimation
    'prediction': '#AA3377',  # Purple - PINN
    'control':    '#CCBB44',  # Yellow - control
    'px4':        '#66CCEE',  # Cyan - PX4/hardware
    'data':       '#BBBBBB',  # Gray - data flow labels
    'bg':         '#FFFFFF',  # White background
    'border':     '#333333',  # Dark border
}


def draw_box(ax, x, y, w, h, label, sublabel, color, bold=False):
    """Draw a rounded box with label and sublabel."""
    box = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.05",
        facecolor=color,
        edgecolor=COLORS['border'],
        linewidth=1.2,
        alpha=0.85,
        zorder=2,
    )
    ax.add_patch(box)

    # Main label
    weight = 'bold' if bold else 'normal'
    ax.text(
        x + w / 2, y + h * 0.62,
        label,
        ha='center', va='center',
        fontsize=9, fontweight=weight,
        color='white' if color in [COLORS['sensor'], COLORS['perception'], COLORS['tracking'], COLORS['prediction'], COLORS['px4']] else '#222222',
        zorder=3,
    )
    # Sublabel (smaller, italic)
    if sublabel:
        ax.text(
            x + w / 2, y + h * 0.28,
            sublabel,
            ha='center', va='center',
            fontsize=7, fontstyle='italic',
            color='white' if color in [COLORS['sensor'], COLORS['perception'], COLORS['tracking'], COLORS['prediction'], COLORS['px4']] else '#444444',
            zorder=3,
        )


def draw_arrow(ax, x1, y1, x2, y2, label=None, color='#555555', style='->', curved=False):
    """Draw an arrow between two points with optional label."""
    if curved:
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle=style,
            mutation_scale=12,
            color=color,
            linewidth=1.3,
            connectionstyle="arc3,rad=0.2",
            zorder=1,
        )
    else:
        arrow = FancyArrowPatch(
            (x1, y1), (x2, y2),
            arrowstyle=style,
            mutation_scale=12,
            color=color,
            linewidth=1.3,
            zorder=1,
        )
    ax.add_patch(arrow)

    if label:
        mx = (x1 + x2) / 2
        my = (y1 + y2) / 2
        ax.text(
            mx, my + 0.08,
            label,
            ha='center', va='bottom',
            fontsize=6.5, color='#555555',
            fontstyle='italic',
            zorder=3,
            bbox=dict(boxstyle='round,pad=0.1', facecolor='white', edgecolor='none', alpha=0.8),
        )


def main():
    fig, ax = plt.subplots(1, 1, figsize=(12, 6.5))
    ax.set_xlim(-0.5, 12.5)
    ax.set_ylim(-0.8, 6.5)
    ax.set_aspect('equal')
    ax.axis('off')

    # Title
    ax.text(
        6.0, 6.2,
        'AIRHOUND System Architecture',
        ha='center', va='center',
        fontsize=14, fontweight='bold',
        color='#222222',
    )

    # Layout: boxes are positioned in a flow diagram
    bw = 2.0   # box width
    bh = 0.8   # box height

    # === ROW 1: Camera → Detection ===
    # D455 Camera
    cam_x, cam_y = 0.0, 4.4
    draw_box(ax, cam_x, cam_y, bw, bh,
             'Intel RealSense D455', 'RGB + Depth @ 30 Hz',
             COLORS['sensor'], bold=True)

    # RGB stream arrow (right)
    ax.annotate('', xy=(2.65, 4.85), xytext=(2.05, 4.85),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(2.35, 5.05, 'RGB', fontsize=7, ha='center', color='#555555', fontstyle='italic')

    # Depth stream arrow (down)
    ax.annotate('', xy=(1.0, 3.35), xytext=(1.0, 4.35),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(0.55, 3.85, 'Depth', fontsize=7, ha='center', color='#555555', fontstyle='italic')

    # Detection Node
    det_x, det_y = 2.8, 4.4
    draw_box(ax, det_x, det_y, bw, bh,
             'Detection Node', 'RF-DETR / YOLOv8 TensorRT',
             COLORS['perception'], bold=True)

    # Arrow: Detection → Depth Fusion (down)
    ax.annotate('', xy=(det_x + bw / 2, 3.4), xytext=(det_x + bw / 2, det_y - 0.05),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(det_x + bw / 2 + 0.25, 3.85, '2D bboxes', fontsize=6.5, ha='left', color='#555555', fontstyle='italic')

    # === ROW 2: Depth Fusion ===
    fus_x, fus_y = 1.5, 2.5
    draw_box(ax, fus_x, fus_y, bw + 0.3, bh,
             'Depth Fusion', 'bbox center + depth → 3D (x, y, z)',
             COLORS['tracking'], bold=True)

    # Arrow: Depth stream into fusion
    ax.annotate('', xy=(fus_x + 0.3, fus_y + bh), xytext=(1.0, 3.3),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.2))

    # Arrow: Fusion → Kalman
    ax.annotate('', xy=(4.55, fus_y + bh / 2), xytext=(fus_x + bw + 0.35, fus_y + bh / 2),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(4.2, fus_y + bh / 2 + 0.18, '3D position', fontsize=6.5, ha='center', color='#555555', fontstyle='italic')

    # === ROW 2: 3D Kalman Filter ===
    kf_x, kf_y = 4.6, 2.5
    draw_box(ax, kf_x, kf_y, bw, bh,
             '3D Kalman Filter', '[x, y, z, vx, vy, vz]',
             COLORS['tracking'], bold=True)

    # Arrow: Kalman → PINN
    ax.annotate('', xy=(7.35, kf_y + bh / 2), xytext=(kf_x + bw + 0.05, kf_y + bh / 2),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(6.7, kf_y + bh / 2 + 0.18, 'filtered state', fontsize=6.5, ha='center', color='#555555', fontstyle='italic')

    # === ROW 2: PINN ===
    pinn_x, pinn_y = 7.4, 2.5
    draw_box(ax, pinn_x, pinn_y, bw + 0.4, bh,
             'PINN Prediction', 'MLP, physics loss (|v| ≤ 10 m/s)',
             COLORS['prediction'], bold=True)

    # Arrow: PINN → Control (down)
    ax.annotate('', xy=(pinn_x + (bw + 0.4) / 2, 1.5), xytext=(pinn_x + (bw + 0.4) / 2, pinn_y - 0.05),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(pinn_x + (bw + 0.4) / 2 + 0.3, 2.0, 'predicted\n3D state', fontsize=6.5, ha='left', color='#555555', fontstyle='italic')

    # === ROW 3: 3D Control ===
    ctrl_x, ctrl_y = 5.2, 0.6
    ctrl_w = 2.6
    draw_box(ax, ctrl_x, ctrl_y, ctrl_w, bh,
             '3D Anticipatory Control', 'yaw + pitch viewing + altitude',
             COLORS['control'], bold=True)

    # Arrow: Control → PX4
    ax.annotate('', xy=(9.35, ctrl_y + bh / 2), xytext=(ctrl_x + ctrl_w + 0.05, ctrl_y + bh / 2),
                arrowprops=dict(arrowstyle='->', color='#555555', lw=1.3))
    ax.text(8.7, ctrl_y + bh / 2 + 0.18, 'TrajectorySetpoint', fontsize=6.5, ha='center', color='#555555', fontstyle='italic')

    # === ROW 3: PX4 ===
    px4_x, px4_y = 9.4, 0.6
    draw_box(ax, px4_x, px4_y, bw, bh,
             'PX4 Autopilot', 'Offboard via uXRCE-DDS',
             COLORS['px4'], bold=True)

    # (Jetson Orin / ROS 2 Humble labels removed for readability)

    # === Kalman → Control direct path (fallback when PINN is bypassed) ===
    ax.annotate('', xy=(ctrl_x + 0.3, ctrl_y + bh), xytext=(kf_x + bw / 2, kf_y - 0.05),
                arrowprops=dict(arrowstyle='->', color='#AAAAAA', lw=1.0, linestyle='dashed'))
    ax.text(4.7, 1.65, 'fallback\n(no PINN)', fontsize=6, ha='center', color='#AAAAAA', fontstyle='italic')

    # === Topics annotation (right side) ===
    topics = [
        '/camera/color/image_raw',
        '/camera/depth/image_raw',
        '/detections',
        '/perception/target_3d',
        '/yaw_command',
        '/fmu/in/trajectory_setpoint',
    ]
    topic_y = 5.6
    ax.text(11.5, topic_y + 0.2, 'Key ROS 2 Topics:', fontsize=7.5, fontweight='bold',
            ha='center', va='bottom', color='#555555')
    for i, t in enumerate(topics):
        ax.text(11.5, topic_y - i * 0.26, t, fontsize=6, ha='center', va='top',
                color='#666666', fontfamily='monospace')

    # === Legend ===
    legend_items = [
        ('Sensor/Hardware', COLORS['sensor']),
        ('Detection', COLORS['perception']),
        ('Tracking/Estimation', COLORS['tracking']),
        ('Prediction (PINN)', COLORS['prediction']),
        ('Control', COLORS['control']),
        ('Flight Controller', COLORS['px4']),
    ]
    for i, (label, color) in enumerate(legend_items):
        lx = 0.0 + (i % 3) * 1.8
        ly = -0.2 - (i // 3) * 0.3
        rect = FancyBboxPatch(
            (lx, ly), 0.2, 0.15,
            boxstyle="round,pad=0.02",
            facecolor=color, edgecolor=COLORS['border'],
            linewidth=0.5, alpha=0.85, zorder=2,
        )
        ax.add_patch(rect)
        ax.text(lx + 0.28, ly + 0.075, label, fontsize=6.5, va='center', color='#333333')

    plt.tight_layout()

    # Save
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'manuscript', 'figures')
    os.makedirs(out_dir, exist_ok=True)

    pdf_path = os.path.join(out_dir, 'system_architecture.pdf')
    png_path = os.path.join(out_dir, 'system_architecture.png')

    fig.savefig(pdf_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    fig.savefig(png_path, dpi=300, bbox_inches='tight', pad_inches=0.1)
    plt.close()

    print(f"Saved: {pdf_path}")
    print(f"Saved: {png_path}")


if __name__ == '__main__':
    main()
