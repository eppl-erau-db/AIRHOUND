#!/usr/bin/env python3
"""
Generate SPIE Defense + Security 2026 publication-quality figures.

Paper: AIRHOUND - Predictive Target Pursuit for Autonomous UAVs using RF-DETR
       with Depth-Aware State Estimation and Physics-Informed Trajectory Prediction
Track: DS112 - Machine Learning from Challenging Data

Generates figures for the RF-DETR Training & Optimization section:
  1. degradation_comparison.pdf   - 4-panel mAP vs severity (MAIN FIGURE)
  2. advantage_heatmap.pdf        - RF-DETR advantage heatmap
  3. severe_bar_chart.pdf         - Bar chart at severity >= 0.75
  4. retention_curve.pdf          - % clean performance retained
  5. clean_baseline_table.tex     - LaTeX table for clean baseline metrics

Usage:
    python3 generate_spie_figures.py

Author: Rylan (Perception Lead)
Date: February 2026
"""

import json
from pathlib import Path
from typing import Dict, List
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, FormatStrFormatter
import matplotlib.gridspec as gridspec

# ============================================================================
# SPIE Proceedings Style
# ============================================================================
# SPIE single-column width: 7.5 in (full page width for proceedings)
# Typical figure widths: 3.5 in (half), 5.0 in (2/3), 7.5 in (full)
# Font: Times/serif, 8-10pt for figure text

SPIE_STYLE = {
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 9,
    'axes.labelsize': 10,
    'axes.titlesize': 10,
    'axes.titleweight': 'bold',
    'legend.fontsize': 8,
    'legend.framealpha': 0.9,
    'legend.edgecolor': '0.8',
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
    'axes.grid': True,
    'grid.alpha': 0.25,
    'grid.linewidth': 0.5,
    'lines.linewidth': 1.8,
    'lines.markersize': 7,
    'axes.linewidth': 0.8,
    'xtick.major.width': 0.8,
    'ytick.major.width': 0.8,
    'text.usetex': False,  # Set True if LaTeX is available
}

# Colorblind-safe palette (Okabe-Ito derived)
COLORS = {
    'yolov8': '#0072B2',   # Blue
    'rfdetr': '#D55E00',   # Vermillion
    'advantage': '#009E73', # Teal (for delta plots)
}

MARKERS = {
    'yolov8': 'o',
    'rfdetr': 's',
}

LINESTYLES = {
    'yolov8': '-',
    'rfdetr': '-',
}

MODEL_LABELS = {
    'yolov8': 'YOLOv8n',
    'rfdetr': 'RF-DETR',
}

DEG_LABELS = {
    'gaussian_noise': 'Gaussian Noise',
    'motion_blur': 'Motion Blur',
    'low_light': 'Low Light',
    'occlusion': 'Occlusion',
}

DEG_ORDER = ['gaussian_noise', 'low_light', 'motion_blur', 'occlusion']

# ============================================================================
# Data Loading
# ============================================================================

DATA_PATH = Path(__file__).parent.parent / 'data' / 'analysis' / 'evaluation_results.json'
OUT_DIR = Path(__file__).parent.parent / 'manuscript' / 'figures'
TABLE_DIR = Path(__file__).parent.parent / 'manuscript' / 'tables'


def load_data() -> Dict:
    """Load and deduplicate evaluation results, organized by degradation/model/severity."""
    with open(DATA_PATH) as f:
        raw = json.load(f)

    seen = set()
    unique = []
    for r in raw:
        key = (r['model'], r['degradation_type'], r['severity'])
        if key not in seen:
            seen.add(key)
            unique.append(r)

    # Organize: {deg: {model: {sev: record}}}
    organized = {}
    for r in unique:
        deg = r['degradation_type']
        model = r['model']
        sev = r['severity']
        organized.setdefault(deg, {}).setdefault(model, {})[sev] = r

    return organized


# ============================================================================
# Figure 1: Main 4-Panel Degradation Comparison
# ============================================================================

def fig_degradation_comparison(data: Dict, out_dir: Path):
    """
    Main paper figure: 2x2 grid showing mAP@0.5 vs severity for each
    degradation type, comparing RF-DETR and YOLOv8.
    """
    plt.rcParams.update(SPIE_STYLE)

    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.0), sharey=True, sharex=True)
    axes = axes.flatten()

    for ax, deg in zip(axes, DEG_ORDER):
        if deg not in data:
            continue

        for model in ['yolov8', 'rfdetr']:
            if model not in data[deg]:
                continue

            model_data = data[deg][model]
            sevs = sorted(model_data.keys())
            maps = [model_data[s]['mAP_0_5'] for s in sevs]

            ax.plot(sevs, maps,
                    marker=MARKERS[model],
                    color=COLORS[model],
                    linestyle=LINESTYLES[model],
                    label=MODEL_LABELS[model],
                    markeredgecolor='white',
                    markeredgewidth=0.5,
                    zorder=3)

        # Shade the region where RF-DETR > YOLOv8
        if 'yolov8' in data[deg] and 'rfdetr' in data[deg]:
            sevs = sorted(data[deg]['yolov8'].keys())
            yolo_maps = [data[deg]['yolov8'][s]['mAP_0_5'] for s in sevs]
            rfdetr_maps = [data[deg]['rfdetr'][s]['mAP_0_5'] for s in sevs]
            ax.fill_between(sevs, yolo_maps, rfdetr_maps,
                           alpha=0.08, color=COLORS['rfdetr'],
                           where=[r >= y for r, y in zip(rfdetr_maps, yolo_maps)])

        ax.set_title(DEG_LABELS.get(deg, deg))
        ax.set_xlim(-0.03, 1.03)
        ax.set_ylim(0, 1.02)
        ax.xaxis.set_major_locator(MultipleLocator(0.25))
        ax.yaxis.set_major_locator(MultipleLocator(0.2))

    # Shared labels
    for ax in axes[2:]:
        ax.set_xlabel('Severity Level')
    for ax in [axes[0], axes[2]]:
        ax.set_ylabel('mAP@0.5')

    # Single legend at top
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=2,
               bbox_to_anchor=(0.5, 1.0), frameon=True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    for ext in ['pdf', 'png']:
        fig.savefig(out_dir / f'degradation_comparison.{ext}')
    print(f"  Saved: degradation_comparison.pdf/png")
    plt.close(fig)


# ============================================================================
# Figure 2: RF-DETR Advantage Heatmap
# ============================================================================

def fig_advantage_heatmap(data: Dict, out_dir: Path):
    """
    Heatmap showing mAP difference (RF-DETR - YOLOv8) for every
    degradation x severity combination. Positive = RF-DETR wins.
    """
    plt.rcParams.update(SPIE_STYLE)

    severities = [0.0, 0.25, 0.5, 0.75, 1.0]
    degs = [d for d in DEG_ORDER if d in data]

    diff_matrix = np.full((len(degs), len(severities)), np.nan)
    for i, deg in enumerate(degs):
        for j, sev in enumerate(severities):
            y = data[deg].get('yolov8', {}).get(sev, {}).get('mAP_0_5')
            r = data[deg].get('rfdetr', {}).get(sev, {}).get('mAP_0_5')
            if y is not None and r is not None:
                diff_matrix[i, j] = r - y

    fig, ax = plt.subplots(figsize=(5.0, 2.8))

    vmax = max(0.3, np.nanmax(np.abs(diff_matrix)))
    im = ax.imshow(diff_matrix, cmap='RdYlGn', aspect='auto',
                   vmin=-vmax, vmax=vmax)

    ax.set_xticks(range(len(severities)))
    ax.set_xticklabels([f'{s:.2f}' for s in severities])
    ax.set_yticks(range(len(degs)))
    ax.set_yticklabels([DEG_LABELS.get(d, d) for d in degs])

    ax.set_xlabel('Severity Level')

    cbar = plt.colorbar(im, ax=ax, shrink=0.9, pad=0.02)
    cbar.set_label(r'$\Delta$mAP (RF-DETR $-$ YOLOv8)', fontsize=9)

    # Annotate cells
    for i in range(len(degs)):
        for j in range(len(severities)):
            val = diff_matrix[i, j]
            if not np.isnan(val):
                color = 'white' if abs(val) > 0.2 else 'black'
                sign = '+' if val > 0 else ''
                ax.text(j, i, f'{sign}{val:.3f}', ha='center', va='center',
                        color=color, fontsize=7, fontweight='bold')

    plt.tight_layout()

    for ext in ['pdf', 'png']:
        fig.savefig(out_dir / f'advantage_heatmap.{ext}')
    print(f"  Saved: advantage_heatmap.pdf/png")
    plt.close(fig)


# ============================================================================
# Figure 3: Severe Degradation Bar Chart
# ============================================================================

def fig_severe_bar_chart(data: Dict, out_dir: Path):
    """
    Grouped bar chart showing average mAP at severity >= 0.75
    for each degradation type.
    """
    plt.rcParams.update(SPIE_STYLE)

    degs = [d for d in DEG_ORDER if d in data]
    severe_sevs = [0.75, 1.0]

    yolo_avgs = []
    rfdetr_avgs = []

    for deg in degs:
        yolo_vals = [data[deg].get('yolov8', {}).get(s, {}).get('mAP_0_5', np.nan)
                     for s in severe_sevs]
        rfdetr_vals = [data[deg].get('rfdetr', {}).get(s, {}).get('mAP_0_5', np.nan)
                       for s in severe_sevs]
        yolo_avgs.append(np.nanmean(yolo_vals))
        rfdetr_avgs.append(np.nanmean(rfdetr_vals))

    fig, ax = plt.subplots(figsize=(5.0, 3.0))

    x = np.arange(len(degs))
    width = 0.35

    bars_yolo = ax.bar(x - width/2, yolo_avgs, width,
                       label=MODEL_LABELS['yolov8'],
                       color=COLORS['yolov8'], edgecolor='white', linewidth=0.5)
    bars_rfdetr = ax.bar(x + width/2, rfdetr_avgs, width,
                         label=MODEL_LABELS['rfdetr'],
                         color=COLORS['rfdetr'], edgecolor='white', linewidth=0.5)

    # Value labels on bars
    for bar, val in zip(bars_yolo, yolo_avgs):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.015,
                f'{val:.2f}', ha='center', va='bottom', fontsize=7)
    for bar, val in zip(bars_rfdetr, rfdetr_avgs):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.015,
                f'{val:.2f}', ha='center', va='bottom', fontsize=7)

    ax.set_xticks(x)
    ax.set_xticklabels([DEG_LABELS.get(d, d) for d in degs], fontsize=8)
    ax.set_ylabel('Mean mAP@0.5 (severity $\\geq$ 0.75)')
    ax.set_ylim(0, 1.0)
    ax.yaxis.set_major_locator(MultipleLocator(0.2))
    ax.legend(loc='upper right', framealpha=0.9)

    plt.tight_layout()

    for ext in ['pdf', 'png']:
        fig.savefig(out_dir / f'severe_bar_chart.{ext}')
    print(f"  Saved: severe_bar_chart.pdf/png")
    plt.close(fig)


# ============================================================================
# Figure 4: Performance Retention Curve
# ============================================================================

def fig_retention_curve(data: Dict, out_dir: Path):
    """
    Shows % of clean-baseline mAP retained at each severity level.
    Highlights how RF-DETR degrades more gracefully.
    """
    plt.rcParams.update(SPIE_STYLE)

    # Focus on the two strongest differentiators
    focus_degs = ['gaussian_noise', 'low_light']

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.0), sharey=True)

    for ax, deg in zip(axes, focus_degs):
        if deg not in data:
            continue

        for model in ['yolov8', 'rfdetr']:
            if model not in data[deg]:
                continue

            model_data = data[deg][model]
            clean_map = model_data.get(0.0, {}).get('mAP_0_5', 1.0)
            if clean_map == 0:
                continue

            sevs = sorted(model_data.keys())
            retention = [(model_data[s]['mAP_0_5'] / clean_map) * 100 for s in sevs]

            ax.plot(sevs, retention,
                    marker=MARKERS[model],
                    color=COLORS[model],
                    label=MODEL_LABELS[model],
                    markeredgecolor='white',
                    markeredgewidth=0.5)

        # Reference lines
        ax.axhline(y=50, color='gray', linestyle='--', alpha=0.4, linewidth=0.8)
        ax.text(1.02, 50, '50%', va='center', fontsize=7, color='gray')

        ax.set_title(DEG_LABELS.get(deg, deg))
        ax.set_xlabel('Severity Level')
        ax.set_xlim(-0.03, 1.03)
        ax.set_ylim(0, 105)
        ax.xaxis.set_major_locator(MultipleLocator(0.25))
        ax.yaxis.set_major_locator(MultipleLocator(20))

    axes[0].set_ylabel('Performance Retained (%)')
    axes[-1].legend(loc='lower left', framealpha=0.9)

    plt.tight_layout()

    for ext in ['pdf', 'png']:
        fig.savefig(out_dir / f'retention_curve.{ext}')
    print(f"  Saved: retention_curve.pdf/png")
    plt.close(fig)


# ============================================================================
# Figure 5: Precision-Recall Trade-off Under Degradation
# ============================================================================

def fig_precision_recall(data: Dict, out_dir: Path):
    """
    Scatter plot of precision vs recall colored by model,
    sized by severity. Shows RF-DETR maintains better P-R balance.
    """
    plt.rcParams.update(SPIE_STYLE)

    fig, ax = plt.subplots(figsize=(4.5, 4.0))

    for deg in DEG_ORDER:
        if deg not in data:
            continue
        for model in ['yolov8', 'rfdetr']:
            if model not in data[deg]:
                continue
            for sev, rec in data[deg][model].items():
                if sev == 0.0:
                    continue  # skip clean
                size = 20 + 60 * sev
                alpha = 0.4 + 0.5 * sev
                ax.scatter(rec['recall'], rec['precision'],
                          marker=MARKERS[model],
                          color=COLORS[model],
                          s=size, alpha=alpha,
                          edgecolors='white', linewidths=0.3)

    # Clean baseline points (larger, distinct)
    for model in ['yolov8', 'rfdetr']:
        for deg in DEG_ORDER:
            if deg in data and model in data[deg] and 0.0 in data[deg][model]:
                rec = data[deg][model][0.0]
                ax.scatter(rec['recall'], rec['precision'],
                          marker=MARKERS[model],
                          color=COLORS[model],
                          s=120, alpha=1.0,
                          edgecolors='black', linewidths=1.0, zorder=5)
                break  # only plot clean baseline once per model

    # Legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['yolov8'],
               markersize=8, label=MODEL_LABELS['yolov8']),
        Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS['rfdetr'],
               markersize=8, label=MODEL_LABELS['rfdetr']),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='gray',
               markersize=5, label='Low severity', alpha=0.5),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='gray',
               markersize=10, label='High severity'),
    ]
    ax.legend(handles=legend_elements, loc='lower left', fontsize=7)

    ax.set_xlabel('Recall')
    ax.set_ylabel('Precision')
    ax.set_xlim(0, 1.02)
    ax.set_ylim(0, 1.02)
    ax.set_aspect('equal')

    plt.tight_layout()

    for ext in ['pdf', 'png']:
        fig.savefig(out_dir / f'precision_recall.{ext}')
    print(f"  Saved: precision_recall.pdf/png")
    plt.close(fig)


# ============================================================================
# Table: Clean Baseline Comparison (LaTeX)
# ============================================================================

def table_clean_baseline(data: Dict, table_dir: Path):
    """Generate LaTeX table for clean baseline comparison."""
    # Get clean baseline from any degradation type (all should be same)
    yolo_clean = None
    rfdetr_clean = None
    for deg in data:
        if yolo_clean is None and 'yolov8' in data[deg] and 0.0 in data[deg]['yolov8']:
            yolo_clean = data[deg]['yolov8'][0.0]
        if rfdetr_clean is None and 'rfdetr' in data[deg] and 0.0 in data[deg]['rfdetr']:
            rfdetr_clean = data[deg]['rfdetr'][0.0]

    if yolo_clean is None or rfdetr_clean is None:
        print("  WARNING: Missing clean baseline data")
        return

    metrics = [
        ('mAP@0.5', 'mAP_0_5'),
        ('mAP@0.5:0.95', 'mAP_0_5_0_95'),
        ('Precision', 'precision'),
        ('Recall', 'recall'),
        ('F1 Score', 'f1'),
    ]

    lines = [
        r'\begin{table}[htbp]',
        r'\centering',
        r'\caption{Clean baseline detection performance on 1{,}983 drone images.}',
        r'\label{tab:clean_baseline}',
        r'\begin{tabular}{lccc}',
        r'\hline',
        r'\textbf{Metric} & \textbf{YOLOv8n} & \textbf{RF-DETR} & \textbf{$\Delta$} \\',
        r'\hline',
    ]

    for label, key in metrics:
        yv = yolo_clean[key]
        rv = rfdetr_clean[key]
        delta = rv - yv
        sign = '+' if delta > 0 else ''
        winner = r'\textbf{' + f'{rv:.3f}' + '}' if rv > yv else f'{rv:.3f}'
        yolo_str = r'\textbf{' + f'{yv:.3f}' + '}' if yv > rv else f'{yv:.3f}'
        lines.append(f'{label} & {yolo_str} & {winner} & {sign}{delta:.3f} \\\\')

    lines.extend([
        r'\hline',
        r'\end{tabular}',
        r'\end{table}',
    ])

    out_path = table_dir / 'clean_baseline.tex'
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"  Saved: {out_path}")


# ============================================================================
# Table: Degradation-Averaged Performance (LaTeX)
# ============================================================================

def table_degradation_averaged(data: Dict, table_dir: Path):
    """Generate LaTeX table for average mAP across all severity levels."""
    lines = [
        r'\begin{table}[htbp]',
        r'\centering',
        r'\caption{Average mAP@0.5 across all severity levels (0.0--1.0) by degradation type.}',
        r'\label{tab:degradation_avg}',
        r'\begin{tabular}{lccc}',
        r'\hline',
        r'\textbf{Degradation} & \textbf{YOLOv8n} & \textbf{RF-DETR} & \textbf{$\Delta$} \\',
        r'\hline',
    ]

    for deg in DEG_ORDER:
        if deg not in data:
            continue
        yolo_vals = [data[deg]['yolov8'][s]['mAP_0_5']
                     for s in sorted(data[deg].get('yolov8', {}).keys())]
        rfdetr_vals = [data[deg]['rfdetr'][s]['mAP_0_5']
                       for s in sorted(data[deg].get('rfdetr', {}).keys())]

        ya = np.mean(yolo_vals)
        ra = np.mean(rfdetr_vals)
        delta = ra - ya
        sign = '+' if delta > 0 else ''

        winner_r = r'\textbf{' + f'{ra:.3f}' + '}' if ra > ya else f'{ra:.3f}'
        winner_y = r'\textbf{' + f'{ya:.3f}' + '}' if ya > ra else f'{ya:.3f}'

        lines.append(
            f'{DEG_LABELS[deg]} & {winner_y} & {winner_r} & {sign}{delta:.3f} \\\\'
        )

    lines.extend([
        r'\hline',
        r'\end{tabular}',
        r'\end{table}',
    ])

    out_path = table_dir / 'degradation_averaged.tex'
    with open(out_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"  Saved: {out_path}")


# ============================================================================
# Main
# ============================================================================

def main():
    print("SPIE DS112 Figure Generator")
    print(f"  Data: {DATA_PATH}")
    print(f"  Output: {OUT_DIR}")
    print(f"  Tables: {TABLE_DIR}")
    print()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    TABLE_DIR.mkdir(parents=True, exist_ok=True)

    data = load_data()
    print(f"Loaded data for {len(data)} degradation types\n")

    print("Generating figures...")
    fig_degradation_comparison(data, OUT_DIR)
    fig_advantage_heatmap(data, OUT_DIR)
    fig_severe_bar_chart(data, OUT_DIR)
    fig_retention_curve(data, OUT_DIR)
    fig_precision_recall(data, OUT_DIR)

    print("\nGenerating tables...")
    table_clean_baseline(data, TABLE_DIR)
    table_degradation_averaged(data, TABLE_DIR)

    print(f"\nDone. {len(list(OUT_DIR.glob('*.pdf')))} PDFs, "
          f"{len(list(OUT_DIR.glob('*.png')))} PNGs, "
          f"{len(list(TABLE_DIR.glob('*.tex')))} LaTeX tables generated.")


if __name__ == '__main__':
    main()
