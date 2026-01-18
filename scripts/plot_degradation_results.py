#!/usr/bin/env python3
"""
Generate paper-ready plots for SPIE Defense + Security 2026 submission.

Visualizes RF-DETR vs YOLOv8 performance under degradation conditions.

Usage:
    python3 plot_degradation_results.py /path/to/evaluation_results.json

Output:
    - degradation_comparison.pdf (main figure)
    - degradation_heatmap.pdf (supplementary)
    - per_degradation_plots/*.pdf

Author: AIRHOUND Perception Team
Date: January 2026
"""

import json
import sys
from pathlib import Path
from typing import List, Dict, Tuple
import numpy as np

# Check for matplotlib
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for server
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("WARNING: matplotlib not found. Install with: pip3 install matplotlib")


# SPIE paper style settings
PAPER_STYLE = {
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.figsize': (6.5, 4.5),  # Column width for SPIE
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.5,
    'lines.markersize': 6,
}

# Color scheme (accessible colors)
COLORS = {
    'yolov8': '#1f77b4',   # Blue
    'rfdetr': '#d62728',   # Red
    'clean': '#2ca02c',    # Green
}

MARKERS = {
    'yolov8': 'o',
    'rfdetr': 's',
}

DEGRADATION_LABELS = {
    'clean': 'Clean (Baseline)',
    'motion_blur': 'Motion Blur',
    'gaussian_noise': 'Gaussian Noise',
    'occlusion': 'Partial Occlusion',
    'low_light': 'Low Light',
}


def load_results(json_path: str) -> List[Dict]:
    """Load and deduplicate evaluation results."""
    with open(json_path) as f:
        data = json.load(f)
    
    # Deduplicate based on (model, degradation, severity)
    seen = set()
    unique = []
    for r in data:
        key = (r['model'], r['degradation_type'], r['severity'])
        if key not in seen:
            seen.add(key)
            unique.append(r)
    
    return unique


def organize_by_degradation(results: List[Dict]) -> Dict[str, Dict[str, Dict[float, Dict]]]:
    """
    Organize results by degradation type, then model, then severity.
    
    Returns:
        {degradation_type: {model: {severity: result}}}
    """
    organized = {}
    for r in results:
        deg = r['degradation_type']
        model = r['model']
        sev = r['severity']
        
        if deg not in organized:
            organized[deg] = {}
        if model not in organized[deg]:
            organized[deg][model] = {}
        organized[deg][model][sev] = r
    
    return organized


def plot_degradation_comparison(results: List[Dict], output_dir: Path):
    """
    Create main comparison figure for paper.
    
    Shows mAP vs severity for each degradation type, comparing models.
    """
    if not HAS_MPL:
        print("Skipping plot_degradation_comparison: matplotlib not available")
        return
    
    plt.rcParams.update(PAPER_STYLE)
    
    organized = organize_by_degradation(results)
    degradations = [d for d in ['motion_blur', 'gaussian_noise', 'occlusion', 'low_light'] 
                   if d in organized]
    
    n_deg = len(degradations)
    if n_deg == 0:
        print("No degradation results to plot")
        return
    
    # Create subplots
    fig, axes = plt.subplots(1, n_deg, figsize=(3.2 * n_deg, 3.5), sharey=True)
    if n_deg == 1:
        axes = [axes]
    
    for ax, deg in zip(axes, degradations):
        for model in ['yolov8', 'rfdetr']:
            if model not in organized[deg]:
                continue
            
            data = organized[deg][model]
            severities = sorted(data.keys())
            mAPs = [data[s]['mAP_0_5'] for s in severities]
            
            label = 'YOLOv8' if model == 'yolov8' else 'RF-DETR'
            ax.plot(severities, mAPs, 
                   marker=MARKERS[model],
                   color=COLORS[model],
                   label=label,
                   linewidth=2,
                   markersize=8)
        
        ax.set_xlabel('Severity Level')
        ax.set_title(DEGRADATION_LABELS.get(deg, deg))
        ax.set_xlim(-0.05, 1.05)
        ax.set_ylim(0, 1.0)
        ax.xaxis.set_major_locator(MultipleLocator(0.25))
        ax.yaxis.set_major_locator(MultipleLocator(0.2))
    
    axes[0].set_ylabel('mAP@0.5')
    axes[-1].legend(loc='lower left', framealpha=0.9)
    
    fig.suptitle('Detection Accuracy Under Image Degradation', fontsize=12, y=1.02)
    plt.tight_layout()
    
    output_path = output_dir / 'degradation_comparison.pdf'
    plt.savefig(output_path, format='pdf')
    plt.savefig(output_dir / 'degradation_comparison.png', format='png')
    print(f"Saved: {output_path}")
    plt.close()


def plot_heatmap(results: List[Dict], output_dir: Path):
    """
    Create heatmap showing relative performance difference.
    
    Shows (RF-DETR mAP - YOLOv8 mAP) for each condition.
    Positive = RF-DETR better, Negative = YOLOv8 better.
    """
    if not HAS_MPL:
        print("Skipping plot_heatmap: matplotlib not available")
        return
    
    plt.rcParams.update(PAPER_STYLE)
    
    organized = organize_by_degradation(results)
    degradations = [d for d in ['clean', 'motion_blur', 'gaussian_noise', 'occlusion', 'low_light'] 
                   if d in organized]
    
    # Get all severity levels
    all_severities = set()
    for deg in degradations:
        for model in organized[deg]:
            all_severities.update(organized[deg][model].keys())
    severities = sorted(all_severities)
    
    # Build difference matrix
    diff_matrix = np.full((len(degradations), len(severities)), np.nan)
    
    for i, deg in enumerate(degradations):
        if 'yolov8' in organized[deg] and 'rfdetr' in organized[deg]:
            for j, sev in enumerate(severities):
                if sev in organized[deg]['yolov8'] and sev in organized[deg]['rfdetr']:
                    yolo_map = organized[deg]['yolov8'][sev]['mAP_0_5']
                    rfdetr_map = organized[deg]['rfdetr'][sev]['mAP_0_5']
                    diff_matrix[i, j] = rfdetr_map - yolo_map
    
    fig, ax = plt.subplots(figsize=(6, 4))
    
    im = ax.imshow(diff_matrix, cmap='RdBu', aspect='auto', vmin=-0.3, vmax=0.3)
    
    ax.set_xticks(range(len(severities)))
    ax.set_xticklabels([f'{s:.2f}' for s in severities])
    ax.set_yticks(range(len(degradations)))
    ax.set_yticklabels([DEGRADATION_LABELS.get(d, d) for d in degradations])
    
    ax.set_xlabel('Severity Level')
    ax.set_ylabel('Degradation Type')
    ax.set_title('RF-DETR vs YOLOv8: mAP Difference\n(Positive = RF-DETR better)')
    
    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('mAP Difference (RF-DETR - YOLOv8)')
    
    # Add text annotations
    for i in range(len(degradations)):
        for j in range(len(severities)):
            val = diff_matrix[i, j]
            if not np.isnan(val):
                color = 'white' if abs(val) > 0.15 else 'black'
                ax.text(j, i, f'{val:+.2f}', ha='center', va='center', 
                       color=color, fontsize=8)
    
    plt.tight_layout()
    
    output_path = output_dir / 'degradation_heatmap.pdf'
    plt.savefig(output_path, format='pdf')
    plt.savefig(output_dir / 'degradation_heatmap.png', format='png')
    print(f"Saved: {output_path}")
    plt.close()


def plot_recall_vs_precision(results: List[Dict], output_dir: Path):
    """
    Create precision-recall scatter for each degradation condition.
    """
    if not HAS_MPL:
        print("Skipping plot_recall_vs_precision: matplotlib not available")
        return
    
    plt.rcParams.update(PAPER_STYLE)
    
    fig, ax = plt.subplots(figsize=(6, 5))
    
    for r in results:
        model = r['model']
        deg = r['degradation_type']
        sev = r['severity']
        
        # Only plot non-clean at higher severity
        if deg == 'clean' or sev < 0.5:
            continue
        
        marker = MARKERS.get(model, 'o')
        color = COLORS.get(model, 'gray')
        alpha = 0.3 + 0.7 * sev  # Higher severity = more visible
        
        ax.scatter(r['recall'], r['precision'], 
                  marker=marker, color=color, alpha=alpha, s=50)
    
    # Add legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['yolov8'], 
               markersize=10, label='YOLOv8'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS['rfdetr'], 
               markersize=10, label='RF-DETR'),
    ]
    ax.legend(handles=legend_elements, loc='lower left')
    
    ax.set_xlabel('Recall')
    ax.set_ylabel('Precision')
    ax.set_title('Precision vs Recall Under Degradation (severity ≥ 0.5)')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.plot([0, 1], [0, 1], 'k--', alpha=0.3, label='P=R')
    
    plt.tight_layout()
    
    output_path = output_dir / 'precision_recall_degraded.pdf'
    plt.savefig(output_path, format='pdf')
    print(f"Saved: {output_path}")
    plt.close()


def generate_summary_table(results: List[Dict], output_dir: Path):
    """Generate markdown summary table."""
    organized = organize_by_degradation(results)
    
    lines = [
        "# Degradation Evaluation Summary",
        "",
        "## Results by Degradation Type",
        "",
        "| Degradation | Severity | YOLOv8 mAP | RF-DETR mAP | Δ mAP |",
        "|-------------|----------|------------|-------------|-------|",
    ]
    
    for deg in ['clean', 'motion_blur', 'gaussian_noise', 'occlusion', 'low_light']:
        if deg not in organized:
            continue
        
        for sev in sorted(set(s for m in organized[deg].values() for s in m.keys())):
            yolo_map = organized[deg].get('yolov8', {}).get(sev, {}).get('mAP_0_5', np.nan)
            rfdetr_map = organized[deg].get('rfdetr', {}).get(sev, {}).get('mAP_0_5', np.nan)
            
            if np.isnan(yolo_map) and np.isnan(rfdetr_map):
                continue
            
            delta = rfdetr_map - yolo_map if not (np.isnan(yolo_map) or np.isnan(rfdetr_map)) else np.nan
            
            yolo_str = f"{yolo_map:.4f}" if not np.isnan(yolo_map) else "N/A"
            rfdetr_str = f"{rfdetr_map:.4f}" if not np.isnan(rfdetr_map) else "N/A"
            delta_str = f"{delta:+.4f}" if not np.isnan(delta) else "N/A"
            
            lines.append(f"| {DEGRADATION_LABELS.get(deg, deg)} | {sev:.2f} | {yolo_str} | {rfdetr_str} | {delta_str} |")
    
    lines.extend([
        "",
        "## Key Findings",
        "",
        "- Positive Δ mAP = RF-DETR outperforms YOLOv8",
        "- Negative Δ mAP = YOLOv8 outperforms RF-DETR",
        "",
        f"Generated: {Path(sys.argv[1]).parent.name if len(sys.argv) > 1 else 'N/A'}",
    ])
    
    output_path = output_dir / 'summary_table.md'
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"Saved: {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_degradation_results.py <evaluation_results.json>")
        print("\nExample:")
        print("  python3 plot_degradation_results.py ~/AIRHOUND/benchmark_results/degradation_eval_*/evaluation_results.json")
        sys.exit(1)
    
    json_path = Path(sys.argv[1])
    if not json_path.exists():
        print(f"Error: {json_path} not found")
        sys.exit(1)
    
    output_dir = json_path.parent
    print(f"Loading results from: {json_path}")
    print(f"Output directory: {output_dir}")
    
    results = load_results(json_path)
    print(f"Loaded {len(results)} unique evaluation results")
    
    if not results:
        print("No results to plot")
        sys.exit(1)
    
    # Generate all outputs
    plot_degradation_comparison(results, output_dir)
    plot_heatmap(results, output_dir)
    plot_recall_vs_precision(results, output_dir)
    generate_summary_table(results, output_dir)
    
    print("\nDone! Check the output directory for generated files.")


if __name__ == '__main__':
    main()
