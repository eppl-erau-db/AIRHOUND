#!/usr/bin/env python3
"""Generate poster-quality latency comparison figure for SPIE poster.

Uses summary statistics from HITL comparison CSV. Only includes the three
camera configurations shown in the poster table (excludes short V4L2 test
and sim-only runs).
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# --- Data from comparison.csv (camera configurations only) ---
configs = {
    'Cam 640\n(bridge)': {
        'mean': 234.5, 'p50': 199.0, 'p95': 598.1, 'p99': 726.0, 'max': 840.4,
        'color': '#4C72B0',
    },
    'Cam 720\n(bridge)': {
        'mean': 233.7, 'p50': 206.2, 'p95': 558.2, 'p99': 629.8, 'max': 747.9,
        'color': '#C44E52',
    },
    'Cam Direct\n(V4L2)': {
        'mean': 58.8, 'p50': 59.0, 'p95': 61.5, 'p99': 63.2, 'max': 161.0,
        'color': '#55A868',
    },
}

labels = list(configs.keys())
means = [configs[k]['mean'] for k in labels]
p50s = [configs[k]['p50'] for k in labels]
p95s = [configs[k]['p95'] for k in labels]

x = np.arange(len(labels))
width = 0.35

fig, ax = plt.subplots(figsize=(8, 4.5))

bars_mean = ax.bar(x - width/2, means, width, label='Mean', color=[configs[k]['color'] for k in labels],
                   alpha=0.85, edgecolor='white', linewidth=1.2)
bars_p95 = ax.bar(x + width/2, p95s, width, label='P95', color=[configs[k]['color'] for k in labels],
                  alpha=0.45, edgecolor='white', linewidth=1.2, hatch='///')

# Value annotations
for bar in bars_mean:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width() / 2., height + 8,
            f'{height:.0f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
for bar in bars_p95:
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width() / 2., height + 8,
            f'{height:.0f}', ha='center', va='bottom', fontsize=11)

ax.set_ylabel('Latency (ms)', fontsize=13)
ax.set_title('Inference Latency by Capture Method', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=12)
ax.legend(fontsize=11, loc='upper left')
ax.set_ylim(0, 700)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# Add 4x improvement annotation
ax.annotate('4× reduction', xy=(2, 61), xytext=(1.4, 350),
            fontsize=12, fontweight='bold', color='#55A868',
            arrowprops=dict(arrowstyle='->', color='#55A868', lw=2),
            ha='center')

fig.tight_layout()
fig.savefig('figures/latency_poster.pdf', bbox_inches='tight')
fig.savefig('figures/latency_poster.png', dpi=200, bbox_inches='tight')
plt.close(fig)
print("Saved figures/latency_poster.pdf and .png")
