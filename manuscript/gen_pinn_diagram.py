#!/usr/bin/env python3
"""
Generate PINN architecture diagram for AIRHOUND manuscript.
Outputs: manuscript/figures/pinn_architecture.pdf
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
import matplotlib.patches as mpatches

plt.rcParams.update({"font.family": "serif", "font.size": 9, "text.usetex": False})

fig, ax = plt.subplots(figsize=(10, 4.6))
ax.set_xlim(0, 10)
ax.set_ylim(-1.8, 4.8)
ax.axis("off")

# ── Colours ────────────────────────────────────────────────────────────────
C_INPUT  = "#4C72B0"
C_HIDDEN = "#55A868"
C_OUTPUT = "#C44E52"
C_PHYS   = "#DD8452"
C_FEED   = "#7B5EA7"
C_ARROW  = "#333333"
C_FACE   = "#F8F8F8"

# ── Layout ─────────────────────────────────────────────────────────────────
BOX_W = 1.30
BOX_H = 2.70
Y_MID = 1.85
Y_TOP = Y_MID + BOX_H / 2   # 3.20
Y_BOT = Y_MID - BOX_H / 2   # 0.50

X_IN  = 1.10
X_H1  = 3.05
X_H2  = 5.00
X_H3  = 6.95
X_OUT = 8.90
COLS  = [X_IN, X_H1, X_H2, X_H3, X_OUT]

# ── Block helper ───────────────────────────────────────────────────────────
STRIPE = 0.38

def block(ax, cx, cy, color, title, body):
    ax.add_patch(FancyBboxPatch(
        (cx - BOX_W/2, cy - BOX_H/2), BOX_W, BOX_H,
        boxstyle="round,pad=0.07", lw=1.3,
        edgecolor=color, facecolor=C_FACE, zorder=3))
    ax.add_patch(FancyBboxPatch(
        (cx - BOX_W/2, cy + BOX_H/2 - STRIPE), BOX_W, STRIPE,
        boxstyle="round,pad=0.0", lw=0,
        edgecolor=color, facecolor=color, alpha=0.88, zorder=4))
    ax.text(cx, cy + BOX_H/2 - STRIPE/2, title,
            ha="center", va="center", fontsize=8.5,
            fontweight="bold", color="white", zorder=5)
    ax.text(cx, cy - 0.10, body,
            ha="center", va="center", fontsize=7.8,
            color="#333333", zorder=5, linespacing=1.5)

# ── Draw blocks ────────────────────────────────────────────────────────────
block(ax, X_IN,  Y_MID, C_INPUT,  "Input",
      "$x,\\ y,\\ z$\n$v_x,\\ v_y,\\ v_z$\n$\\Delta t$\n$\\in\\mathbb{R}^7$")
for xi, lbl in [(X_H1,"Hidden 1"),(X_H2,"Hidden 2"),(X_H3,"Hidden 3")]:
    block(ax, xi, Y_MID, C_HIDDEN, lbl,
          "128 units\nReLU\n$\\in\\mathbb{R}^{128}$")
block(ax, X_OUT, Y_MID, C_OUTPUT, "Output",
      "$x',\\ y',\\ z'$\n$v_x',\\ v_y',\\ v_z'$\n$\\in\\mathbb{R}^6$")

# ── Forward arrows ─────────────────────────────────────────────────────────
for x0, x1 in zip(COLS, COLS[1:]):
    ax.annotate("", xy=(x1 - BOX_W/2, Y_MID), xytext=(x0 + BOX_W/2, Y_MID),
                arrowprops=dict(arrowstyle="-|>", color=C_ARROW, lw=1.3), zorder=2)

# ── Autoregressive feedback: U-shape ABOVE the boxes ───────────────────────
# Rise above boxes then arc left from output back to input
ARC_Y = Y_TOP + 0.52   # clear of all box tops

# Vertical stub up from output top to ARC_Y (no arrowhead)
ax.plot([X_OUT, X_OUT], [Y_TOP, ARC_Y], color=C_FEED, lw=1.4,
        linestyle=(0, (5, 3)), zorder=2)
# Horizontal run right→left, plain line (no arrowhead)
ax.plot([X_OUT, X_IN], [ARC_Y, ARC_Y], color=C_FEED, lw=1.4,
        linestyle=(0, (5, 3)), zorder=2)
# Vertical stub DOWN from ARC_Y into input top — arrowhead points downward into box
ax.annotate("", xy=(X_IN, Y_TOP), xytext=(X_IN, ARC_Y),
            arrowprops=dict(arrowstyle="-|>", color=C_FEED, lw=1.4,
                            linestyle=(0, (5, 3))), zorder=2)

ax.text(5.0, ARC_Y + 0.18,
        "autoregressive feedback  (dropout mode)",
        ha="center", va="bottom", fontsize=7.8,
        color=C_FEED, style="italic")

# ── Physics penalty box — centred in the figure, below the blocks ──────────
# Placed at x=5.0 so it stays fully inside xlim [0,10]
PX   = 5.0
PY   = -1.05
PW   = 7.80    # 5.0 ± 3.9  →  x from 1.1 to 8.9  ✓
PH   = 0.64

ax.add_patch(FancyBboxPatch(
    (PX - PW/2, PY - PH/2), PW, PH,
    boxstyle="round,pad=0.08", lw=1.2,
    edgecolor=C_PHYS, facecolor="#FFF4EC", alpha=0.96, zorder=3))

ax.text(PX, PY,
        r"Physics penalty:  $\mathcal{L}_\mathrm{physics} = "
        r"\frac{1}{N}\sum_i \max(0,\;"
        r"\|\mathbf{v}'_i\| - v_\mathrm{max})^{2}$"
        r"$\quad v_\mathrm{max} = 10\ \mathrm{m/s},\quad \lambda = 0.1$",
        ha="center", va="center", fontsize=8.0,
        color="#7A3B00", zorder=5)

# Arrow: straight down from output box bottom to physics box top
ax.annotate("", xy=(X_OUT, PY + PH/2),
            xytext=(X_OUT, Y_BOT),
            arrowprops=dict(arrowstyle="-|>", color=C_PHYS, lw=1.2,
                            linestyle=(0, (4, 2))), zorder=2)

# small label beside that arrow
MID_Y = (Y_BOT + PY + PH/2) / 2
ax.text(X_OUT + 0.14, MID_Y, "physics\npenalty",
        ha="left", va="center", fontsize=7.4,
        color=C_PHYS, style="italic")

# ── Footer ─────────────────────────────────────────────────────────────────
ax.text(5.0, -1.65,
        "Xavier uniform init  ·  "
        r"inputs & outputs normalised ($\mu=0,\;\sigma=1$)  ·  "
        "Adam  $\\mathrm{lr}=10^{-3}$  ·  200 epochs  ·  batch 256",
        ha="center", va="center", fontsize=7.1, color="#666666")

plt.tight_layout(pad=0.2)
plt.savefig("figures/pinn_architecture.pdf", bbox_inches="tight", dpi=200)
print("Saved figures/pinn_architecture.pdf")
