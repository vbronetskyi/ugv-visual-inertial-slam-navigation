#!/usr/bin/env python3
"""Ranking visualisation - two panels:
  (a) horizontal bar chart of composite score (outdoor UGV weighting),
      sorted worst -> best, broken down by sub-score contribution
  (b) heatmap of raw sub-scores per experiment, rows sorted by composite
"""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap

HERE = Path(__file__).parent

WEIGHTS_OUTDOOR = dict(safety=0.30, precision=0.20, efficiency=0.15,
                       completion=0.15, speed=0.10, smoothness=0.10)
SUBS = list(WEIGHTS_OUTDOOR.keys())

rows = []
with open(HERE / 'composite_scores.csv') as f:
    rows = list(csv.DictReader(f))
rows.sort(key=lambda r: float(r['composite_outdoor']))   # worst -> best
names = [r['name'] for r in rows]
composites = [float(r['composite_outdoor']) for r in rows]
subs = {s: [float(r[s]) for r in rows] for s in SUBS}
research = [float(r['composite_research']) for r in rows]
production = [float(r['composite_production']) for r in rows]

# Contribution of each sub-score to the composite
contribs = {s: np.array(subs[s]) * WEIGHTS_OUTDOOR[s] for s in SUBS}

fig = plt.figure(figsize=(18, 9))
gs = fig.add_gridspec(1, 2, width_ratios=[1.4, 1.0], wspace=0.25)

# -------- (a) stacked horizontal bars --------
ax1 = fig.add_subplot(gs[0, 0])
y = np.arange(len(names))
left = np.zeros(len(names))
sub_colors = {
    'safety':     '#d62728',  # red   - safety is first priority
    'precision':  '#1f77b4',  # blue
    'efficiency': '#2ca02c',  # green
    'completion': '#ff7f0e',  # orange
    'speed':      '#9467bd',  # purple
    'smoothness': '#8c564b',  # brown
}
for s in SUBS:
    ax1.barh(y, contribs[s], left=left, color=sub_colors[s],
             label=f'{s} (w={WEIGHTS_OUTDOOR[s]:.2f})',
             edgecolor='white', linewidth=0.5)
    left += contribs[s]
# annotations: composite value + also show research/production in grey ticks
for i, (comp, res, prd) in enumerate(zip(composites, research, production)):
    ax1.text(comp + 0.01, i, f"  {comp:.3f}",
             va='center', fontsize=10, weight='bold')
    ax1.text(-0.02, i, f"R {res:.2f} | P {prd:.2f}",
             ha='right', va='center', fontsize=8, color='gray')
ax1.set_yticks(y)
ax1.set_yticklabels(names, fontsize=10)
ax1.set_xlabel('Composite score  (outdoor UGV weighting)')
ax1.set_xlim(0, 1.0)
ax1.set_title('Ranking - each bar = composite, colour = sub-score contribution\n'
              '(grey: R = research weighting · P = production weighting)',
              fontsize=11)
ax1.axvline(0.5, color='gray', linestyle=':', alpha=0.5)
ax1.legend(loc='lower right', fontsize=8, framealpha=0.95)
ax1.grid(axis='x', alpha=0.3)

# -------- (b) heatmap of raw sub-scores --------
ax2 = fig.add_subplot(gs[0, 1])
mat = np.array([[float(r[s]) for s in SUBS] for r in rows])
# Keep same order (worst bottom, best top) - aligns with bar chart
cmap = LinearSegmentedColormap.from_list('rg',
       ['#b80000', '#ffcc00', '#009a00'], N=256)
im = ax2.imshow(mat, cmap=cmap, vmin=0, vmax=1, aspect='auto')
ax2.set_yticks(y); ax2.set_yticklabels(names, fontsize=10)
ax2.set_xticks(range(len(SUBS)))
ax2.set_xticklabels(SUBS, rotation=35, ha='right', fontsize=9)
ax2.set_title('Sub-score matrix (0 = worst, 1 = best)', fontsize=11)
# value annotations
for i in range(mat.shape[0]):
    for j in range(mat.shape[1]):
        v = mat[i, j]
        c = 'white' if v < 0.35 or v > 0.75 else 'black'
        ax2.text(j, i, f'{v:.2f}', ha='center', va='center',
                 color=c, fontsize=8)
cb = plt.colorbar(im, ax=ax2, fraction=0.04, pad=0.03)
cb.set_label('Sub-score')

plt.suptitle('Composite ranking of visual T&R experiments (52..62)',
             fontsize=13, weight='bold', y=1.00)
plt.tight_layout()
out = HERE / 'ranking_plots.png'
plt.savefig(out, dpi=110, bbox_inches='tight')
print(f"saved {out}")
