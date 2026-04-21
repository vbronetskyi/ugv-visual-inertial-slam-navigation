#!/usr/bin/env python3
"""Three-panel comparison of the top-3 ranked runs.

Each panel shows:
  - green terrain base
  - dashed black teach reference
  - orange cone barriers + green tent at true positions
  - colored run trajectory
  - start / turnaround / end markers
  - metrics overlay
"""
import csv
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

ROOT = Path('/workspace/simulation/isaac')
TEACH_CSV = ROOT / 'experiments/48_vio_roundtrip__/logs/exp48_vio_traj.csv'

CONES = [(-75,-24),(-75,-25),(-75,-26),(-18,-24),(-18,-25),
         (5,-17),(5,-18),(5,-19),(5,-20)]
TENT_C, TENT_HX, TENT_HY = (-45.0,-38.0), 1.1, 1.0

RUNS = [
    dict(
        label='Exp 59 - rank 1',
        csv=ROOT / 'experiments/59_wp_lookahead_detour/results/repeat_run/traj_gt.csv',
        x='x', y='y', color='#d62728',
        metrics=['TRACK p50 1.60 m  p95 7.38 m',
                 'ARRIVAL p50 3.68 m  p95 5.40 m',
                 'tent −22 cm   cone −75 cm',
                 'loop 5.95 m   path 1.41×   time 10 min'],
    ),
    dict(
        label='Exp 60 - rank 2',
        csv=ROOT / 'experiments/60_final_approach_10cm/results/repeat_run/traj_gt.csv',
        x='x', y='y', color='#1f77b4',
        metrics=['TRACK p50 2.59 m  p95 5.42 m',
                 'ARRIVAL p50 3.46 m  p95 6.96 m',
                 'tent +151 cm   cone −58 cm',
                 'loop 13.46 m   path 1.20×   time 5 min'],
    ),
    dict(
        label='Exp 56 - rank 3',
        csv=ROOT / 'experiments/56_projection_cap_1m/results/repeat_run/trajectory.csv',
        x='gt_x', y='gt_y', color='#2ca02c',
        metrics=['TRACK p50 0.69 m  p95 3.99 m',
                 'ARRIVAL p50 2.85 m  p95 3.70 m',
                 'tent −10 cm   cone −35 cm',
                 'loop 9.08 m   path 3.55×   time 44 min'],
    ),
]

# Load teach path
teach_x, teach_y = [], []
with open(TEACH_CSV) as f:
    for r in csv.DictReader(f):
        teach_x.append(float(r['gt_x']))
        teach_y.append(float(r['gt_y']))
teach_x, teach_y = np.array(teach_x), np.array(teach_y)
TURN_I = int(np.argmax(teach_x))
START = (teach_x[0], teach_y[0])
TURN = (teach_x[TURN_I], teach_y[TURN_I])
END = (teach_x[-1], teach_y[-1])

XLIM = (-110, 85)
YLIM = (-50, 5)

fig, axes = plt.subplots(1, 3, figsize=(22, 7), sharey=True)
for ax, run in zip(axes, RUNS):
    ax.set_facecolor('#6b8e4e')
    # teach reference
    ax.plot(teach_x, teach_y, 'k--', linewidth=1.2, alpha=0.7, label='teach GT')
    # obstacles
    for cx, cy in CONES:
        ax.add_patch(Circle((cx, cy), 0.3, color='#ff7f00', alpha=0.9, zorder=5))
    ax.add_patch(Rectangle((TENT_C[0] - TENT_HX, TENT_C[1] - TENT_HY),
                           2 * TENT_HX, 2 * TENT_HY,
                           color='#005500', alpha=0.9, zorder=5))
    # actual run
    xs, ys = [], []
    with open(run['csv']) as f:
        for r in csv.DictReader(f):
            try:
                xs.append(float(r[run['x']])); ys.append(float(r[run['y']]))
            except: pass
    ax.plot(xs, ys, color=run['color'], linewidth=1.8, alpha=0.9, label='GT trajectory')
    # markers
    ax.plot(*START, marker='o', color='white', markersize=14, markeredgecolor='black',
            markeredgewidth=2, zorder=10)
    ax.plot(*TURN,  marker='s', color='yellow', markersize=14, markeredgecolor='black',
            markeredgewidth=2, zorder=10)
    ax.plot(*END,   marker='X', color='red',    markersize=14, markeredgecolor='black',
            markeredgewidth=2, zorder=10)
    # physical end position
    if xs:
        ax.plot(xs[-1], ys[-1], marker='*', color='black', markersize=16,
                markeredgecolor=run['color'], markeredgewidth=2, zorder=11,
                label='robot stopped here')
    # labels on markers (first axis only to avoid clutter)
    if ax is axes[0]:
        ax.annotate('START',     START,     xytext=(6, 6),  textcoords='offset points', fontsize=9, weight='bold')
        ax.annotate('TURNAROUND', TURN,     xytext=(6, 6),  textcoords='offset points', fontsize=9, weight='bold')
        ax.annotate('END (teach)', END,     xytext=(-75, -16), textcoords='offset points', fontsize=9, weight='bold')

    ax.set_xlim(XLIM); ax.set_ylim(YLIM)
    ax.set_xlabel('x  [m]')
    if ax is axes[0]: ax.set_ylabel('y  [m]')
    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    ax.set_title(run['label'], color=run['color'], fontsize=13, weight='bold')
    # metrics overlay
    ax.text(0.02, 0.02, '\n'.join(run['metrics']),
            transform=ax.transAxes, fontsize=9, family='monospace',
            bbox=dict(facecolor='white', alpha=0.85, edgecolor='gray'),
            verticalalignment='bottom')
    ax.legend(loc='upper right', fontsize=8)

plt.suptitle('Top-3 ranked runs - GT trajectory vs teach reference',
             fontsize=14, weight='bold')
plt.tight_layout()
OUT = ROOT / 'analysis/experiment_ranking/top3_trajectories.png'
plt.savefig(OUT, dpi=110, bbox_inches='tight')
print(f"saved {OUT}")
