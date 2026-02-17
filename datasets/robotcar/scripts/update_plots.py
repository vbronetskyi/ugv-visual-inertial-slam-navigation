#!/usr/bin/env python3
"""Generate updated comparison plots for all completed methods"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

RESULTS_DIR = Path('/workspace/datasets/robotcar/results/robotcar_seasons_hloc')
PLOTS_DIR = RESULTS_DIR / 'plots'
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

# load all available results
METHODS = {
    'SP+SuperGlue': 'sp_sg_nv',
    'SP+LightGlue': 'sp_lg_nv',
    'ALIKED+LightGlue': 'aliked_lg_nv',
}

results = {}
for label, short in METHODS.items():
    path = RESULTS_DIR / short / 'eval_results.json'
    if path.exists():
        with open(path) as f:
            results[label] = json.load(f)

print(f"Loaded {len(results)} methods: {list(results.keys())}")

CONDITIONS = ['dawn', 'dusk', 'night', 'night-rain', 'overcast-summer',
              'overcast-winter', 'rain', 'snow', 'sun']

colors = {
    'SP+SuperGlue': '#2196F3',
    'SP+LightGlue': '#F44336',
    'ALIKED+LightGlue': '#4CAF50',
    'DISK+LightGlue': '#FF9800',
    'SP+SG+OpenIBL': '#9C27B0',
    'SIFT+NN-ratio': '#795548',
}

# plot 1: Overall comparison bar chart (3 thresholds)
fig, ax = plt.subplots(figsize=(12, 6))
thresholds = ['0.25m_2deg', '0.5m_5deg', '5.0m_10deg']
th_labels = ['0.25m, 2°', '0.5m, 5°', '5m, 10°']
x = np.arange(len(thresholds))
width = 0.22
offsets = np.arange(len(results)) - (len(results)-1)/2

for i, (method, data) in enumerate(results.items()):
    vals = [data['overall'][t] for t in thresholds]
    bars = ax.bar(x + offsets[i]*width, vals, width, label=method, color=colors.get(method, f'C{i}'))
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{v:.1f}', ha='center', va='bottom', fontsize=9, fontweight='bold')

ax.set_ylabel('% localized')
ax.set_title('Overall Accuracy - All Methods', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(th_labels, fontsize=12)
ax.legend(fontsize=10)
ax.set_ylim(0, 100)
ax.grid(axis='y', alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR / '1_overall_comparison.png', dpi=150)
plt.close()
print('Saved 1_overall_comparison.png')

# plot 2: Per-condition grouped bars (0.5m/5°)
fig, ax = plt.subplots(figsize=(16, 7))
x = np.arange(len(CONDITIONS))
width = 0.25
offsets = np.arange(len(results)) - (len(results)-1)/2

for i, (method, data) in enumerate(results.items()):
    vals = []
    for cond in CONDITIONS:
        if cond in data['by_condition']:
            vals.append(data['by_condition'][cond]['0.5m_5deg'])
        else:
            vals.append(0)
    bars = ax.bar(x + offsets[i]*width, vals, width, label=method, color=colors.get(method, f'C{i}'))
    for bar, v in zip(bars, vals):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                f'{v:.0f}', ha='center', va='bottom', fontsize=7)

ax.set_ylabel('% localized (0.5m, 5°)')
ax.set_title('Per-Condition Accuracy - All Methods (0.5m, 5°)', fontsize=14, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(CONDITIONS, rotation=30, ha='right', fontsize=10)
ax.legend(fontsize=10)
ax.set_ylim(0, 95)
ax.axhline(y=50, color='gray', linestyle='--', alpha=0.5)
ax.grid(axis='y', alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR / '2_per_condition.png', dpi=150)
plt.close()
# print(f"DEBUG pose_est={pose_est}")
print('Saved 2_per_condition.png')

# plot 3: Day vs Night
fig, axes = plt.subplots(1, 2, figsize=(14, 6))

for ax_idx, (split, title) in enumerate([('day', 'Day'), ('night', 'Night')]):
    ax = axes[ax_idx]
    x = np.arange(3)
    width = 0.22
    offsets = np.arange(len(results)) - (len(results)-1)/2

    for i, (method, data) in enumerate(results.items()):
        if data[split]:
            vals = [data[split][t] for t in thresholds]
        else:
            vals = [0, 0, 0]
        bars = ax.bar(x + offsets[i]*width, vals, width, label=method, color=colors.get(method, f'C{i}'))
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.5,
                    f'{v:.1f}', ha='center', va='bottom', fontsize=8)

    ax.set_ylabel('% localized')
    ax.set_title(f'{title} Conditions', fontsize=13, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(th_labels, fontsize=11)
    ax.set_ylim(0, 110)
    ax.legend(fontsize=9)
    ax.grid(axis='y', alpha=0.3)

plt.suptitle('Day vs Night - All Methods', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(PLOTS_DIR / '3_day_vs_night.png', dpi=150)
plt.close()
print("Saved 3_day_vs_night.png")

# plot 4: Heatmap - methods x conditions
fig, ax = plt.subplots(figsize=(14, 5))
method_names = list(results.keys())
data_matrix = []
for method in method_names:
    row = []
    for cond in CONDITIONS:
        if cond in results[method]['by_condition']:
            row.append(results[method]['by_condition'][cond]['0.5m_5deg'])
        else:
            row.append(0)
    data_matrix.append(row)

data_matrix = np.array(data_matrix)
im = ax.imshow(data_matrix, cmap='RdYlGn', aspect='auto', vmin=0, vmax=100)

ax.set_xticks(np.arange(len(CONDITIONS)))
ax.set_xticklabels(CONDITIONS, rotation=35, ha='right', fontsize=10)
ax.set_yticks(np.arange(len(method_names)))
ax.set_yticklabels(method_names, fontsize=11)

for i in range(len(method_names)):
    for j in range(len(CONDITIONS)):
        val = data_matrix[i, j]
        color = 'white' if val < 40 else 'black'
        ax.text(j, i, f'{val:.1f}', ha='center', va='center', fontsize=10, color=color, fontweight='bold')

plt.colorbar(im, ax=ax, label='% localized (0.5m, 5°)')
ax.set_title('Accuracy Heatmap - Methods × Conditions (0.5m, 5°)', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig(PLOTS_DIR / '4_heatmap.png', dpi=150)
plt.close()
print("Saved 4_heatmap.png")

# plot 5: Summary table as figure
fig, ax = plt.subplots(figsize=(12, 4))
ax.axis('off')

headers = ['Method', '0.25m/2°', '0.5m/5°', '5m/10°', 'Day', 'Night', 'Med. trans', 'Med. rot']
table_data = []
for method, data in results.items():
    ov = data['overall']
    d = data.get('day', {}) or {}
    n = data.get('night', {}) or {}
    table_data.append([
        method,
        f'{ov["0.25m_2deg"]:.1f}%',
        f'{ov["0.5m_5deg"]:.1f}%',
        f'{ov["5.0m_10deg"]:.1f}%',
        f'{d.get("0.5m_5deg", 0):.1f}%',
        f'{n.get("0.5m_5deg", 0):.1f}%',
        f'{ov["median_trans_m"]:.3f}m',
        f'{ov["median_rot_deg"]:.3f}°',
    ])

table = ax.table(cellText=table_data, colLabels=headers, loc='center', cellLoc='center')
table.auto_set_font_size(False)
table.set_fontsize(11)
table.scale(1.2, 1.8)

# color header
for j in range(len(headers)):
    table[0, j].set_facecolor('#333333')
    table[0, j].set_text_props(color='white', fontweight='bold')

# highlight best values
for i in range(1, len(table_data) + 1):
    for j in range(1, 6):
        table[i, j].set_facecolor('#E8F5E9')

ax.set_title('Summary - All Completed Methods', fontsize=14, fontweight='bold', pad=20)
plt.tight_layout()
plt.savefig(PLOTS_DIR / '5_summary_table.png', dpi=150, bbox_inches='tight')
plt.close()
print("Saved 5_summary_table.png")

# plot 6: Median error comparison
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

method_names = list(results.keys())
x = np.arange(len(method_names))

# translation
for i, method in enumerate(method_names):
    ov = results[method]['overall']
    d = results[method].get('day', {}) or {}
    n = results[method].get('night', {}) or {}

    ax1.bar(i - 0.2, ov['median_trans_m'], 0.2, color=colors.get(method, f'C{i}'), label='Overall' if i==0 else '')
    if d:
        ax1.bar(i, d.get('median_trans_m', 0), 0.2, color=colors.get(method, f'C{i}'), alpha=0.6)
    if n:
        ax1.bar(i + 0.2, min(n.get('median_trans_m', 0), 80), 0.2, color=colors.get(method, f'C{i}'), alpha=0.3)

ax1.set_ylabel('Median translation error (m)')
ax1.set_title('Median Translation Error', fontweight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels([m.replace('+', '+\n') for m in method_names], fontsize=9)
ax1.grid(axis='y', alpha=0.3)

# rotation
for i, method in enumerate(method_names):
    ov = results[method]['overall']
    d = results[method].get('day', {}) or {}
    n = results[method].get('night', {}) or {}

    ax2.bar(i - 0.2, ov['median_rot_deg'], 0.2, color=colors.get(method, f'C{i}'))
    if d:
        ax2.bar(i, d.get('median_rot_deg', 0), 0.2, color=colors.get(method, f'C{i}'), alpha=0.6)
    if n:
        ax2.bar(i + 0.2, min(n.get('median_rot_deg', 0), 40), 0.2, color=colors.get(method, f'C{i}'), alpha=0.3)

ax2.set_ylabel('Median rotation error (°)')
ax2.set_title('Median Rotation Error', fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels([m.replace('+', '+\n') for m in method_names], fontsize=9)
ax2.grid(axis='y', alpha=0.3)

# legend
from matplotlib.patches import Patch
legend_elements = [Patch(facecolor='gray', alpha=1.0, label='Overall'),
                   Patch(facecolor='gray', alpha=0.6, label='Day'),
                   Patch(facecolor='gray', alpha=0.3, label='Night')]
ax1.legend(handles=legend_elements, fontsize=9)

plt.suptitle('Median Pose Error - All Methods', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig(PLOTS_DIR / '6_median_errors.png', dpi=150)
plt.close()
print("Saved 6_median_errors.png")

# print(f">>> image {i}/{len(images)}")
print("\nAll plots saved to:", PLOTS_DIR)
