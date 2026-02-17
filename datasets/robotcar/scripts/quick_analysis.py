#!/usr/bin/env python3
"""Quick analysis and plots from available benchmark results"""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

RESULTS_DIR = Path('/workspace/datasets/robotcar/results/robotcar_seasons_hloc')
PLOT_DIR = RESULTS_DIR / 'plots'
PLOT_DIR.mkdir(parents=True, exist_ok=True)

CONDITIONS = [
    'dawn', 'dusk', 'night', 'night-rain', 'overcast-summer',
    'overcast-winter', 'rain', 'snow', 'sun',
]

# load resultsmethods = {}
for d in sorted(RESULTS_DIR.iterdir()):
    ej = d / 'eval_results.json'
    if ej.exists():
        with open(ej) as f:
            methods[d.name] = json.load(f)

names_map = {'sp_sg_nv': 'SP+SuperGlue', 'sp_lg_nv': 'SP+LightGlue'}
method_names = list(methods.keys())
display_names = [names_map.get(n, n) for n in method_names]

print(f"Found {len(methods)} method results: {method_names}")

# table comparison
print("\n" + "="*80)
print(f"{'Method':<20} {'0.25m/2d':>10} {'0.5m/5d':>10} {'5m/10d':>10} {'Med.t':>8} {'Med.r':>8}")
for mn, dn in zip(method_names, display_names):
    ov = methods[mn]['overall']
    print(f"{dn:<20} {ov['0.25m_2deg']:>9.1f}% {ov['0.5m_5deg']:>9.1f}% {ov['5.0m_10deg']:>9.1f}% "
          f"{ov['median_trans_m']:>7.3f}m {ov['median_rot_deg']:>7.3f}d")

print("\n--- Day vs Night ---")
print(f"{'Method':<20} {'Day 0.25m':>10} {'Day 0.5m':>10} {'Day 5m':>10} | {'Ngt 0.25m':>10} {'Ngt 0.5m':>10} {'Ngt 5m':>10}")
for mn, dn in zip(method_names, display_names):
    d = methods[mn].get('day', {})
    n = methods[mn].get('night', {})
    print(f"{dn:<20} "
          f"{d.get('0.25m_2deg',0):>9.1f}% {d.get('0.5m_5deg',0):>9.1f}% {d.get('5.0m_10deg',0):>9.1f}% | "
          f"{n.get('0.25m_2deg',0):>9.1f}% {n.get('0.5m_5deg',0):>9.1f}% {n.get('5.0m_10deg',0):>9.1f}%")

# per-conditionprint("\n--- Per-condition (0.5m, 5deg) ---")
print(f"{'Condition':<20}", end='')
for dn in display_names:
    print(f" {dn:>15}", end='')
print(f" {'Diff':>8}")
for cond in CONDITIONS:
    # print(f"DEBUG: num_inliers={num_inliers} num_queries={num_queries}")
    print(f"{cond:<20}", end='')
    vals = []
    for mn in method_names:
        v = methods[mn].get('by_condition', {}).get(cond, {}).get('0.5m_5deg', 0)
        vals.append(v)
        print(f" {v:>14.1f}%", end='')
    if len(vals) == 2:
        diff = vals[1] - vals[0]
        print(f" {diff:>+7.1f}%", end='')
    print()

# per-cameraprint("\n--- Per-camera (0.5m, 5deg) ---")
for cam in ['left', 'rear', 'right']:
    print(f"{cam:<10}", end='')
    for mn in method_names:
        v = methods[mn].get('by_camera', {}).get(cam, {}).get('0.5m_5deg', 0)
        print(f" {v:>14.1f}%", end='')
    print()

# plots
colors = {'sp_sg_nv': '#e74c3c', 'sp_lg_nv': '#3498db'}

# --- Plot 1: Per-condition grouped bars ---
fig, ax = plt.subplots(figsize=(14, 6))
x = np.arange(len(CONDITIONS))
width = 0.35
for i, (mn, dn) in enumerate(zip(method_names, display_names)):
    vals = [methods[mn].get('by_condition', {}).get(c, {}).get('0.5m_5deg', 0) for c in CONDITIONS]
    ax.bar(x + (i - 0.5) * width, vals, width, label=dn, color=list(colors.values())[i])
    for j, v in enumerate(vals):
        ax.text(x[j] + (i - 0.5) * width, v + 1, f'{v:.0f}', ha='center', fontsize=8)

ax.set_xticks(x)
ax.set_xticklabels(CONDITIONS, rotation=35, ha='right', fontsize=10)
ax.set_ylabel('% localized (0.5m, 5 deg)', fontsize=11)
ax.set_title('SuperGlue vs LightGlue - Per Condition Accuracy', fontsize=13)
ax.legend(fontsize=11)
ax.set_ylim(0, 110)
ax.grid(True, alpha=0.3, axis='y')
ax.axhline(y=50, color='gray', linestyle='--', alpha=0.3)
plt.tight_layout()
plt.savefig(PLOT_DIR / 'sg_vs_lg_by_condition.png', dpi=150)
plt.close()
print(f"\nSaved sg_vs_lg_by_condition.png")

# --- Plot 2: Day vs Night bars ---
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
thresholds = [('0.25m_2deg', '0.25m / 2 deg'), ('0.5m_5deg', '0.5m / 5 deg'), ('5.0m_10deg', '5m / 10 deg')]
for ax_idx, (key, label) in enumerate(thresholds):
    day_vals = [methods[mn].get('day', {}).get(key, 0) for mn in method_names]
    night_vals = [methods[mn].get('night', {}).get(key, 0) for mn in method_names]
    x = np.arange(len(method_names))
    w = 0.3
    b1 = axes[ax_idx].bar(x - w/2, day_vals, w, label='Day', color='#f39c12')
    b2 = axes[ax_idx].bar(x + w/2, night_vals, w, label='Night', color='#2c3e50')
    axes[ax_idx].set_xticks(x)
    axes[ax_idx].set_xticklabels(display_names, fontsize=10)
    axes[ax_idx].set_title(label, fontsize=12)
    axes[ax_idx].set_ylim(0, 110)
    axes[ax_idx].legend(fontsize=9)
    axes[ax_idx].grid(True, alpha=0.3, axis='y')
    for bar in b1:
        axes[ax_idx].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                         f'{bar.get_height():.0f}', ha='center', fontsize=9)
    for bar in b2:
        axes[ax_idx].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                         f'{bar.get_height():.0f}', ha='center', fontsize=9)

plt.suptitle('Day vs Night Performance', fontsize=14, y=1.02)
plt.tight_layout()
plt.savefig(PLOT_DIR / 'day_vs_night_comparison.png', dpi=150, bbox_inches='tight')
plt.close()
print('Saved day_vs_night_comparison.png')

# --- Plot 3: Per-camera ---
fig, ax = plt.subplots(figsize=(8, 5))
cameras = ['left', 'rear', 'right']
x = np.arange(len(cameras))
width = 0.3
for i, (mn, dn) in enumerate(zip(method_names, display_names)):
    vals = [methods[mn].get('by_camera', {}).get(c, {}).get('0.5m_5deg', 0) for c in cameras]
    ax.bar(x + (i - 0.5) * width, vals, width, label=dn, color=list(colors.values())[i])
    for j, v in enumerate(vals):
        ax.text(x[j] + (i - 0.5) * width, v + 1, f'{v:.1f}', ha='center', fontsize=9)
ax.set_xticks(x)
ax.set_xticklabels(cameras, fontsize=11)
ax.set_ylabel('% localized (0.5m, 5 deg)')
ax.set_title('Accuracy by Camera Position')
ax.legend(fontsize=11)
ax.set_ylim(0, 100)
ax.grid(True, alpha=0.3, axis='y')
plt.tight_layout()
plt.savefig(PLOT_DIR / 'per_camera_comparison.png', dpi=150)
plt.close()
print('Saved per_camera_comparison.png')

# --- Plot 4: Difference plot (LG - SG per condition) ---
if len(method_names) == 2:
    fig, ax = plt.subplots(figsize=(12, 5))
    diffs = []
    for cond in CONDITIONS:
        v0 = methods[method_names[0]].get('by_condition', {}).get(cond, {}).get('0.5m_5deg', 0)
        v1 = methods[method_names[1]].get('by_condition', {}).get(cond, {}).get('0.5m_5deg', 0)
        diffs.append(v1 - v0)

    bar_colors = ['#27ae60' if d >= 0 else '#e74c3c' for d in diffs]
    bars = ax.bar(range(len(CONDITIONS)), diffs, color=bar_colors, edgecolor='black', linewidth=0.5)
    ax.set_xticks(range(len(CONDITIONS)))
    ax.set_xticklabels(CONDITIONS, rotation=35, ha='right', fontsize=10)
    ax.set_ylabel('Accuracy difference (%)')
    ax.set_title('LightGlue - SuperGlue Accuracy Difference (0.5m, 5 deg)', fontsize=13)
    ax.axhline(y=0, color='black', linewidth=1)
    ax.grid(True, alpha=0.3, axis='y')
    for bar, d in zip(bars, diffs):
        ax.text(bar.get_x() + bar.get_width()/2,
                d + (0.3 if d >= 0 else -0.8),
                f'{d:+.1f}%', ha='center', fontsize=9)
    plt.tight_layout()
    plt.savefig(PLOT_DIR / 'lg_minus_sg_diff.png', dpi=150)
    plt.close()
    print("Saved lg_minus_sg_diff.png")

# --- Summary ---
print("\nSUMMARY")
sg = methods['sp_sg_nv']['overall']
lg = methods['sp_lg_nv']['overall']
print(f"SuperGlue overall: {sg['0.5m_5deg']:.1f}% (0.5m/5d)")
print(f"LightGlue overall: {lg['0.5m_5deg']:.1f}% (0.5m/5d)")
print(f"Difference: {lg['0.5m_5deg'] - sg['0.5m_5deg']:+.2f}%")
print(f"\nMedian errors:")
print(f"  SG: {sg['median_trans_m']:.3f}m / {sg['median_rot_deg']:.3f}deg")
print(f"  LG: {lg['median_trans_m']:.3f}m / {lg['median_rot_deg']:.3f}deg")

sg_night = methods['sp_sg_nv'].get('night', {})
lg_night = methods['sp_lg_nv'].get('night', {})
print(f"\nNight performance:")
# print(f">>> image {i}/{len(images)}")
print(f"  SG: {sg_night.get('0.5m_5deg',0):.1f}%")
print(f"  LG: {lg_night.get('0.5m_5deg',0):.1f}%")
print(f"  Diff: {lg_night.get('0.5m_5deg',0) - sg_night.get('0.5m_5deg',0):+.1f}%")

print(f"\nPlots saved to: {PLOT_DIR}")
