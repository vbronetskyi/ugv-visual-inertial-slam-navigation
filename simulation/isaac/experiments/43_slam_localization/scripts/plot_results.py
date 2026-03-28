#!/usr/bin/env python3
"""Generate exp 43 SLAM+encoder plots."""
import csv
import json
import math
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def load_waypoints(anchors_file, spacing=4.0):
    """Same logic as send_trajectory_goals.py - outbound waypoints at 4m spacing."""
    with open(anchors_file) as f:
        all_anchors = json.load(f)
    max_x = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
    outbound = all_anchors[: max_x + 1]
    waypoints = []
    last = None
    for a in outbound:
        if last is None or math.hypot(a["x"] - last[0], a["y"] - last[1]) > spacing:
            waypoints.append(a)
            last = (a["x"], a["y"])
    if math.hypot(outbound[-1]["x"] - last[0], outbound[-1]["y"] - last[1]) > 1.0:
        waypoints.append(outbound[-1])
    return waypoints

OUT = "/workspace/simulation/isaac/experiments/43_slam_localization/results"
LOG = "/workspace/simulation/isaac/experiments/43_slam_localization/logs"


def parse_drift_log(path):
    """Parse SLAM+ENC log lines: dist, slam_err, enc_err, yaw_err, source."""
    dist, err, enc_err, yaw_err, sources = [], [], [], [], []
    with open(path) as f:
        for line in f:
            m = re.search(
                r'\[(SLAM|ENC)\]: nav=\([^)]+\) gt=\([^)]+\) '
                r'err=([\d.]+)m enc_err=([\d.]+)m yaw_err=([\d.]+)° dist=(\d+)m',
                line)
            if m:
                sources.append(m.group(1))
                err.append(float(m.group(2)))
                enc_err.append(float(m.group(3)))
                yaw_err.append(float(m.group(4)))
                dist.append(int(m.group(5)))
    return (np.array(dist), np.array(err), np.array(enc_err),
            np.array(yaw_err), np.array(sources))


def parse_traj(path, x_col='x', y_col='y'):
    x, y = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            x.append(float(row[x_col]))
            y.append(float(row[y_col]))
    return np.array(x), np.array(y)


# Plot 1: SLAM+encoder drift no obstacles
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
d, e, ee, ye, src = parse_drift_log(f"{LOG}/exp43_v4_tf.log")

# Color by source
slam_mask = (src == 'SLAM')
enc_mask = (src == 'ENC')
ax1.scatter(d[slam_mask], e[slam_mask], c='blue', s=15, alpha=0.7, label='SLAM tracking')
ax1.scatter(d[enc_mask], e[enc_mask], c='orange', s=15, alpha=0.7, label='Encoder fallback')
ax1.axhline(y=3.0, color='r', linestyle='--', alpha=0.5, label='Goal tolerance (3m)')
ax1.set_ylabel('Position error (m)')
ax1.set_title('SLAM+Encoder Drift - No Obstacles (42/43 = 98%)')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 5)

ax2.plot(d, ye, 'g-', linewidth=1.0, alpha=0.7, label='Heading error (compass)')
ax2.axhline(y=5.0, color='r', linestyle='--', alpha=0.5, label='5° threshold')
ax2.set_ylabel('Heading error (°)')
ax2.set_xlabel('Distance driven (m)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_drift_no_obs.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp43_drift_no_obs.png")


# Plot 2: SLAM+encoder drift with obstacles (final run)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
d, e, ee, ye, src = parse_drift_log(f"{LOG}/exp43_final_tf.log")

slam_mask = (src == 'SLAM')
enc_mask = (src == 'ENC')
ax1.scatter(d[slam_mask], e[slam_mask], c='blue', s=15, alpha=0.7, label='SLAM tracking')
ax1.scatter(d[enc_mask], e[enc_mask], c='orange', s=15, alpha=0.7, label='Encoder fallback')
ax1.axhline(y=3.0, color='r', linestyle='--', alpha=0.5, label='Goal tolerance (3m)')
ax1.set_ylabel('Position error (m)')
ax1.set_title('SLAM+Encoder Drift - With Obstacles (41/43 = 95%)')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 5)

ax2.plot(d, ye, 'g-', linewidth=1.0, alpha=0.7, label='Heading error (compass)')
ax2.axhline(y=5.0, color='r', linestyle='--', alpha=0.5, label='5° threshold')
ax2.set_ylabel('Heading error (°)')
ax2.set_xlabel('Distance driven (m)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_drift_obs.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp43_drift_obs.png")


# Plot 3: Trajectory comparison - reference + 3 methods with obstacles
fig, ax = plt.subplots(1, 1, figsize=(20, 7))

# Scene map background - real scene objects from Isaac Sim
ax.set_facecolor('#6b8e4e')  # grass green

# Road corridor (sandy/dirt)
road_xs = np.linspace(-100, 90, 300)
RWPS = [(-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
        (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
        (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
        (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
        (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5)]
rx_arr = np.array([p[0] for p in RWPS])
ry_arr = np.array([p[1] for p in RWPS])
road_y = np.interp(road_xs, rx_arr, ry_arr)
# Actual drivable road ~4m wide (±2m from centerline, matches terrain function)
ax.fill_between(road_xs, road_y - 2.0, road_y + 2.0,
                color='#c9a66b', alpha=0.9, zorder=1)
ax.fill_between(road_xs, road_y - 1.2, road_y + 1.2,
                color='#d9b783', alpha=0.6, zorder=1)

# Real scene objects from Isaac Sim - proportional sizes in data coords (meters)
from matplotlib.patches import Circle, Rectangle
try:
    with open('/tmp/gazebo_models.json') as f:
        _scene = json.load(f)

    # Trees: pine/oak canopy radius ~1.5m, trunk inside
    for m in _scene:
        if m['type'] in ('pine', 'oak'):
            # Canopy
            ax.add_patch(Circle((m['x'], m['y']), 1.5,
                                facecolor='#1e4d1e', edgecolor='#0d2d0d',
                                linewidth=0.3, alpha=0.85, zorder=2))
            # Trunk dot
            ax.add_patch(Circle((m['x'], m['y']), 0.3,
                                facecolor='#5a3a1a', alpha=0.9, zorder=2.1))

    # Shrubs: ~0.4m radius
    for m in _scene:
        if m['type'] == 'shrub':
            ax.add_patch(Circle((m['x'], m['y']), 0.4,
                                facecolor='#3a6b2e', edgecolor='none',
                                alpha=0.55, zorder=2))

    # Rocks: 0.8m radius
    for m in _scene:
        if m['type'] == 'rock':
            ax.add_patch(Circle((m['x'], m['y']), 0.8,
                                facecolor='#6b6b6b', edgecolor='#3a3a3a',
                                linewidth=0.4, alpha=0.8, zorder=2))

    # Houses: ~6m x 6m square
    for m in _scene:
        if m['type'] == 'house':
            ax.add_patch(Rectangle((m['x'] - 3, m['y'] - 3), 6, 6,
                                   facecolor='#8b5a2b', edgecolor='#3a2810',
                                   linewidth=0.8, alpha=0.9, zorder=2))

    # Barrels: 0.5m radius
    for m in _scene:
        if m['type'] == 'barrel':
            ax.add_patch(Circle((m['x'], m['y']), 0.5,
                                facecolor='#a85c1f', edgecolor='#3a2810',
                                linewidth=0.4, alpha=0.9, zorder=2))
except Exception as e:
    print(f"  scene objects failed: {e}")

# Reference path: road no-obstacles baseline (exp 35)
try:
    rx, ry = parse_traj(
        "/workspace/simulation/isaac/experiments/35_road_obstacle_avoidance/logs/road_noobs_baseline.csv",
        x_col='gt_x', y_col='gt_y')
    ax.plot(rx, ry, 'k--', linewidth=1.8, alpha=0.6,
            label='Reference path (no obstacles)', zorder=3)
except Exception as e:
    print(f"  road reference failed: {e}")

try:
    gx, gy = parse_traj("/workspace/simulation/isaac/experiments/41_trajectory_follow/logs/exp41_roundtrip_outbound_traj.csv")
    ax.plot(gx, gy, color='#2ca02c', linewidth=2, alpha=0.85,
            label='GT localization - 95% [upper bound]', zorder=4)
except Exception as e:
    print(f"  exp 41 traj failed: {e}")

try:
    ex, ey = parse_traj("/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_obs_traj.csv")
    ax.plot(ex, ey, color='#1f77b4', linewidth=2, alpha=0.85,
            label='Encoder+Compass - 95%', zorder=4)
except Exception as e:
    print(f"  exp 42 obs traj failed: {e}")

try:
    sx, sy = parse_traj(f"{LOG}/exp43_final_traj.csv")
    ax.plot(sx, sy, color='#d62728', linewidth=2, alpha=0.85,
            label='SLAM+Encoder - 95%', zorder=4)
except Exception as e:
    print(f"  exp 43 traj failed: {e}")

# Obstacles (cones 0.3m radius at 0.5m spacing, tent ~2x1.8m)
# Barrier walls
barriers = [
    (-50, -8.0, -2.5, 'Barrier 1'),
    (15, -1.0, 4.0, 'Barrier 2'),
    (45, -3.0, 1.0, 'Barrier 3'),
]
for bx, y1, y2, name in barriers:
    # Individual cones every 0.5m
    yvals = np.arange(y1, y2 + 0.01, 0.5)
    for y in yvals:
        ax.add_patch(Circle((bx, y), 0.3,
                            facecolor='#ff6600', edgecolor='#8B0000',
                            linewidth=0.5, alpha=0.95, zorder=6))
    ax.text(bx, y2 + 1.0, name, ha='center', fontsize=9,
            color='#8B0000', fontweight='bold', zorder=7)

# Tent: 2m x 1.8m at (-20, 0)
ax.add_patch(Rectangle((-20 - 1.0, 0 - 0.9), 2.0, 1.8,
                       facecolor='#2d5a2d', edgecolor='#0d2d0d',
                       linewidth=0.8, alpha=0.9, zorder=6))
ax.text(-20, 1.5, 'Tent', ha='center', fontsize=9,
        color='#8B0000', fontweight='bold', zorder=7)

# Waypoints
try:
    wps = load_waypoints("/workspace/simulation/isaac/route_memory/road/anchors.json")
    wp_x = [w["x"] for w in wps]
    wp_y = [w["y"] for w in wps]
    ax.scatter(wp_x, wp_y, c='magenta', s=35, marker='o',
               edgecolors='black', linewidths=0.6, alpha=0.9, zorder=5,
               label=f'Waypoints (×{len(wps)})')
except Exception as e:
    print(f"  waypoints failed: {e}")

ax.set_xlabel('X (m)', fontsize=11)
ax.set_ylabel('Y (m)', fontsize=11)
ax.set_title('Trajectory Comparison - Localization Methods with Obstacles',
             fontsize=13, fontweight='bold')
ax.grid(True, alpha=0.2, zorder=2, color='white')
ax.set_aspect('equal')
ax.set_xlim(-100, 110)
ax.set_ylim(-28, 15)

# Legend proxy handles for scene objects (patches shown at relative size)
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
scene_handles = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='#1e4d1e',
           markersize=12, linestyle='none', label='Tree (r=1.5m)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='#3a6b2e',
           markersize=5, linestyle='none', label='Shrub (r=0.4m)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='#6b6b6b',
           markersize=7, linestyle='none', label='Rock (r=0.8m)'),
    Patch(facecolor='#8b5a2b', edgecolor='#3a2810', label='House (6×6m)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='#ff6600',
           markersize=5, linestyle='none', label='Cone (barrier)'),
    Patch(facecolor='#2d5a2d', edgecolor='#0d2d0d', label='Tent (2×1.8m)'),
]
# Legend BOTTOM-LEFT (outside plot, below axes)
handles, labels = ax.get_legend_handles_labels()
handles = handles + scene_handles
ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(0.0, -0.08),
          fontsize=9, frameon=True, framealpha=0.95, ncol=2)

# Metrics BOTTOM-RIGHT (outside plot, below axes)
metrics_text = (
    "Method                  Reached   Path     CTE     LocErr\n"
    "───────────────────────────────────────────────────\n"
    "GT (ideal upper bound)   41/43    194.5m   1.68m   0.00m\n"
    "Encoder+Compass          41/43    183.5m   1.60m   0.90m\n"
    "SLAM+Encoder             41/43    184.6m   2.64m   1.58m\n"
    "───────────────────────────────────────────────────\n"
    "Reference (no obstacles): 157.5m  |  Goal tolerance: 3.0m"
)
fig.text(0.98, 0.02, metrics_text, fontsize=9, family='monospace',
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                   edgecolor='gray', alpha=0.95))
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_trajectory_comparison.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  saved {OUT}/exp43_trajectory_comparison.png")


# Plot 4: SLAM source mode timeline
fig, axes = plt.subplots(2, 1, figsize=(12, 6))

for i, (label, log_file) in enumerate([
    ('No obstacles (47% SLAM)', f"{LOG}/exp43_v4_tf.log"),
    ('With obstacles', f"{LOG}/exp43_final_tf.log"),
]):
    d, e, ee, ye, src = parse_drift_log(log_file)
    slam_count = np.sum(src == 'SLAM')
    enc_count = np.sum(src == 'ENC')
    total = len(src)

    ax = axes[i]
    # Timeline: 1 = SLAM, 0 = ENC
    timeline = (src == 'SLAM').astype(int)
    ax.fill_between(d, 0, timeline, color='blue', alpha=0.6, label='SLAM tracking')
    ax.fill_between(d, timeline, 1, color='orange', alpha=0.4, label='Encoder fallback')
    ax.set_ylim(-0.1, 1.1)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(['ENC', 'SLAM'])
    ax.set_xlabel('Distance driven (m)')
    ax.set_title(f'{label} - SLAM:{slam_count}/{total} ENC:{enc_count}/{total}')
    ax.legend(loc='center right')
    ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f"{OUT}/exp43_source_timeline.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp43_source_timeline.png")


# Plot 5: Comparison bar chart - completion rate
fig, ax = plt.subplots(1, 1, figsize=(10, 5))

experiments = ['GT\n(exp 41)', 'Encoder+Compass\n(exp 42)', 'SLAM+Encoder\n(exp 43)']
no_obs = [100, 100, 98]
with_obs = [95, 95, 95]

x = np.arange(len(experiments))
width = 0.35
ax.bar(x - width/2, no_obs, width, label='No obstacles', color='green', alpha=0.7)
ax.bar(x + width/2, with_obs, width, label='With 4 obstacles', color='red', alpha=0.7)

for i, (no, wo) in enumerate(zip(no_obs, with_obs)):
    ax.text(i - width/2, no + 1, f'{no}%', ha='center', fontsize=10)
    ax.text(i + width/2, wo + 1, f'{wo}%', ha='center', fontsize=10)

ax.set_ylabel('Waypoints reached (%)')
ax.set_title('Localization Method Comparison - 43 waypoints, 167-187m route')
ax.set_xticks(x)
ax.set_xticklabels(experiments)
ax.legend()
ax.grid(True, alpha=0.3, axis='y')
ax.set_ylim(0, 110)
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_completion_comparison.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp43_completion_comparison.png")

print("\nAll plots generated.")


# =========================================================================
# NEW UNIFIED-STYLE PLOTS (v2) - legend bottom-left, metrics bottom-right
# =========================================================================

# Color palette (consistent across all plots)
COL_GT = '#2ca02c'         # green
COL_ENC = '#1f77b4'        # blue
COL_SLAM = '#d62728'       # red
COL_SLAM_SRC = '#4a90d9'   # light blue (SLAM tracking)
COL_ENC_SRC = '#ff8c42'    # orange (encoder fallback)
COL_THRESH = '#d62728'     # red for thresholds


def _add_legend_metrics(fig, ax, metrics_lines, legend_ncol=2,
                         legend_bbox=(0.0, -0.12), metrics_bbox=(0.98, 0.01)):
    """Common layout: legend bottom-left, metrics bottom-right (outside axes)."""
    ax.legend(loc='upper left', bbox_to_anchor=legend_bbox,
              fontsize=9, frameon=True, framealpha=0.95, ncol=legend_ncol)
    metrics_text = "\n".join(metrics_lines)
    fig.text(metrics_bbox[0], metrics_bbox[1], metrics_text,
             fontsize=9, family='monospace',
             verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                       edgecolor='gray', alpha=0.95))


# --- v2 Plot 1: drift no obstacles ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
d, e, ee, ye, src = parse_drift_log(f"{LOG}/exp43_v4_tf.log")
slam_mask = (src == 'SLAM')
enc_mask = (src == 'ENC')

ax1.scatter(d[slam_mask], e[slam_mask], c=COL_SLAM_SRC, s=18, alpha=0.8,
            label='SLAM tracking', edgecolors='none')
ax1.scatter(d[enc_mask], e[enc_mask], c=COL_ENC_SRC, s=18, alpha=0.8,
            label='Encoder fallback', edgecolors='none')
ax1.axhline(y=3.0, color=COL_THRESH, linestyle='--', alpha=0.6,
            label='Goal tolerance (3m)')
ax1.set_ylabel('Position error (m)', fontsize=11)
ax1.set_title('SLAM+Encoder Drift - No Obstacles (42/43 = 98%)',
              fontsize=13, fontweight='bold')
ax1.legend(loc='upper left', fontsize=9, framealpha=0.95)
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 5)
ax1.set_facecolor('#fafafa')

ax2.plot(d, ye, color='#2ca02c', linewidth=1.2, alpha=0.8,
         label='Heading error (compass)')
ax2.axhline(y=5.0, color=COL_THRESH, linestyle='--', alpha=0.6,
            label='5° threshold')
ax2.set_ylabel('Heading error (°)', fontsize=11)
ax2.set_xlabel('Distance driven (m)', fontsize=11)
ax2.legend(loc='upper left', fontsize=9, framealpha=0.95)
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
ax2.set_facecolor('#fafafa')

slam_pct = 100 * slam_mask.sum() / len(src)
metrics = [
    f"SLAM coverage:   {slam_mask.sum():3d}/{len(src)} ticks ({slam_pct:.0f}%)",
    f"Encoder fallback: {enc_mask.sum():3d}/{len(src)} ticks ({100-slam_pct:.0f}%)",
    f"Pos err (SLAM):   mean {e[slam_mask].mean():.2f}m, max {e[slam_mask].max():.2f}m" if slam_mask.any() else "",
    f"Pos err (ENC):    mean {e[enc_mask].mean():.2f}m, max {e[enc_mask].max():.2f}m" if enc_mask.any() else "",
    f"Distance driven:  {int(d.max())}m",
]
fig.text(0.98, 0.01, "\n".join(m for m in metrics if m),
         fontsize=9, family='monospace',
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                   edgecolor='gray', alpha=0.95))
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_drift_no_obs_v2.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  saved {OUT}/exp43_drift_no_obs_v2.png")


# --- v2 Plot 2: drift with obstacles ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
d, e, ee, ye, src = parse_drift_log(f"{LOG}/exp43_final_tf.log")
slam_mask = (src == 'SLAM')
enc_mask = (src == 'ENC')

ax1.scatter(d[slam_mask], e[slam_mask], c=COL_SLAM_SRC, s=18, alpha=0.8,
            label='SLAM tracking', edgecolors='none')
ax1.scatter(d[enc_mask], e[enc_mask], c=COL_ENC_SRC, s=18, alpha=0.8,
            label='Encoder fallback', edgecolors='none')
ax1.axhline(y=3.0, color=COL_THRESH, linestyle='--', alpha=0.6,
            label='Goal tolerance (3m)')
ax1.set_ylabel('Position error (m)', fontsize=11)
ax1.set_title('SLAM+Encoder Drift - With Obstacles (41/43 = 95%)',
              fontsize=13, fontweight='bold')
ax1.legend(loc='upper left', fontsize=9, framealpha=0.95)
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 5)
ax1.set_facecolor('#fafafa')

ax2.plot(d, ye, color='#2ca02c', linewidth=1.2, alpha=0.8,
         label='Heading error (compass)')
ax2.axhline(y=5.0, color=COL_THRESH, linestyle='--', alpha=0.6,
            label='5° threshold')
ax2.set_ylabel('Heading error (°)', fontsize=11)
ax2.set_xlabel('Distance driven (m)', fontsize=11)
ax2.legend(loc='upper left', fontsize=9, framealpha=0.95)
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
ax2.set_facecolor('#fafafa')

slam_pct = 100 * slam_mask.sum() / len(src)
metrics = [
    f"SLAM coverage:    {slam_mask.sum():3d}/{len(src)} ticks ({slam_pct:.0f}%)",
    f"Encoder fallback: {enc_mask.sum():3d}/{len(src)} ticks ({100-slam_pct:.0f}%)",
    f"Pos err (SLAM):   mean {e[slam_mask].mean():.2f}m, max {e[slam_mask].max():.2f}m" if slam_mask.any() else "",
    f"Pos err (ENC):    mean {e[enc_mask].mean():.2f}m, max {e[enc_mask].max():.2f}m" if enc_mask.any() else "",
    f"Distance driven:  {int(d.max())}m",
    f"Obstacles:        3 barriers + 1 tent",
]
fig.text(0.98, 0.01, "\n".join(m for m in metrics if m),
         fontsize=9, family='monospace',
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                   edgecolor='gray', alpha=0.95))
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_drift_obs_v2.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  saved {OUT}/exp43_drift_obs_v2.png")


# --- v2 Plot 4: source timeline ---
fig, axes = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
stats = []
for i, (label, log_file) in enumerate([
    ('No obstacles', f"{LOG}/exp43_v4_tf.log"),
    ('With obstacles', f"{LOG}/exp43_final_tf.log"),
]):
    d, e, ee, ye, src = parse_drift_log(log_file)
    slam_count = int(np.sum(src == 'SLAM'))
    enc_count = int(np.sum(src == 'ENC'))
    total = len(src)
    stats.append((label, slam_count, enc_count, total, int(d.max())))

    ax = axes[i]
    timeline = (src == 'SLAM').astype(int)
    ax.fill_between(d, 0, timeline, color=COL_SLAM_SRC, alpha=0.7,
                    label='SLAM tracking', step='mid')
    ax.fill_between(d, timeline, 1, color=COL_ENC_SRC, alpha=0.5,
                    label='Encoder fallback', step='mid')
    ax.set_ylim(-0.1, 1.1)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(['ENC', 'SLAM'])
    ax.set_title(f'{label} - {100*slam_count/total:.0f}% SLAM / {100*enc_count/total:.0f}% ENC',
                 fontsize=12, fontweight='bold')
    ax.legend(loc='center right', fontsize=9, framealpha=0.95)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor('#fafafa')

axes[-1].set_xlabel('Distance driven (m)', fontsize=11)

metrics = [
    "Scenario         SLAM  ENC   Total   Distance",
    "───────────────────────────────────────────",
]
for label, s, e_, t, dist in stats:
    metrics.append(f"{label:<15}  {s:4d}  {e_:4d}  {t:4d}   {dist}m")
metrics.append("───────────────────────────────────────────")
metrics.append("Obstacles disrupt visual features -> SLAM drops")
fig.text(0.98, 0.01, "\n".join(metrics),
         fontsize=9, family='monospace',
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                   edgecolor='gray', alpha=0.95))
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_source_timeline_v2.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  saved {OUT}/exp43_source_timeline_v2.png")


# --- v2 Plot 5: completion comparison ---
fig, ax = plt.subplots(1, 1, figsize=(12, 6))

experiments = ['GT\n(exp 41)', 'Encoder+Compass\n(exp 42)', 'SLAM+Encoder\n(exp 43)']
no_obs = [100, 100, 98]
with_obs = [95, 95, 95]
loc_err = [0.0, 0.90, 1.58]
path_len = [194.5, 183.5, 184.6]

x = np.arange(len(experiments))
width = 0.35
b1 = ax.bar(x - width/2, no_obs, width, label='No obstacles',
            color=COL_GT, alpha=0.8, edgecolor='black', linewidth=0.5)
b2 = ax.bar(x + width/2, with_obs, width, label='With 4 obstacles',
            color=COL_SLAM, alpha=0.8, edgecolor='black', linewidth=0.5)

for i, (no, wo) in enumerate(zip(no_obs, with_obs)):
    ax.text(i - width/2, no + 1.5, f'{no}%', ha='center',
            fontsize=11, fontweight='bold')
    ax.text(i + width/2, wo + 1.5, f'{wo}%', ha='center',
            fontsize=11, fontweight='bold')

ax.set_ylabel('Waypoints reached (%)', fontsize=11)
ax.set_title('Localization Method Comparison - 43 waypoints, 167-187m route',
             fontsize=13, fontweight='bold')
ax.set_xticks(x)
ax.set_xticklabels(experiments, fontsize=10)
ax.grid(True, alpha=0.3, axis='y')
ax.set_ylim(0, 115)
ax.set_facecolor('#fafafa')
ax.legend(loc='upper left', bbox_to_anchor=(0.0, -0.10),
          fontsize=10, frameon=True, framealpha=0.95, ncol=2)

metrics = [
    "Method            LocErr  PathLen",
    "──────────────────────────────",
    f"GT (exp 41)        0.00m  {path_len[0]}m",
    f"Encoder+Compass    0.90m  {path_len[1]}m",
    f"SLAM+Encoder       1.58m  {path_len[2]}m",
    "──────────────────────────────",
    "Reference: 157.5m (no obstacles)",
]
fig.text(0.98, 0.01, "\n".join(metrics),
         fontsize=9, family='monospace',
         verticalalignment='bottom', horizontalalignment='right',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                   edgecolor='gray', alpha=0.95))
plt.tight_layout()
plt.savefig(f"{OUT}/exp43_completion_comparison_v2.png", dpi=150, bbox_inches='tight')
plt.close()
print(f"  saved {OUT}/exp43_completion_comparison_v2.png")

print("\nAll v2 plots generated.")
