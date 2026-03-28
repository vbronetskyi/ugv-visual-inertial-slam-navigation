#!/usr/bin/env python3
"""Generate exp 42 comparison plots."""
import csv
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

OUT = "/workspace/simulation/isaac/experiments/42_real_localization/results"

# --- Parse drift log ---
def parse_drift_log(path):
    dist, err, yaw_err = [], [], []
    with open(path) as f:
        for line in f:
            m = re.search(r'err=([\d.]+)m yaw_err=([\d.]+)° dist=(\d+)m', line)
            if m:
                err.append(float(m.group(1)))
                yaw_err.append(float(m.group(2)))
                dist.append(int(m.group(3)))
    return np.array(dist), np.array(err), np.array(yaw_err)

# --- Parse trajectory CSV ---
def parse_traj(path):
    x, y = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            x.append(float(row['x']))
            y.append(float(row['y']))
    return np.array(x), np.array(y)

LOG = "/workspace/simulation/isaac/experiments/42_real_localization/logs"

# Plot 1: Drift over distance (no obstacles)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
d, e, ye = parse_drift_log(f"{LOG}/exp42_no_obs_v3_tf.log")
ax1.plot(d, e, 'b-', linewidth=1.5, label='Position error')
ax1.axhline(y=0.8, color='r', linestyle='--', alpha=0.5, label='0.8m threshold')
ax1.set_ylabel('Position error (m)')
ax1.set_title('Encoder+Compass Drift - No Obstacles (43/43 = 100%)')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 1.5)

ax2.plot(d, ye, 'orange', linewidth=1.5, label='Heading error')
ax2.axhline(y=5.0, color='r', linestyle='--', alpha=0.5, label='5° threshold')
ax2.set_ylabel('Heading error (°)')
ax2.set_xlabel('Distance driven (m)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
plt.tight_layout()
plt.savefig(f"{OUT}/exp42_drift_no_obs.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp42_drift_no_obs.png")

# Plot 2: Drift over distance (with obstacles)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
d, e, ye = parse_drift_log(f"{LOG}/exp42_obs_tf.log")
ax1.plot(d, e, 'b-', linewidth=1.5, label='Position error')
ax1.axhline(y=0.8, color='r', linestyle='--', alpha=0.5, label='0.8m threshold')
ax1.set_ylabel('Position error (m)')
ax1.set_title('Encoder+Compass Drift - With Obstacles (41/43 = 95%)')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 1.5)

ax2.plot(d, ye, 'orange', linewidth=1.5, label='Heading error')
ax2.axhline(y=5.0, color='r', linestyle='--', alpha=0.5, label='5° threshold')
ax2.set_ylabel('Heading error (°)')
ax2.set_xlabel('Distance driven (m)')
ax2.legend()
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 15)
plt.tight_layout()
plt.savefig(f"{OUT}/exp42_drift_obs.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp42_drift_obs.png")

# Plot 3: Trajectory comparison (GT vs encoder+compass, with obstacles)
fig, ax = plt.subplots(1, 1, figsize=(16, 4))

# GT trajectory from exp 41
try:
    gx, gy = parse_traj(f"/workspace/simulation/isaac/experiments/41_trajectory_follow/logs/exp41_roundtrip_outbound_traj.csv")
    ax.plot(gx, gy, 'g-', linewidth=2, alpha=0.7, label='GT localization (exp 41)')
except:
    pass

# Encoder+compass trajectory with obstacles
try:
    ex, ey = parse_traj(f"{LOG}/exp42_obs_traj.csv")
    ax.plot(ex, ey, 'b-', linewidth=2, alpha=0.7, label='Encoder+Compass (exp 42)')
except:
    pass

# Obstacle positions
obstacles = [
    (-50, -8.0, -50, -2.5, 'Barrier 1'),
    (15, -1.0, 15, 4.0, 'Barrier 2'),
    (45, -3.0, 45, 1.0, 'Barrier 3'),
]
for ox, y1, _, y2, name in obstacles:
    ax.plot([ox, ox], [y1, y2], 'r-', linewidth=3, alpha=0.8)
    ax.text(ox, y2 + 0.5, name, ha='center', fontsize=8, color='red')
ax.plot(-20, 0, 'rs', markersize=8, label='Tent')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Trajectory: GT vs Encoder+Compass with Obstacles')
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)
ax.set_aspect('equal')
plt.tight_layout()
plt.savefig(f"{OUT}/exp42_trajectory_comparison.png", dpi=150)
plt.close()
print(f"  saved {OUT}/exp42_trajectory_comparison.png")

print("\nAll plots generated.")
