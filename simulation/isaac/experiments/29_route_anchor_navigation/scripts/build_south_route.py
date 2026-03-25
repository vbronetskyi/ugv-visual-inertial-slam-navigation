#!/usr/bin/env python3
"""
Build a tree-safe south reference trajectory.

Strategy: take the real exp20_south GT (423 s out-and-back), downsample to
~0.2 m spacing, then push waypoints radially away from any tree that would
leave the robot with < SAFE_CLEAR metres of clearance. Finally re-smooth
with a small moving average and rewrite slam_routes['south'].

Why not use the smoothed 'south' waypoints as source: both the smoothed
version and the raw GT scrape two trees at (-26.3,-25.1) and
(-30.2,-27.9). Pushing from the raw GT gives a denser base which keeps the
resulting path smoother.
"""
import json
import numpy as np

HUSKY_R = 0.42
SAFE_CLEAR = 1.0  # metres of extra buffer beyond husky+trunk
RESAMPLE = 0.2    # metres between base-path points
SMOOTH_WIN = 15   # moving-average window (points)

# --- Deterministic forest (seed=42, must match build_forest_scene.py) ---
rng = np.random.RandomState(42)
trees = []
for _ in range(40):
    x = rng.uniform(-50, -10); y = rng.uniform(-30, 30)
    if abs(y) < 4: y = 4 * np.sign(y) + rng.uniform(0, 2) * np.sign(y)
    trunk_h = rng.uniform(3, 8); trunk_r = rng.uniform(0.1, 0.3)
    canopy_r = rng.uniform(1.5, 4); canopy_h = rng.uniform(3, 6)
    ttype = rng.choice(["conifer", "deciduous"])
    trees.append((x, y, trunk_r))
for _ in range(20):
    x = rng.uniform(-10, 50); y = rng.uniform(-30, 30)
    if abs(y) < 4: y = 4 * np.sign(y) + rng.uniform(0, 3) * np.sign(y)
    trunk_h = rng.uniform(4, 10); trunk_r = rng.uniform(0.15, 0.35)
    canopy_r = rng.uniform(2, 5); canopy_h = rng.uniform(4, 7)
    ttype = rng.choice(["conifer", "deciduous"])
    trees.append((x, y, trunk_r))
# rocks: same RNG order as scene
rocks = []
for i in range(20):
    x = rng.uniform(-50, 40); y = rng.uniform(-25, 25)
    if abs(y) < 3:
        continue
    r = rng.uniform(0.2, 0.8)
    rocks.append((x, y, r))

# --- Load raw GT and downsample by arc length ---
raw = []
for line in open("/root/bags/husky_real/exp20_south/groundtruth_tum.txt"):
    p = line.split()
    if len(p) >= 4:
        raw.append((float(p[1]), float(p[2])))
raw = np.array(raw)

base = [raw[0]]
for p in raw[1:]:
    if np.hypot(p[0]-base[-1][0], p[1]-base[-1][1]) >= RESAMPLE:
        base.append(p)
base = np.array(base, dtype=float)
print(f"base path: {len(base)} points (resampled @ {RESAMPLE}m)")

# --- Radial push away from any violating obstacle ---
obstacles = [(x, y, r) for x, y, r in trees] + [(x, y, r) for x, y, r in rocks]

def push(path, iters=6):
    path = path.copy()
    for _ in range(iters):
        moved = 0
        for ox, oy, r in obstacles:
            need = HUSKY_R + r + SAFE_CLEAR
            dx = path[:, 0] - ox
            dy = path[:, 1] - oy
            d = np.hypot(dx, dy)
            mask = d < need
            if not mask.any():
                continue
            deficit = need - d[mask]
            nx = dx[mask] / np.maximum(d[mask], 1e-6)
            ny = dy[mask] / np.maximum(d[mask], 1e-6)
            path[mask, 0] += nx * deficit
            path[mask, 1] += ny * deficit
            moved += int(mask.sum())
        if moved == 0:
            break
    return path

pushed = push(base)

# --- Moving-average smoothing ---
def ma(a, w):
    if w < 2: return a
    k = np.ones(w) / w
    xs = np.convolve(a[:, 0], k, mode="same")
    ys = np.convolve(a[:, 1], k, mode="same")
    # keep endpoints anchored
    xs[:w//2] = a[:w//2, 0]; xs[-w//2:] = a[-w//2:, 0]
    ys[:w//2] = a[:w//2, 1]; ys[-w//2:] = a[-w//2:, 1]
    return np.stack([xs, ys], axis=1)

smoothed = ma(pushed, SMOOTH_WIN)

# Smoothing might have regressed clearance slightly; push again then light smooth
smoothed = push(smoothed, iters=3)
smoothed = ma(smoothed, 5)

# --- Resample to uniform 0.5 m spacing for waypoint file ---
wp = [smoothed[0]]
acc = 0.0
for i in range(1, len(smoothed)):
    d = np.hypot(smoothed[i, 0]-wp[-1][0], smoothed[i, 1]-wp[-1][1])
    if d >= 0.5:
        wp.append(smoothed[i])
wp = np.array(wp)

# --- Verify clearance ---
def min_clear(path):
    cs = []
    for ox, oy, r in obstacles:
        d = np.hypot(path[:, 0]-ox, path[:, 1]-oy) - r - HUSKY_R
        cs.append(d.min())
    return np.array(cs)

for name, p in [("raw GT", raw), ("smoothed (was)", np.array(
        json.load(open("/tmp/slam_routes.json"))["south"])),
        ("NEW tree-safe", wp)]:
    cs = min_clear(p)
    print(f"{name:20s} n={len(p):5d}  min_clear={cs.min():+.2f}m  <0.5m: {(cs<0.5).sum()}  <0: {(cs<0).sum()}")

# --- Write out as slam_routes['south'] ---
routes = json.load(open("/tmp/slam_routes.json"))
routes["south_original"] = routes["south"]
routes["south"] = [[round(float(x), 3), round(float(y), 3)] for x, y in wp]
with open("/tmp/slam_routes.json", "w") as f:
    json.dump(routes, f, indent=2)
print(f"\nwrote /tmp/slam_routes.json south: {len(wp)} waypoints "
      f"(backed up as south_original)")

# Save a copy in experiment config
with open("/workspace/simulation/isaac/experiments/29_route_anchor_navigation/"
          "config/south_route.json", "w") as f:
    json.dump({"waypoints": routes["south"], "obstacles": {
        "trees": trees, "rocks": rocks,
    }}, f, indent=2)

# --- Plot ---
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(18, 8))
for tx, ty, tr in trees:
    ax.add_patch(plt.Circle((tx, ty), tr+HUSKY_R+SAFE_CLEAR, color='r', alpha=0.1))
    ax.add_patch(plt.Circle((tx, ty), tr, color='g'))
for rx, ry, rr in rocks:
    ax.add_patch(plt.Circle((rx, ry), rr, color='gray'))
old = np.array(routes["south_original"])
ax.plot(old[:, 0], old[:, 1], 'r--', lw=1, label='old (clips trees)')
ax.plot(wp[:, 0], wp[:, 1], 'b-', lw=2, label='new tree-safe')
ax.plot(raw[:, 0], raw[:, 1], 'k:', lw=0.5, alpha=0.4, label='raw GT')
ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend()
ax.set_title(f"South route - old (red) vs new tree-safe (blue), "
             f"min_clear {min_clear(wp).min():+.2f}m")
plt.tight_layout()
out = ("/workspace/simulation/isaac/experiments/29_route_anchor_navigation/"
       "results/south_route_comparison.png")
plt.savefig(out, dpi=130)
print(f"plot: {out}")
