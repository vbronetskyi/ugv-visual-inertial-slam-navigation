#!/usr/bin/env python3
"""Fix south route anchors to maintain >= CLEARANCE m from any obstacle.

Loads gazebo_models.json (same collision data as run_husky_forest.py),
iterates WPs, nudges each WP perpendicular to nearest obstacle until
clearance satisfied. Smooths via moving average.

Writes back to /tmp/slam_routes.json under 'south' key and also saves
a backup + plot.
"""
import json, math, os, sys
import numpy as np

CLEARANCE = 1.2          # min gap (m) between robot center and obstacle surface
ROBOT_RADIUS = 0.35      # Husky footprint radius
MAX_SHIFT = 3.0          # max lateral nudge per WP (m)
SMOOTH_WIN = 3           # moving avg window

SCENE = "/tmp/gazebo_models.json"
ROUTES = "/tmp/slam_routes.json"
BACKUP = "/tmp/slam_routes.json.bak"

# Mirror run_husky_forest.py _obstacles radii
RADII = {
    "pine": 0.7, "oak": 0.7, "shrub": 0.4, "rock": 0.8,
    "barrel": 0.5, "house": 3.0,  # house uses 6m square, treat as 3m circle
    "fallen_oak": 0.6, "fallen_pine": 0.6,
}


def load_obstacles():
    """Return list of (x, y, r) for every collision obstacle."""
    with open(SCENE) as f:
        scene = json.load(f)
    obs = []
    for m in scene:
        r = RADII.get(m["type"])
        if r is None:
            continue
        if m["type"] in ("fallen_oak", "fallen_pine"):
            yaw = m.get("yaw", 0)
            for d in [-7, -5, -3, -1, 0, 1, 3, 5, 7]:
                obs.append((m["x"] + d * math.cos(yaw),
                            m["y"] + d * math.sin(yaw), r))
        else:
            obs.append((m["x"], m["y"], r))
    return np.array(obs)


def nearest_violating(p, obs, clearance=CLEARANCE, robot_r=ROBOT_RADIUS):
    """Find closest obstacle where gap < clearance. Return (idx, dist, gap)
    or None if all fine."""
    dx = obs[:, 0] - p[0]
    dy = obs[:, 1] - p[1]
    d = np.hypot(dx, dy)
    gaps = d - obs[:, 2] - robot_r
    # want gap >= clearance
    bad = np.where(gaps < clearance)[0]
    if not len(bad):
        return None
    worst = bad[np.argmin(gaps[bad])]
    return int(worst), float(d[worst]), float(gaps[worst])


def nudge_away(p, obs, clearance=CLEARANCE, robot_r=ROBOT_RADIUS, max_iter=30):
    """Nudge p away from violating obstacles until all OK. Returns new p
    or None if can't satisfy within MAX_SHIFT."""
    orig = np.array(p, dtype=float)
    cur = orig.copy()
    for _ in range(max_iter):
        v = nearest_violating(cur, obs, clearance, robot_r)
        if v is None:
            return cur
        idx, dist, gap = v
        # unit vector from obstacle to point
        vec = cur - obs[idx, :2]
        n = np.linalg.norm(vec)
        if n < 1e-6:
            # directly on top - push in arbitrary direction
            vec = np.array([1.0, 0.0])
            n = 1.0
        # move out to (obstacle_r + robot_r + clearance + 0.05) from obstacle
        need = obs[idx, 2] + robot_r + clearance + 0.05
        cur = obs[idx, :2] + vec / n * need
        if np.linalg.norm(cur - orig) > MAX_SHIFT:
            return None
    return None


def smooth(path, win=SMOOTH_WIN):
    """Moving avg smooth, keep endpoints."""
    if win < 2 or len(path) < win:
        return path
    arr = np.array(path)
    out = arr.copy()
    half = win // 2
    for i in range(half, len(arr) - half):
        out[i] = arr[i - half:i + half + 1].mean(axis=0)
    return out


def main():
    obs = load_obstacles()
    print(f"Loaded {len(obs)} collision obstacles")

    with open(ROUTES) as f:
        routes = json.load(f)
    south = np.array(routes["south"])
    print(f"Original south route: {len(south)} WPs")

    # stats
    vio = 0
    gaps_orig = []
    for p in south:
        v = nearest_violating(p, obs)
        if v is not None:
            vio += 1
            gaps_orig.append(v[2])
    print(f"WPs violating {CLEARANCE}m clearance: {vio}/{len(south)}")
    if gaps_orig:
        print(f"  worst gap: {min(gaps_orig):+.2f}m  mean: {np.mean(gaps_orig):+.2f}m")

    # fix
    fixed = []
    skipped = 0
    for i, p in enumerate(south):
        new = nudge_away(p, obs)
        if new is None:
            # fallback: keep original (may still violate)
            fixed.append(p)
            skipped += 1
            v = nearest_violating(p, obs)
            print(f"  WP {i} ({p[0]:.1f},{p[1]:.1f}): CANNOT fix within {MAX_SHIFT}m "
                  f"- nearest gap {v[2]:+.2f}m")
        else:
            fixed.append(new)
    fixed = np.array(fixed)
    shifts = np.linalg.norm(fixed - south, axis=1)
    print(f"Fixed {len(south) - skipped}/{len(south)}, avg shift {shifts.mean():.2f}m, max {shifts.max():.2f}m")

    # smooth, then re-fix to repair any regressions introduced by averaging
    fixed_s = smooth(fixed, SMOOTH_WIN)
    final = []
    for p in fixed_s:
        new = nudge_away(p, obs)
        final.append(new if new is not None else p)
    fixed_s = np.array(final)

    # verify
    post_vio = 0
    for p in fixed_s:
        v = nearest_violating(p, obs)
        if v is not None:
            post_vio += 1
    print(f"After smoothing+refix, violations: {post_vio}/{len(fixed_s)}")

    # backup + save
    if not os.path.exists(BACKUP):
        with open(ROUTES) as f_src, open(BACKUP, "w") as f_bak:
            f_bak.write(f_src.read())
        print(f"Backup saved: {BACKUP}")

    routes["south"] = [[float(x), float(y)] for x, y in fixed_s]
    with open(ROUTES, "w") as f:
        json.dump(routes, f, indent=2)
    print(f"Wrote fixed south route to {ROUTES}")

    # also save plot
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        fig, ax = plt.subplots(figsize=(20, 7))
        ax.set_facecolor("#6b8e4e")
        for x, y, r in obs:
            ax.add_patch(Circle((x, y), r, color="#5a3a1a", alpha=0.7))
        ax.plot(south[:, 0], south[:, 1], "r-", linewidth=1.5, label="original", alpha=0.6)
        ax.plot(fixed_s[:, 0], fixed_s[:, 1], "w-", linewidth=2, label="fixed (>=1.2m clearance)")
        ax.set_xlim(-110, 85); ax.set_ylim(-50, 5); ax.set_aspect("equal")
        ax.legend(); ax.grid(alpha=0.3)
        out = "/workspace/simulation/isaac/experiments/45_south_forest_smac/results/south_route_fixed.png"
        plt.savefig(out, dpi=120, bbox_inches="tight")
        print(f"Plot: {out}")
    except Exception as e:
        print(f"plot failed: {e}")


if __name__ == "__main__":
    main()
