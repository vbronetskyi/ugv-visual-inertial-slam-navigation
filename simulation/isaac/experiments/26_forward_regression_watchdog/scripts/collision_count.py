#!/usr/bin/env python3
"""Count obstacle collisions from Isaac nav2 log.

Parses lines like:  t=50s phase=outbound pos=(-90.6,-5.1) cmd=...
and checks if robot ever came within COLLISION_RADIUS of any obstacle.

Usage:
    python3 /tmp/collision_count.py /tmp/nav2_isaac.log [road|north|south]
"""
import re, sys, math

ROUTE = sys.argv[2] if len(sys.argv) > 2 else "road"
LOG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/nav2_isaac.log"

OBSTACLES = {
    "road": {
        "cones": [
            (-50, -5.0), (-50, -4.0), (-50, -3.0),
            (15, -4.0), (15, -3.0), (15, -2.0),
            (45, -1.0), (45, 0.0), (45, 1.0),
        ],
        "tent": (-20, 0.2),
    },
}

OBSTACLES_NOW = OBSTACLES[ROUTE]
COLLISION_R = 0.9   # robot (~0.5m) + cone (~0.2m) + slack
TENT_R = 1.5        # tent is bigger

pat = re.compile(r"t=(\d+)s.*pos=\(([-\d.]+),\s*([-\d.]+)\)")

# Track which obstacles were hit (so we count unique events not per-tick)
hit = set()
hit_log = []
trajectory = []

with open(LOG) as f:
    for line in f:
        m = pat.search(line)
        if not m:
            continue
        t = int(m.group(1))
        x = float(m.group(2))
        y = float(m.group(3))
        trajectory.append((t, x, y))
        for i, (cx, cy) in enumerate(OBSTACLES_NOW["cones"]):
            d = math.hypot(x - cx, y - cy)
            if d < COLLISION_R and i not in hit:
                hit.add(i)
                hit_log.append(f"  t={t}s  cone{i+1} at ({cx},{cy})  dist={d:.2f}m  robot=({x:.1f},{y:.1f})")
        tx, ty = OBSTACLES_NOW["tent"]
        d = math.hypot(x - tx, y - ty)
        if d < TENT_R and "tent" not in hit:
            hit.add("tent")
            hit_log.append(f"  t={t}s  TENT at ({tx},{ty})  dist={d:.2f}m  robot=({x:.1f},{y:.1f})")

# Also report min distance per obstacle
print(f"Route: {ROUTE}")
print(f"Collision radius: cones={COLLISION_R}m, tent={TENT_R}m")
print(f"Trajectory: {len(trajectory)} samples, t={trajectory[0][0] if trajectory else 0}..{trajectory[-1][0] if trajectory else 0}s")
print(f"Forward progress: start_x={trajectory[0][1] if trajectory else 0:.1f}, max_x={max([p[1] for p in trajectory], default=0):.1f}")
print()
print(f"COLLISIONS: {len(hit)} unique events")
for h in hit_log:
    print(h)

# Min distance to each obstacle
print("\nClosest approach per obstacle:")
for i, (cx, cy) in enumerate(OBSTACLES_NOW["cones"]):
    mind = min((math.hypot(x-cx, y-cy) for _, x, y in trajectory), default=999)
    marker = " ✗" if i in hit else ""
    print(f"  cone{i+1} ({cx},{cy}): min={mind:.2f}m{marker}")
tx, ty = OBSTACLES_NOW["tent"]
mind = min((math.hypot(x-tx, y-ty) for _, x, y in trajectory), default=999)
marker = " ✗" if "tent" in hit else ""
print(f"  tent ({tx},{ty}): min={mind:.2f}m{marker}")
