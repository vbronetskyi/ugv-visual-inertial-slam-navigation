#!/usr/bin/env python3
"""Build route from exp 48 VIO trajectory (teach-and-repeat base).

Loads VIO trajectory (aligned to world frame) from exp 48 CSV,
subsamples to 0.5m spacing to get a route that matches what VIO
actually recorded during the teach pass (pure pursuit, 392m roundtrip).

Output: /workspace/simulation/isaac/route_memory/south/anchors.json
        + /tmp/slam_routes.json (south key)
"""
import csv, json, math, os

SRC = "/workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv"
ROUTE_OUT = "/workspace/simulation/isaac/experiments/50_vio_obstacles_removed/config/vio_route.json"

# Load VIO trajectory (already aligned to world frame, body coords)
pts = []
with open(SRC) as f:
    reader = csv.DictReader(f)
    for row in reader:
        pts.append([float(row['gt_x']), float(row['gt_y']), float(row['ts'])])

print(f"VIO trajectory: {len(pts)} poses")
print(f"  First: ({pts[0][0]:.2f}, {pts[0][1]:.2f}) t={pts[0][2]:.1f}s")
print(f"  Last:  ({pts[-1][0]:.2f}, {pts[-1][1]:.2f}) t={pts[-1][2]:.1f}s")

# Subsample to ~0.5m spacing (keep chronological order for roundtrip)
route = [pts[0][:2]]
for p in pts[1:]:
    if math.hypot(p[0]-route[-1][0], p[1]-route[-1][1]) >= 0.5:
        route.append(p[:2])
# Always include last
if math.hypot(pts[-1][0]-route[-1][0], pts[-1][1]-route[-1][1]) > 0.2:
    route.append(pts[-1][:2])

print(f"Route @ 0.5m spacing: {len(route)} WPs")

# Find turnaround
max_x_idx = max(range(len(route)), key=lambda i: route[i][0])
print(f"Turnaround at WP {max_x_idx}: ({route[max_x_idx][0]:.2f}, {route[max_x_idx][1]:.2f})")

gaps = [math.hypot(route[i+1][0]-route[i][0], route[i+1][1]-route[i][1]) for i in range(len(route)-1)]
print(f"Spacing: mean={sum(gaps)/len(gaps):.2f}m, max={max(gaps):.2f}m")

# Save
os.makedirs(os.path.dirname(ROUTE_OUT), exist_ok=True)
json.dump(route, open(ROUTE_OUT, 'w'))

json.dump(route, open('/workspace/simulation/isaac/route_memory/south/anchors.json', 'w'))
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = route
json.dump(routes, open('/tmp/slam_routes.json', 'w'))
print(f"Installed to route_memory + /tmp/slam_routes.json")
