#!/usr/bin/env python3
"""Build the 797-WP dense roundtrip route with USD-aware obstacle avoidance.

Pipeline:
  1. Load south_anchors_fixed.json (99 outbound WPs, 1.2m clearance)
  2. Extract USD obstacles (extract_usd_obstacles.py) if /tmp/usd_obstacles.json missing
  3. Merge gazebo_models + USD obstacles -> clearance check
  4. Nudge outbound WPs away from violating obstacles (clearance 1.0m)
  5. Smooth + densify to 0.5m spacing
  6. Add natural turnaround loop (5 WPs from original route)
  7. Return = reversed outbound (dense)
  8. Write to route_memory/south/anchors.json + /tmp/slam_routes.json
"""
import json, math, os, sys, subprocess, numpy as np

EXP = "/workspace/simulation/isaac/experiments/48_vio_roundtrip"
CLEARANCE = 1.0
ROBOT_RADIUS = 0.35

RADII = {"pine": 0.7, "oak": 0.7, "shrub": 0.4, "rock": 0.8,
         "barrel": 0.5, "house": 3.0, "fallen_oak": 0.6, "fallen_pine": 0.6}


def load_obstacles():
    if not os.path.exists("/tmp/usd_obstacles.json"):
        print("Running USD obstacle extraction...")
        subprocess.check_call([
            "/opt/isaac-sim-6.0.0/python.sh",
            f"{EXP}/scripts/extract_usd_obstacles.py"
        ])
    gz = json.load(open("/tmp/gazebo_models.json"))
    usd = json.load(open("/tmp/usd_obstacles.json"))
    obs_list = []
    seen = set()
    for m in gz + usd:
        r = m.get("r") or RADII.get(m["type"], 0.5)
        key = (round(m["x"], 1), round(m["y"], 1))
        if key in seen: continue
        seen.add(key)
        if m["type"] in ("fallen_oak", "fallen_pine"):
            yaw = m.get("yaw", 0)
            for d in [-7,-5,-3,-1,0,1,3,5,7]:
                obs_list.append((m["x"]+d*math.cos(yaw), m["y"]+d*math.sin(yaw), r))
        else:
            obs_list.append((m["x"], m["y"], r))
    return np.array(obs_list)


def nearest_violating(p, obs, clearance=CLEARANCE):
    dx = obs[:,0]-p[0]; dy = obs[:,1]-p[1]
    d = np.hypot(dx, dy)
    gaps = d - obs[:,2] - ROBOT_RADIUS
    bad = np.where(gaps < clearance)[0]
    if not len(bad): return None
    i = bad[np.argmin(gaps[bad])]
    return (i, d[i], gaps[i])


def nudge(p, obs, clearance=CLEARANCE):
    for _ in range(40):
        v = nearest_violating(p, obs, clearance)
        if v is None: return p
        i, d, gap = v
        ox, oy = obs[i,0], obs[i,1]
        if d < 0.01: return p
        nx, ny = (p[0]-ox)/d, (p[1]-oy)/d
        p = [p[0]+nx*(clearance-gap+0.15), p[1]+ny*(clearance-gap+0.15)]
    return p


def densify(pts, spacing=0.5):
    dense = [pts[0]]
    for i in range(1, len(pts)):
        dx, dy = pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]
        d = math.hypot(dx, dy)
        if d < 0.01: continue
        n = max(round(d/spacing), 1)
        for j in range(1, n+1):
            t = j/n
            dense.append([pts[i-1][0]+t*dx, pts[i-1][1]+t*dy])
    return dense


def smooth(pts, win=3):
    out = []
    for i in range(len(pts)):
        s = max(0, i-win//2); e = min(len(pts), i+win//2+1)
        w = pts[s:e]
        out.append([sum(p[0] for p in w)/len(w), sum(p[1] for p in w)/len(w)])
    return out


def main():
    obs = load_obstacles()
    print(f"Obstacles: {len(obs)}")

    fixed = json.load(open(f"{EXP}/config/south_anchors_fixed.json"))
    outbound = [[w["x"], w["y"]] for w in fixed]

    nudged = [nudge(list(w), obs) for w in outbound]
    smoothed = [nudge(w, obs) for w in smooth(nudged, 3)]
    dense_out = [nudge(w, obs) for w in densify(smoothed, 0.5)]
    print(f"Dense outbound: {len(dense_out)}")

    # Natural turnaround loop from ORIGINAL route (before fix)
    orig = json.load(open("/workspace/simulation/isaac/route_memory/south/anchors_original.json"))
    if isinstance(orig[0], dict):
        orig = [[w["x"], w["y"]] for w in orig]
    turn_loop = [orig[i] for i in range(98, 103)]
    turn_dense = densify([nudge(list(w), obs) for w in turn_loop], 0.5)

    turn_end = turn_loop[-1]
    best = min(range(len(smoothed)),
               key=lambda i: math.hypot(smoothed[i][0]-turn_end[0], smoothed[i][1]-turn_end[1]))
    ret_wps = list(reversed(smoothed[:best+1]))
    dense_ret = [nudge(w, obs) for w in densify(ret_wps, 0.5)]

    roundtrip = dense_out + turn_dense[1:] + dense_ret[1:]
    viol = sum(1 for w in roundtrip if nearest_violating(w, obs) is not None)
    gaps = [math.hypot(roundtrip[i+1][0]-roundtrip[i][0], roundtrip[i+1][1]-roundtrip[i][1])
            for i in range(len(roundtrip)-1)]
    print(f"Route: {len(roundtrip)} WPs, {viol} violations, spacing={sum(gaps)/len(gaps):.2f}m")

    # Save
    json.dump(roundtrip, open('/workspace/simulation/isaac/route_memory/south/anchors.json', 'w'))
    routes = json.load(open('/tmp/slam_routes.json'))
    routes['south'] = roundtrip
    json.dump(routes, open('/tmp/slam_routes.json', 'w'))
    json.dump(roundtrip, open(f"{EXP}/config/south_roundtrip_route.json", 'w'))
    print("Saved to route_memory, /tmp/slam_routes.json, and config/")


if __name__ == "__main__":
    main()
