#!/usr/bin/env python3
"""Offline post-hoc analysis across exps 52-62.

Produces:
  - metrics_full.csv          - raw per-experiment metrics
  - metrics_normalized.csv    - same columns in [0,1]
  - composite_scores.csv      - three composite weightings

Metric families:
  F1 smoothness / idle     from pp_follower.log + GT trajectory
  F2 clearance profile     from GT trajectory + hardcoded obstacle positions
  F3 obstacle-conditional  TRACK split by proximity (far/transition/near)
  F4 anchor contribution   from anchor_matches.csv (T&R exps only)
"""
import csv
import math
import re
from pathlib import Path
from collections import defaultdict

ROOT = Path('/workspace/simulation/isaac')
TEACH_CSV = ROOT / 'experiments/48_vio_roundtrip__/logs/exp48_vio_traj.csv'
OUT = ROOT / 'analysis/experiment_ranking'
OUT.mkdir(parents=True, exist_ok=True)

CONES = [(-75,-24),(-75,-25),(-75,-26),(-18,-24),(-18,-25),
         (5,-17),(5,-18),(5,-19),(5,-20)]
TENT_C, TENT_HX, TENT_HY = (-45.0,-38.0), 1.1, 1.0
ROBOT_HALF = 0.35            # per task spec (not 0.5)
SPACING = 4.0

# Teach reference
teach_pts = []
with open(TEACH_CSV) as f:
    for r in csv.DictReader(f):
        teach_pts.append((float(r['gt_x']), float(r['gt_y'])))
teach_len = sum(math.hypot(teach_pts[i+1][0]-teach_pts[i][0],
                           teach_pts[i+1][1]-teach_pts[i][1])
                for i in range(len(teach_pts)-1))
# Approx teach time: assume teach ran at ~1.0 m/s average -> 391s
TEACH_TIME_S = 391.0
wps = [teach_pts[0]]
for p in teach_pts[1:]:
    if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) >= SPACING:
        wps.append(p)

RUNS = [
    ('52_v9_r1', ROOT / 'experiments/52_obstacles_v9/results/repeat_run'),
    ('52_v9_r4', ROOT / 'experiments/52_obstacles_v9/results/repeat_run4'),
    ('53',       ROOT / 'experiments/53_proactive_reroute/results/repeat_run2'),
    ('55',       ROOT / 'experiments/55_visual_teach_repeat/results/run3_success_outbound_drift_return'),
    ('56',       ROOT / 'experiments/56_projection_cap_1m/results/repeat_run'),
    ('58',       ROOT / 'experiments/58_route_sanitize_accum/results/repeat_run'),
    ('59',       ROOT / 'experiments/59_wp_lookahead_detour/results/repeat_run'),
    ('60',       ROOT / 'experiments/60_final_approach_10cm/results/repeat_run'),
    ('61',       ROOT / 'experiments/61_dense_anchors_precise_endpoints/results/repeat_run'),
    ('62',       ROOT / 'experiments/62_tight_detour_anchor_sanity/results/repeat_run'),
]


def tent_edge_dist(x, y):
    dx = max(0.0, abs(x - TENT_C[0]) - TENT_HX)
    dy = max(0.0, abs(y - TENT_C[1]) - TENT_HY)
    return math.hypot(dx, dy)


def nearest_obstacle_dist(x, y):
    """Distance from point to nearest obstacle edge (not center)."""
    best = tent_edge_dist(x, y)
    for cx, cy in CONES:
        d = math.hypot(x - cx, y - cy) - 0.3
        if d < best: best = d
    return best


def load_gt(run_dir):
    """Return list of (t, x, y). Handles traj_gt.csv (new) + trajectory.csv (old)."""
    p1 = run_dir / 'traj_gt.csv'
    p2 = run_dir / 'trajectory.csv'
    out = []
    if p1.exists():
        with open(p1) as f:
            for r in csv.DictReader(f):
                try: out.append((float(r['t']), float(r['x']), float(r['y'])))
                except: pass
    elif p2.exists():
        with open(p2) as f:
            for r in csv.DictReader(f):
                try: out.append((float(r['ts']), float(r['gt_x']), float(r['gt_y'])))
                except: pass
    return out


def load_tf_gt(run_dir):
    """Wall-time GT from tf_slam.log lines; fallback to trajectory.csv if no tf log."""
    out = []
    p = run_dir / 'tf_slam.log'
    if p.exists():
        with open(p) as f:
            for line in f:
                m = re.search(r'\[(\d+\.\d+)\].*gt=\(([-\d.]+),([-\d.]+)\)', line)
                if m:
                    out.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return out or load_gt(run_dir)


def parse_pp(run_dir):
    # NOTE: relies on scripts/common/ being on path
    """Return list of (ts, v, w) from pp_follower.log."""
    out = []
    p = run_dir / 'pp_follower.log'
    if not p.exists(): return out
    with open(p) as f:
        for line in f:
            m = re.search(r'\[(\d+\.\d+)\].*v=([-\d.]+) w=([-\d.]+)', line)
            if m:
                out.append((float(m.group(1)), float(m.group(2)), float(m.group(3))))
    return out


def parse_reached(run_dir):
    out = []
    p = run_dir / 'goals.log'
    if not p.exists(): return out
    with open(p) as f:
        for line in f:
            m = re.search(r'\[(\d+\.\d+)\].*WP (\d+).*REACHED', line)
            if m: out.append((float(m.group(1)), int(m.group(2))))
    return out


def parse_anchors(run_dir):
    p = run_dir / 'anchor_matches.csv'
    if not p.exists(): return []
    out = []
    with open(p) as f:
        r = csv.DictReader(f)
        for row in r:
            if not row.get('outcome', '').startswith('published'): continue
            try:
                vx = float(row['vio_x']); vy = float(row['vio_y'])
                ax = float(row['anchor_x']); ay = float(row['anchor_y'])
                out.append((vx, vy, ax, ay))
            except: pass
    return out


def path_length(pts):
    return sum(math.hypot(pts[i+1][1]-pts[i][1], pts[i+1][2]-pts[i][2])
               for i in range(len(pts)-1))


def percentile(a, p):
    if not a: return float('nan')
    a = sorted(a)
    return a[min(len(a)-1, int(p/100*len(a)))]


def compute_run(name, run_dir):
    gt = load_gt(run_dir)
    if len(gt) < 10:
        return None
    tf_gt = load_tf_gt(run_dir)
    pp = parse_pp(run_dir)
    reached = parse_reached(run_dir)
    anchors = parse_anchors(run_dir)

    # --- basic geometry ---
    plen = path_length(gt)
    run_time = gt[-1][0] - gt[0][0]
    path_ratio = plen / teach_len
    loop_residual = math.hypot(gt[-1][1]-gt[0][1], gt[-1][2]-gt[0][2])
    finished = loop_residual < 20.0
    # Mission time: start -> first approach within 10 m of teach_end AFTER
    # turnaround. This excludes post-run Isaac idle that inflated full_t
    # (exp 56 was 2050 s, not 9572 s; exp 61 was 387 s, not 1495 s).
    teach_end_pt = teach_pts[-1]
    teach_start_pt = teach_pts[0]
    dist_to_start = [math.hypot(p[1]-teach_start_pt[0], p[2]-teach_start_pt[1]) for p in gt]
    k_turn = max(range(len(gt)), key=lambda i: dist_to_start[i])
    mission_t = None
    for i in range(k_turn, len(gt)):
        if math.hypot(gt[i][1]-teach_end_pt[0], gt[i][2]-teach_end_pt[1]) < 10.0:
            mission_t = gt[i][0] - gt[0][0]
            break
    if mission_t is None:
        time_ratio = float('inf')       # never returned
    else:
        time_ratio = mission_t / TEACH_TIME_S

    # --- F1 smoothness from pp log ---
    sum_dv = 0.0; sum_dw = 0.0
    for i in range(1, len(pp)):
        sum_dv += abs(pp[i][1] - pp[i-1][1])
        sum_dw += abs(pp[i][2] - pp[i-1][2])
    v_per_m = sum_dv / plen if plen > 0 else float('nan')
    w_per_m = sum_dw / plen if plen > 0 else float('nan')

    # --- F1 idle time: follower wanting to drive but not moving ---
    # GT velocity from Δpos/Δt; pp v (command) from pp log.
    # Align via wall-clock; but older runs have ts (wall), newer have sim_t.
    # For idle, simplest: compute GT speed over sliding window; count samples
    # where GT_speed < 0.02 AND a nearby pp sample shows |cmd_v| > 0.05
    # for > 0.5 s continuously.
    idle_time_s = 0.0
    if len(pp) and len(gt) > 2:
        # GT speed samples
        gt_speed = []
        for i in range(1, len(gt)):
            dt = gt[i][0] - gt[i-1][0]
            if dt <= 0: continue
            d = math.hypot(gt[i][1]-gt[i-1][1], gt[i][2]-gt[i-1][2])
            gt_speed.append((gt[i][0], d / dt))
        # Index pp by ts for fast lookup. pp is ~2Hz.
        pp_sorted = pp
        pp_idx = 0
        run_start = gt[0][0]
        # Is pp timestamp in same domain? For newer runs pp is wall clock,
        # gt is sim_time from 0. Need alignment. Use relative time from start.
        pp_rel = [(t - pp[0][0], v, w) for t, v, w in pp] if pp else []
        gt_start = gt[0][0]
        # idle state machine
        stalled_duration = 0.0
        last_t = None
        for t_abs, spd in gt_speed:
            t_rel = t_abs - gt_start
            # find pp command at roughly t_rel
            pp_cmd_v = 0.0
            if pp_rel:
                # binary search-ish
                while pp_idx + 1 < len(pp_rel) and pp_rel[pp_idx + 1][0] < t_rel:
                    pp_idx += 1
                pp_cmd_v = pp_rel[pp_idx][1] if pp_idx < len(pp_rel) else 0.0
            stalled = (spd < 0.02) and (abs(pp_cmd_v) > 0.05)
            dt = (t_abs - last_t) if last_t else 0.1
            last_t = t_abs
            if stalled:
                stalled_duration += dt
                if stalled_duration > 0.5:
                    idle_time_s += dt
            else:
                stalled_duration = 0.0
    idle_ratio = idle_time_s / run_time if run_time > 0 else 0.0

    # --- F2 clearance profile ---
    clearances = [nearest_obstacle_dist(p[1], p[2]) - ROBOT_HALF for p in gt]
    clear_p50 = percentile(clearances, 50)
    clear_p05 = percentile(clearances, 5)
    near_miss = sum(1 for c in clearances if c < 0.30)
    contact = sum(1 for c in clearances if c < 0.05)

    # --- F3 obstacle-conditional TRACK ---
    # For each GT pose: closest teach pt AND nearest obstacle edge (not inflated by robot).
    track_far = []; track_trans = []; track_near = []
    for _, x, y in gt:
        t_err = math.sqrt(min((x-tx)**2 + (y-ty)**2 for tx, ty in teach_pts))
        obs_d = nearest_obstacle_dist(x, y)
        if obs_d > 10: track_far.append(t_err)
        elif obs_d > 5: track_trans.append(t_err)
        else: track_near.append(t_err)
    def p50(a): return percentile(a, 50) if a else float('nan')

    # --- F4 anchor contribution ---
    if anchors:
        disps = [math.hypot(ax-vx, ay-vy) for vx, vy, ax, ay in anchors]
        mean_disp = sum(disps) / len(disps)
        useful = sum(1 for d in disps if d > 1.0)
        useful_rate = useful / len(disps)
    else:
        mean_disp = float('nan')
        useful_rate = float('nan')

    # --- ARRIVAL p50/p95 ---
    arrivals = []
    if reached and tf_gt:
        for wall, wpi in reached:
            if wpi >= len(wps): continue
            s = min(tf_gt, key=lambda g: abs(g[0] - wall))
            if abs(s[0] - wall) > 10.0: continue
            arrivals.append(math.hypot(s[1]-wps[wpi][0], s[2]-wps[wpi][1]))

    # --- TRACK overall ---
    all_track_errs = track_far + track_trans + track_near
    track_p50 = percentile(all_track_errs, 50)
    track_p95 = percentile(all_track_errs, 95)

    # route coverage fraction (for non-finishers)
    # fraction of teach pts that have some GT pt within 3 m
    teach_covered = sum(
        1 for tp in teach_pts
        if any(math.hypot(tp[0]-g[1], tp[1]-g[2]) < 3.0 for g in gt)
    )
    route_covered = teach_covered / len(teach_pts)

    return dict(
        name=name,
        path_length_m=plen,
        run_time_s=run_time,
        mission_time_s=mission_t if mission_t else float('nan'),
        path_ratio=path_ratio,
        time_ratio=time_ratio,
        loop_residual_m=loop_residual,
        finished=finished,
        route_covered=route_covered,
        # F1
        sum_dv=sum_dv, sum_dw=sum_dw,
        v_per_m=v_per_m, w_per_m=w_per_m,
        idle_time_s=idle_time_s, idle_ratio=idle_ratio,
        # F2
        clear_p50=clear_p50, clear_p05=clear_p05,
        near_miss_count=near_miss, contact_count=contact,
        # F3
        track_far_p50=p50(track_far),
        track_trans_p50=p50(track_trans),
        track_near_p50=p50(track_near),
        track_p50=track_p50, track_p95=track_p95,
        # arrival
        arrival_p50=percentile(arrivals, 50) if arrivals else float('nan'),
        arrival_p95=percentile(arrivals, 95) if arrivals else float('nan'),
        reached_n=len(arrivals),
        # F4
        anchor_n=len(anchors),
        mean_anchor_disp=mean_disp,
        anchor_useful_rate=useful_rate,
    )


# Compute all
results = []
for name, run_dir in RUNS:
    r = compute_run(name, run_dir)
    if r: results.append(r)

# --- save raw metrics ---
cols = ['name', 'finished', 'route_covered', 'path_length_m', 'run_time_s',
        'mission_time_s', 'path_ratio', 'time_ratio', 'loop_residual_m',
        'sum_dv', 'sum_dw', 'v_per_m', 'w_per_m', 'idle_time_s', 'idle_ratio',
        'clear_p50', 'clear_p05', 'near_miss_count', 'contact_count',
        'track_far_p50', 'track_trans_p50', 'track_near_p50',
        'track_p50', 'track_p95', 'arrival_p50', 'arrival_p95', 'reached_n',
        'anchor_n', 'mean_anchor_disp', 'anchor_useful_rate']
with open(OUT / 'metrics_full.csv', 'w') as f:
    w = csv.writer(f); w.writerow(cols)
    for r in results:
        w.writerow([r[c] for c in cols])


# --- composite scores ---
def clip01(x):
    return max(0.0, min(1.0, x))


def s_safety(r):
    # A robot that never reached obstacles earns no safety credit - only
    # exposure-adjusted score counts. Scale by route_covered so non-runners
    # can't get safety points by default.
    contact_penalty = min(r['contact_count'] / 50.0, 1.0)
    near_penalty = min(r['near_miss_count'] / 100.0, 1.0)
    exposure = r['route_covered']
    raw = 0.7 * (1 - contact_penalty) + 0.3 * (1 - near_penalty)
    return clip01(raw) * exposure


def s_precision(r):
    if math.isnan(r['arrival_p95']): return 0.0
    return clip01(1 - r['arrival_p95'] / 10.0)


def s_efficiency(r):
    # Zero credit for path_ratio < 1 (robot didn't travel the route) -
    # efficiency is only meaningful once the run covers a real distance.
    if r['path_ratio'] < 0.9:
        return 0.0
    return clip01(1 - (r['path_ratio'] - 1.0) / 2.0)


def s_completion(r):
    base = 1.0 if r['finished'] else 0.5 * r['route_covered']
    penalty = clip01(1 - r['loop_residual_m'] / 30.0) if r['finished'] \
              else r['route_covered']
    return base * penalty


def s_speed(r):
    # Only earned after actually covering the route.
    if r['path_ratio'] < 0.9 or r['time_ratio'] == float('inf'):
        return 0.0
    return clip01(1 - (r['time_ratio'] - 1.0) / 10.0)


def s_smoothness(r):
    if math.isnan(r['idle_ratio']): return 0.5
    return clip01(1 - r['idle_ratio'] * 2)


WEIGHTS = {
    'outdoor_ugv': dict(safety=0.30, precision=0.20, efficiency=0.15,
                        completion=0.15, speed=0.10, smoothness=0.10),
    'research':    dict(safety=0.20, precision=0.35, efficiency=0.10,
                        completion=0.20, speed=0.08, smoothness=0.07),
    'production':  dict(safety=0.45, precision=0.10, efficiency=0.05,
                        completion=0.20, speed=0.15, smoothness=0.05),
}

with open(OUT / 'composite_scores.csv', 'w') as f:
    w = csv.writer(f)
    w.writerow(['name', 'safety', 'precision', 'efficiency', 'completion',
                'speed', 'smoothness',
                'composite_outdoor', 'composite_research', 'composite_production'])
    for r in results:
        sc = dict(safety=s_safety(r), precision=s_precision(r),
                  efficiency=s_efficiency(r), completion=s_completion(r),
                  speed=s_speed(r), smoothness=s_smoothness(r))
        comps = {k: sum(sc[m] * v for m, v in wv.items())
                 for k, wv in WEIGHTS.items()}
        # Non-finishers are capped at 0.5 - half credit for partial work.
        if not r['finished']:
            comps = {k: v * 0.5 for k, v in comps.items()}
        w.writerow([r['name']] + [f"{sc[k]:.3f}" for k in ['safety','precision','efficiency','completion','speed','smoothness']]
                   + [f"{comps[k]:.3f}" for k in ['outdoor_ugv','research','production']])
        r.update(sc)
        r.update({f'composite_{k}': v for k, v in comps.items()})


# --- normalized metrics for display ---
def norm01(values, higher_better=True):
    xs = [v for v in values if not math.isnan(v) and v != float('inf')]
    if not xs: return [0.0] * len(values)
    lo, hi = min(xs), max(xs)
    if hi == lo: return [1.0] * len(values)
    out = []
    for v in values:
        if math.isnan(v) or v == float('inf'): out.append(0.0)
        else:
            n = (v - lo) / (hi - lo)
            out.append(n if higher_better else 1 - n)
    return out

raw_cols_for_norm = [
    ('path_ratio', False), ('time_ratio', False), ('loop_residual_m', False),
    ('idle_ratio', False), ('clear_p50', True), ('clear_p05', True),
    ('near_miss_count', False), ('contact_count', False),
    ('track_p50', False), ('track_p95', False), ('arrival_p50', False),
    ('arrival_p95', False), ('anchor_useful_rate', True),
]
with open(OUT / 'metrics_normalized.csv', 'w') as f:
    w = csv.writer(f)
    w.writerow(['name'] + [c for c, _ in raw_cols_for_norm])
    for i, r in enumerate(results):
        row = [r['name']]
        for col, higher in raw_cols_for_norm:
            vs = [rr[col] for rr in results]
            nn = norm01(vs, higher)[i]
            row.append(f"{nn:.3f}")
        w.writerow(row)

# Print summary
print(f"Ran analysis over {len(results)} experiments -> {OUT}")
for r in sorted(results, key=lambda x: -x['composite_outdoor_ugv']):
    print(f"  {r['name']:<10}  outdoor={r['composite_outdoor_ugv']:.3f}  "
          f"research={r['composite_research']:.3f}  "
          f"production={r['composite_production']:.3f}")
