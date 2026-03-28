#!/usr/bin/env python3
"""Compute fair comparison metrics for exp 41/42/43."""
import csv
import re
import math
import numpy as np


def load_traj(path, x='gt_x', y='gt_y'):
    """Try multiple column names."""
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            for xc in [x, 'x']:
                if xc in row:
                    rows.append((float(row[xc]), float(row[y if y in row else 'y'])))
                    break
    return np.array(rows)


def path_length(traj):
    """Total path length."""
    if len(traj) < 2:
        return 0.0
    diffs = np.diff(traj, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def cross_track_error(traj, reference):
    """Mean perpendicular distance from each traj point to nearest reference segment."""
    errors = []
    ref_x = reference[:, 0]
    ref_y = reference[:, 1]
    for px, py in traj:
        # Distance to all reference points (approximation)
        d = np.sqrt((ref_x - px)**2 + (ref_y - py)**2)
        errors.append(d.min())
    return float(np.mean(errors)), float(np.max(errors))


def parse_localization_errors(tf_log, max_dist=200):
    """Parse err=Xm from tf_relay log, only while dist<200m (filter post-run noise)."""
    errs = []
    with open(tf_log) as f:
        for line in f:
            m_e = re.search(r'err=([\d.]+)m', line)
            m_d = re.search(r'dist=(\d+)m', line)
            if m_e and m_d:
                d = int(m_d.group(1))
                if d <= max_dist:
                    errs.append(float(m_e.group(1)))
    return np.array(errs) if errs else np.array([0.0])


def parse_slam_coverage(tf_log):
    """Count [SLAM] vs [ENC] modes."""
    slam = enc = 0
    with open(tf_log) as f:
        for line in f:
            if '[SLAM]' in line:
                slam += 1
            elif '[ENC]' in line:
                enc += 1
    total = slam + enc
    return (100 * slam / total) if total > 0 else 0


# Reference path
REF_PATH = "/workspace/simulation/isaac/experiments/35_road_obstacle_avoidance/logs/road_noobs_baseline.csv"
ref = load_traj(REF_PATH, x='gt_x', y='gt_y')

# Three obstacle runs
runs = {
    'GT (exp 41)': {
        'traj': "/workspace/simulation/isaac/experiments/41_trajectory_follow/logs/exp41_roundtrip_outbound_traj.csv",
        'tf_log': None,  # GT mode logs no error
        'reached': 41, 'total': 43, 'duration': 491,
    },
    'Encoder+Compass (exp 42)': {
        'traj': "/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_obs_traj.csv",
        'tf_log': "/workspace/simulation/isaac/experiments/42_real_localization/logs/exp42_obs_tf.log",
        'reached': 41, 'total': 43, 'duration': 461,
    },
    'SLAM+Encoder (exp 43)': {
        'traj': "/workspace/simulation/isaac/experiments/43_slam_localization/logs/exp43_final_traj.csv",
        'tf_log': "/workspace/simulation/isaac/experiments/43_slam_localization/logs/exp43_final_tf.log",
        'reached': 41, 'total': 43, 'duration': 460,
    },
}

print(f"{'Method':<28} {'Reached':>8} {'Path m':>8} {'CTE m':>9} {'CTEmax':>7} {'Loc err':>9} {'Loc max':>8} {'Dur s':>6}")

for name, r in runs.items():
    traj = load_traj(r['traj'], x='gt_x', y='gt_y')
    if len(traj) == 0:
        traj = load_traj(r['traj'], x='x', y='y')
    plen = path_length(traj)
    cte_mean, cte_max = cross_track_error(traj, ref)
    if r['tf_log']:
        loc = parse_localization_errors(r['tf_log'])
        loc_mean, loc_max = float(loc.mean()), float(loc.max())
    else:
        loc_mean = loc_max = 0.0  # GT mode = perfect localization
    pct = f"{100*r['reached']/r['total']:.0f}%"
    print(f"{name:<28} {r['reached']}/{r['total']:<3} ({pct})  {plen:>6.1f}  "
          f"{cte_mean:>7.2f}  {cte_max:>6.2f}  {loc_mean:>7.2f}  {loc_max:>7.2f}  {r['duration']:>5}")

print()
slam_pct = parse_slam_coverage(runs['SLAM+Encoder (exp 43)']['tf_log'])
print(f"SLAM tracking coverage (exp 43): {slam_pct:.0f}% of ticks")
print(f"Reference path length: {path_length(ref):.1f} m")
