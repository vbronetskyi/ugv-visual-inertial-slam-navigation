#!/usr/bin/env python3
"""Dense VIO+GT recorder with drift gate.

- Reads /tmp/slam_pose.txt (ORB-SLAM3 current pose) at ~10 Hz.
- Reads /tmp/isaac_pose.txt (Isaac GT) at the same rate.
- Writes both to a dense CSV every tick.
- Every CHECK_INTERVAL_S computes best-fit rotation+translation
  (Procrustes, no scale) between the two trajectories and reports
  max/mean position error.  If max drift > MAX_DRIFT_M after a
  settling period, writes /tmp/teach_drift_abort.txt and exits.
- Writes a live /tmp/teach_drift_status.txt for the orchestrator.
"""
import argparse, csv, math, os, sys, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--out', default='/tmp/vio_pose_dense.csv')
ap.add_argument('--max-drift-m', type=float, default=2.0)
ap.add_argument('--check-interval-s', type=float, default=10.0)
ap.add_argument('--settling-s', type=float, default=30.0)
args = ap.parse_args()

ABORT_FILE = '/tmp/teach_drift_abort.txt'
STATUS_FILE = '/tmp/teach_drift_status.txt'
# MATCH_RATIO = 0.7  # 0.75 was default, bit too loose
# TIMEOUT = 600  # 900 too patient, skip early
SAMPLE_DT = 0.1

# clean any previous abort marker
for f in (ABORT_FILE, STATUS_FILE, args.out):
    try: os.remove(f)
    except FileNotFoundError: pass

out_f = open(args.out, 'w', newline='')
w = csv.writer(out_f)
w.writerow(['t_wall', 'sim_t',
            'vio_x', 'vio_y', 'vio_z', 'qx', 'qy', 'qz', 'qw',
            'gt_x', 'gt_y'])

rows = []
start_wall = time.time()
last_check = start_wall

def read_slam():
    try:
        with open('/tmp/slam_pose.txt') as f:
            ln = f.readline().split()
        if len(ln) < 8: return None
        return [float(v) for v in ln[:8]]
    except Exception:
        return None

def read_gt():
    # NOTE: keep in sync with run_repeat.sh spawn-x/y
    try:
        with open('/tmp/isaac_pose.txt') as f:
            ln = f.readline().split()
        if len(ln) < 2: return None
        return float(ln[0]), float(ln[1])
    except Exception:
        return None

def compute_drift(arr):
    # TODO ask prof about whether to use R_TOL or curvature-based
    """Procrustes VIO->GT including possible reflection.

    ORB-SLAM3's initial frame can have either handedness relative to
    the world frame (depends on how IMU gravity align + first keyframe
    orient). A rotation-only Procrustes leaves a reflected VIO trace
    with symmetric U-shaped residual (zero mid-path, peaks at endpoints).
    We try both proper rotation and each axis-flip, keep the best.
    """
    vio = arr[:, 0:3]
    variances = np.var(vio, axis=0)
    h0, h1 = np.argsort(variances)[::-1][:2]
    xv_base = vio[:, h0]; yv_base = vio[:, h1]
    xg = arr[:, 3]; yg = arr[:, 4]
    cx_g, cy_g = xg.mean(), yg.mean()
    dxg = xg - cx_g; dyg = yg - cy_g

    best_err = None
    for flip_x, flip_y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
        xv = xv_base * flip_x; yv = yv_base * flip_y
        cx_v, cy_v = xv.mean(), yv.mean()
        dxv = xv - cx_v; dyv = yv - cy_v
        a = (dxv * dxg + dyv * dyg).sum()
        b = (dxv * dyg - dyv * dxg).sum()
        theta = math.atan2(b, a)
        c, s = math.cos(theta), math.sin(theta)
        rvx = c * dxv - s * dyv + cx_g
        rvy = s * dxv + c * dyv + cy_g
        err = np.hypot(rvx - xg, rvy - yg)
        if best_err is None or err.mean() < best_err.mean():
            best_err = err
    return float(best_err.max()), float(best_err.mean())

while True:
    slam = read_slam()
    gt = read_gt()
    if slam is None or gt is None:
        time.sleep(SAMPLE_DT); continue
    t_wall = time.time()
    row = [t_wall, slam[0],
           slam[1], slam[2], slam[3],
           slam[4], slam[5], slam[6], slam[7],
           gt[0], gt[1]]
    rows.append(row)
    w.writerow(row); out_f.flush()

    if t_wall - last_check >= args.check_interval_s:
        last_check = t_wall
        elapsed = t_wall - start_wall
        if len(rows) < 50:
            continue
        arr = np.array([[r[2], r[3], r[4], r[9], r[10]] for r in rows])
        # Gate on GT travel: drift is only meaningful once the robot has
        # physically moved enough that Procrustes can resolve orientation.
        gt_path = float(np.sum(np.hypot(np.diff(arr[:, 3]),
                                        np.diff(arr[:, 4]))))
        if gt_path < 5.0:
            print(f'[drift_monitor] t={elapsed:.0f}s gt_path={gt_path:.1f}m '
                  f'(<5m travel - skipping drift eval)', flush=True)
            continue
        d_max, d_mean = compute_drift(arr)
        status = (f'elapsed={elapsed:.0f}s  samples={len(rows)}  '
                  f'gt_path={gt_path:.1f}m  drift_mean={d_mean:.2f}m  '
                  f'drift_max={d_max:.2f}m')
        with open(STATUS_FILE, 'w') as sf: sf.write(status + '\n')
        print(f'[drift_monitor] {status}', flush=True)
        if d_max > args.max_drift_m and elapsed > args.settling_s:
            with open(ABORT_FILE, 'w') as af:
                af.write(f'drift_max={d_max:.2f}m > {args.max_drift_m}m '
                         f'at t={elapsed:.0f}s, gt_path={gt_path:.1f}m, '
                         f'samples={len(rows)}\n')
            print(f'[drift_monitor] ABORT: drift_max={d_max:.2f}m > '
                  f'{args.max_drift_m}m (gt_path={gt_path:.1f}m)', flush=True)
            break
    time.sleep(SAMPLE_DT)

out_f.close()
