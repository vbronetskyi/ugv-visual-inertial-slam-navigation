#!/usr/bin/env python3
"""Exp 48 analysis: compute ATE for VIO and RGB-D-only, write CSVs for plots."""
import numpy as np, math, csv, sys

REC = sys.argv[1] if len(sys.argv) > 1 else "/root/bags/husky_real/isaac_slam_1776423021"
EXP = "/workspace/simulation/isaac/experiments/48_vio_roundtrip"
VIO = f"{EXP}/logs/vio_final_camera.txt"
RGBD = f"{EXP}/logs/rgbd_only_camera.txt"


def analyze(traj_file, label):
    gt_data = {}
    for line in open(f'{REC}/groundtruth.csv').readlines()[1:]:
        p = line.strip().split(',')
        t = round(float(p[0]), 4)
        gt_data[t] = (float(p[1]), float(p[2]), float(p[3]), float(p[4]))
    gt_times = np.array(sorted(gt_data.keys()))

    traj = np.array([[float(x) for x in l.split()] for l in open(traj_file)])
    mv, mg, ts_list = [], [], []
    for r in traj:
        ts = round(r[0], 4)
        idx = np.argmin(np.abs(gt_times - ts))
        if abs(gt_times[idx] - ts) < 0.15:
            gx, gy, gz, gyaw = gt_data[gt_times[idx]]
            mv.append(r[1:4])
            mg.append([gx + 0.5*math.cos(gyaw), gy + 0.5*math.sin(gyaw), gz + 0.48])
            ts_list.append(r[0])
    mv = np.array(mv); mg = np.array(mg)
    order = np.argsort(ts_list)
    mv = mv[order]; mg = mg[order]; ts_arr = np.array(ts_list)[order]
    n = len(mv)

    # Umeyama SE(3) alignment
    mu_v, mu_g = mv.mean(0), mg.mean(0)
    vz, gz = mv-mu_v, mg-mu_g
    H = vz.T @ gz / n
    U, D, Vt = np.linalg.svd(H)
    S = np.eye(3)
    if np.linalg.det(U)*np.linalg.det(Vt) < 0: S[2,2] = -1
    R = Vt.T @ S @ U.T
    c = np.trace(np.diag(D) @ S) * n / np.sum(vz**2)
    t = mu_g - c*R @ mu_v
    al = c*(mv @ R.T) + t

    err = np.sqrt(np.sum((al-mg)**2, axis=1))
    gt_dist = np.sum(np.sqrt(np.sum(np.diff(mg, axis=0)**2, axis=1)))
    traj_dist = np.sum(np.sqrt(np.sum(np.diff(al, axis=0)**2, axis=1)))
    print(f"\n{label}:")
    print(f"  Matched: {n} poses")
    print(f"  GT dist: {gt_dist:.0f}m, traj: {traj_dist:.0f}m, ratio: {traj_dist/gt_dist:.3f}")
    print(f"  Scale: {c:.3f}, ATE RMSE: {np.sqrt(np.mean(err**2)):.3f}m, max: {err.max():.3f}m")
    return al, mg, ts_arr, err


# Write GT CSV
gt_data = {}
for line in open(f'{REC}/groundtruth.csv').readlines()[1:]:
    p = line.strip().split(',')
    gt_data[round(float(p[0]), 4)] = (float(p[1]), float(p[2]), float(p[3]), float(p[4]))
with open(f'{EXP}/logs/exp48_gt_traj.csv', 'w') as f:
    w = csv.writer(f); w.writerow(['ts','gt_x','gt_y'])
    for t_key in sorted(gt_data.keys()):
        gx, gy, _, _ = gt_data[t_key]
        w.writerow([t_key, gx, gy])

# VIO analysis + CSV
vio_al, vio_gt, vio_ts, vio_err = analyze(VIO, "VIO (RGB-D-Inertial)")
gt_times = np.array(sorted(gt_data.keys()))
with open(f'{EXP}/logs/exp48_vio_traj.csv', 'w') as f:
    w = csv.writer(f); w.writerow(['ts','gt_x','gt_y'])
    for i in range(len(vio_al)):
        idx = np.argmin(np.abs(gt_times - vio_ts[i]))
        _,_,_,gyaw = gt_data[gt_times[idx]]
        w.writerow([vio_ts[i], vio_al[i,0]-0.5*math.cos(gyaw), vio_al[i,1]-0.5*math.sin(gyaw)])

# RGB-D analysis + CSV
rgbd_al, rgbd_gt, rgbd_ts, rgbd_err = analyze(RGBD, "RGB-D only (no IMU)")
with open(f'{EXP}/logs/exp48_rgbd_traj.csv', 'w') as f:
    w = csv.writer(f); w.writerow(['ts','gt_x','gt_y'])
    for i in range(len(rgbd_al)):
        idx = np.argmin(np.abs(gt_times - rgbd_ts[i]))
        _,_,_,gyaw = gt_data[gt_times[idx]]
        w.writerow([rgbd_ts[i], rgbd_al[i,0]-0.5*math.cos(gyaw), rgbd_al[i,1]-0.5*math.sin(gyaw)])

print("\nCSVs written to logs/")
