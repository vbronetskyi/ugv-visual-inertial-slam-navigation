#!/usr/bin/env python3
"""Exp 49 analysis: compute ATE for VIO trajectory with Umeyama alignment.

Writes GT and VIO CSVs for plot_trajectory_map (body-frame, matched by timestamp).
"""
import numpy as np, math, csv, sys, os

EXP = "/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip"

REC = sys.argv[1] if len(sys.argv) > 1 else None
if REC is None:
    # Try to read from run_info.txt
    info = open(f"{EXP}/logs/run_info.txt").read()
    REC = info.split("REC=")[1].strip().split()[0]

VIO = f"{EXP}/logs/vio_final_camera.txt"

gt_data = {}
for line in open(f'{REC}/groundtruth.csv').readlines()[1:]:
    p = line.strip().split(',')
    gt_data[round(float(p[0]), 4)] = (float(p[1]), float(p[2]), float(p[3]), float(p[4]))
gt_times = np.array(sorted(gt_data.keys()))

# GT CSV
with open(f'{EXP}/logs/exp49_gt_traj.csv', 'w') as f:
    w = csv.writer(f); w.writerow(['ts', 'gt_x', 'gt_y'])
    for t_key in sorted(gt_data.keys()):
        gx, gy, _, _ = gt_data[t_key]
        w.writerow([t_key, gx, gy])

# VIO analysis
vio = np.array([[float(x) for x in l.split()] for l in open(VIO)])
mv, mg, tss = [], [], []
for r in vio:
    ts = round(r[0], 4)
    idx = np.argmin(np.abs(gt_times - ts))
    if abs(gt_times[idx] - ts) < 0.15:
        gx, gy, gz, gyaw = gt_data[gt_times[idx]]
        mv.append(r[1:4])
        mg.append([gx + 0.5*math.cos(gyaw), gy + 0.5*math.sin(gyaw), gz + 0.48])
        tss.append(r[0])
mv, mg = np.array(mv), np.array(mg)
order = np.argsort(tss); mv = mv[order]; mg = mg[order]
ts_arr = np.array(tss)[order]
n = len(mv)

mu_v, mu_g = mv.mean(0), mg.mean(0)
vz, gz = mv - mu_v, mg - mu_g
H = vz.T @ gz / n
U, D, Vt = np.linalg.svd(H)
S = np.eye(3)
if np.linalg.det(U) * np.linalg.det(Vt) < 0: S[2, 2] = -1
R = Vt.T @ S @ U.T
c = np.trace(np.diag(D) @ S) * n / np.sum(vz**2)
t = mu_g - c * R @ mu_v
al = c * (mv @ R.T) + t

err = np.sqrt(np.sum((al - mg)**2, axis=1))
gt_dist = np.sum(np.sqrt(np.sum(np.diff(mg, axis=0)**2, axis=1)))
vio_dist = np.sum(np.sqrt(np.sum(np.diff(al, axis=0)**2, axis=1)))

print(f"Tracked: {len(vio)}/{len(gt_data)} ({len(vio)/len(gt_data)*100:.0f}%)")
print(f"GT dist: {gt_dist:.1f}m, VIO dist: {vio_dist:.1f}m, ratio: {vio_dist/gt_dist:.3f}")
print(f"Scale: {c:.3f}")
print(f"ATE RMSE: {np.sqrt(np.mean(err**2)):.3f}m, max: {err.max():.3f}m, median: {np.median(err):.3f}m")

# Segment analysis (5 segments)
n_segs = 5; seg = n // n_segs
print("\nSegment analysis:")
for i in range(n_segs):
    s_i = i * seg
    e_i = (i + 1) * seg if i < n_segs - 1 else n
    se = err[s_i:e_i]; sg = mg[s_i:e_i]
    print(f"  {i*20}-{(i+1)*20}%: ATE={np.sqrt(np.mean(se**2)):.3f}m max={se.max():.3f}m x=[{sg[0,0]:.0f}..{sg[-1,0]:.0f}]")

# VIO CSV (body-frame)
with open(f'{EXP}/logs/exp49_vio_traj.csv', 'w') as f:
    w = csv.writer(f); w.writerow(['ts', 'gt_x', 'gt_y'])
    for i in range(n):
        idx = np.argmin(np.abs(gt_times - ts_arr[i]))
        _, _, _, gyaw = gt_data[gt_times[idx]]
        w.writerow([ts_arr[i], al[i, 0] - 0.5*math.cos(gyaw), al[i, 1] - 0.5*math.sin(gyaw)])
print(f"\nCSVs written to {EXP}/logs/")
