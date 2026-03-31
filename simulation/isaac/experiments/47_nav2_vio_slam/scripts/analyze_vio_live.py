#!/usr/bin/env python3
"""Analyze live VIO accuracy during Nav2 navigation.
Compares VIO SLAM trajectory with GT, computes ATE at matched timestamps."""
import numpy as np
import math, sys, os

REC_DIR = sys.argv[1] if len(sys.argv) > 1 else max(
    [f"/root/bags/husky_real/{d}" for d in os.listdir("/root/bags/husky_real")
     if d.startswith("isaac_slam_")], key=os.path.getmtime)
VIO_TRAJ = sys.argv[2] if len(sys.argv) > 2 else "/workspace/third_party/ORB_SLAM3/CameraTrajectory.txt"

print(f"REC: {REC_DIR}")
print(f"VIO: {VIO_TRAJ}")

# Load GT
gt_file = f"{REC_DIR}/groundtruth.csv"
gt_data = {}
for line in open(gt_file).readlines()[1:]:
    parts = line.strip().split(',')
    t = round(float(parts[0]), 4)
    gt_data[t] = (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))

# Load VIO trajectory (TUM format: t tx ty tz qx qy qz qw)
if not os.path.exists(VIO_TRAJ) or os.path.getsize(VIO_TRAJ) == 0:
    print("No VIO trajectory file - SLAM may not have saved yet")
    sys.exit(1)

vio = []
for line in open(VIO_TRAJ):
    if line.startswith('#'):
        continue
    parts = line.strip().split()
    if len(parts) >= 4:
        vio.append([float(x) for x in parts])
vio = np.array(vio)
print(f"VIO: {len(vio)} poses, GT: {len(gt_data)} poses")

# Match timestamps
gt_times = np.array(sorted(gt_data.keys()))
matched_vio = []
matched_gt = []
for row in vio:
    ts = round(row[0], 4)
    idx = np.argmin(np.abs(gt_times - ts))
    if abs(gt_times[idx] - ts) < 0.15:
        gt_t = gt_times[idx]
        gx, gy, gz, gyaw = gt_data[gt_t]
        # GT camera position (same as TUM building in run_vio.sh)
        cx = gx + 0.5 * math.cos(gyaw)
        cy = gy + 0.5 * math.sin(gyaw)
        cz = gz + 0.48
        matched_vio.append(row[1:4])
        matched_gt.append([cx, cy, cz])

matched_vio = np.array(matched_vio)
matched_gt = np.array(matched_gt)
print(f"Matched: {len(matched_vio)} poses")

if len(matched_vio) < 10:
    print("Too few matched poses")
    sys.exit(1)

# Umeyama alignment (Sim3)
mu_v, mu_g = matched_vio.mean(0), matched_gt.mean(0)
vz, gz_ = matched_vio - mu_v, matched_gt - mu_g
n = len(matched_vio)
H = vz.T @ gz_ / n
U, D, Vt = np.linalg.svd(H)
S = np.eye(3)
if np.linalg.det(U) * np.linalg.det(Vt) < 0:
    S[2, 2] = -1
R = Vt.T @ S @ U.T
c = np.trace(np.diag(D) @ S) * n / np.sum(vz ** 2)
t = mu_g - c * R @ mu_v

# Apply alignment
aligned = c * (matched_vio @ R.T) + t
errors = np.sqrt(np.sum((aligned - matched_gt) ** 2, axis=1))

# Path lengths
vio_dist = np.sum(np.sqrt(np.sum(np.diff(matched_vio, axis=0)**2, axis=1)))
gt_dist = np.sum(np.sqrt(np.sum(np.diff(matched_gt, axis=0)**2, axis=1)))

# Segment analysis (every 20% of route)
n_segs = 5
seg_size = len(errors) // n_segs
print(f"\n{'='*60}")
print(f"VIO Live Accuracy (Umeyama-aligned)")
print(f"{'='*60}")
print(f"Tracked: {len(vio)}/{len(gt_data)} ({len(vio)/len(gt_data)*100:.0f}%)")
print(f"VIO dist: {vio_dist:.1f}m, GT dist: {gt_dist:.1f}m, ratio: {vio_dist/gt_dist:.3f}")
print(f"Scale: {c:.3f}")
print(f"ATE RMSE: {np.sqrt(np.mean(errors**2)):.3f}m")
print(f"ATE max: {errors.max():.3f}m")
print(f"ATE median: {np.median(errors):.3f}m")

print(f"\nSegment analysis:")
for i in range(n_segs):
    s = i * seg_size
    e = (i + 1) * seg_size if i < n_segs - 1 else len(errors)
    seg_err = errors[s:e]
    seg_gt = matched_gt[s:e]
    pct = f"{i*20}-{(i+1)*20}%"
    x_range = f"x=[{seg_gt[0,0]:.0f}..{seg_gt[-1,0]:.0f}]"
    print(f"  {pct}: ATE={np.sqrt(np.mean(seg_err**2)):.3f}m max={seg_err.max():.3f}m {x_range}")

# Track failures (lost count from SLAM log)
print(f"\nConclusion:")
if np.sqrt(np.mean(errors**2)) < 1.0:
    print(f"  VIO ATE {np.sqrt(np.mean(errors**2)):.2f}m - VIABLE for Nav2 (xy_goal_tolerance=1.5m)")
elif np.sqrt(np.mean(errors**2)) < 3.0:
    print(f"  VIO ATE {np.sqrt(np.mean(errors**2)):.2f}m - MARGINAL for Nav2")
else:
    print(f"  VIO ATE {np.sqrt(np.mean(errors**2)):.2f}m - NOT viable for Nav2")
