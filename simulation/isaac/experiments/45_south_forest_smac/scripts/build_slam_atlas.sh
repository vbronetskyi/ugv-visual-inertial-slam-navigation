#!/bin/bash
# Exp 45: Build ORB-SLAM3 atlas from the latest GT-fixed recording.
# Steps:
#   1. Use latest isaac_slam_* bag
#   2. Build TUM dataset (symlinks rgb/depth + assoc + groundtruth.txt)
#   3. Run ORB-SLAM3 rgbd_tum to create keyframe trajectory + atlas
#   4. Report ATE + copy outputs to exp 45 logs
set -euo pipefail

BAGS=/root/bags/husky_real
SLAM=/workspace/third_party/ORB_SLAM3
CONFIG=$BAGS/rgbd_d435i_v2.yaml
EXP=/workspace/simulation/isaac/experiments/45_south_forest_smac

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

REC=$(ls -td $BAGS/isaac_slam_* | head -1)
TUM=$BAGS/tum_south_exp45
echo "Source bag: $REC"
echo "TUM target: $TUM"
rm -rf $TUM; mkdir -p $TUM/rgb $TUM/depth

python3 << PYEOF
import os, hashlib, math
import numpy as np

SRC = "$REC"
OUT = "$TUM"

gt = {}
for l in open(f'{SRC}/groundtruth.csv').readlines()[1:]:
    parts = l.strip().split(',')
    t = round(float(parts[0]), 4)
    gt[t] = (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))

spawn_x, spawn_y = -95.0, -6.0
move_start = None
for t in sorted(gt.keys()):
    x, y, z, yaw = gt[t]
    if math.hypot(x - spawn_x, y - spawn_y) > 0.3:
        move_start = t; break
keep_from = (move_start - 3.0) if move_start else min(gt.keys())

src_rgb = sorted(os.listdir(f'{SRC}/camera_rgb'), key=lambda f: float(f.replace('.jpg','')))
src_dep = sorted(os.listdir(f'{SRC}/camera_depth'), key=lambda f: float(f.replace('.png','')))
rgb_by_ts = {round(float(f.replace('.jpg','')),4): f for f in src_rgb}
dep_by_ts = {round(float(f.replace('.png','')),4): f for f in src_dep}
gt_keys = np.array(sorted(gt.keys()))

CAM_FWD, CAM_UP = 0.5, 0.48
prev_hash = None
kept = 0; n_stale = 0
assoc, gt_out = [], []

for ts in sorted(rgb_by_ts.keys()):
    if ts < keep_from: continue
    if ts not in dep_by_ts: continue
    dep_path = f'{SRC}/camera_depth/{dep_by_ts[ts]}'
    h = hashlib.md5(open(dep_path,'rb').read()).hexdigest()
    if h == prev_hash: n_stale += 1; continue
    prev_hash = h
    idx = np.argmin(np.abs(gt_keys - ts))
    if abs(gt_keys[idx] - ts) > 0.1: continue
    x, y, z, yaw = gt[gt_keys[idx]]
    cx = x + CAM_FWD * math.cos(yaw)
    cy = y + CAM_FWD * math.sin(yaw)
    cz = z + CAM_UP
    qw, qz_ = math.cos(yaw/2), math.sin(yaw/2)
    ts_str = f"{ts:.4f}"
    os.symlink(os.path.abspath(f'{SRC}/camera_rgb/{rgb_by_ts[ts]}'), f'{OUT}/rgb/{ts_str}.jpg')
    os.symlink(os.path.abspath(dep_path), f'{OUT}/depth/{ts_str}.png')
    assoc.append(f"{ts_str} rgb/{ts_str}.jpg {ts_str} depth/{ts_str}.png\n")
    gt_out.append(f"{ts_str} {cx:.6f} {cy:.6f} {cz:.6f} 0.000000 0.000000 {qz_:.6f} {qw:.6f}\n")
    kept += 1

open(f'{OUT}/associations.txt','w').writelines(assoc)
open(f'{OUT}/groundtruth.txt','w').writelines(gt_out)
ts_list = [float(l.split()[0]) for l in assoc]
xs = [float(l.split()[1]) for l in gt_out]
ys = [float(l.split()[2]) for l in gt_out]
dist = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]) for i in range(1,len(xs)))
print(f"TUM built: {kept} frames, stale={n_stale}, dist={dist:.0f}m, duration={ts_list[-1]-ts_list[0]:.0f}s")
PYEOF

echo "=== Running ORB-SLAM3 rgbd_tum on $TUM ==="
cd $SLAM
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt $CONFIG $TUM $TUM/associations.txt \
    2>&1 | tee /tmp/slam_exp45.log

cp CameraTrajectory.txt $TUM/CameraTrajectory.txt
cp KeyFrameTrajectory.txt $TUM/KeyFrameTrajectory.txt
cp $TUM/CameraTrajectory.txt $EXP/logs/exp45_slam_camera_traj.txt
cp $TUM/KeyFrameTrajectory.txt $EXP/logs/exp45_slam_keyframe_traj.txt

echo "=== Evaluating ==="
python3 << PYEOF
import numpy as np, math
slam = np.array([[float(p) for p in l.split()] for l in open('$TUM/CameraTrajectory.txt')])
gt = np.array([[float(p) for p in l.split()] for l in open('$TUM/groundtruth.txt')])
gt_dict = {round(r[0],4): r[1:4] for r in gt}
ms, mg = [], []
for s in slam:
    ts = round(s[0],4)
    if ts in gt_dict: ms.append(s[1:4]); mg.append(gt_dict[ts])
ms, mg = np.array(ms), np.array(mg)
if len(ms) < 10:
    print("Too few matched - SLAM probably failed"); import sys; sys.exit(1)
sd = np.sum(np.sqrt(np.sum(np.diff(ms,axis=0)**2,axis=1)))
gd = np.sum(np.sqrt(np.sum(np.diff(mg,axis=0)**2,axis=1)))
mu_m, mu_d = ms.mean(0), mg.mean(0)
mz, dz = ms-mu_m, mg-mu_d
n = len(ms); H = mz.T@dz/n; U,D,Vt = np.linalg.svd(H)
S = np.eye(3)
if np.linalg.det(U)*np.linalg.det(Vt)<0: S[2,2]=-1
R = Vt.T@S@U.T; c = np.trace(np.diag(D)@S)*n/np.sum(mz**2)
t = mu_d - c*R@mu_m
aligned = c*(ms@R.T)+t
errors = np.sqrt(np.sum((aligned-mg)**2, axis=1))
tracked = len(slam); total = len(gt)
resets = sum(1 for l in open('/tmp/slam_exp45.log') if 'Creation of new map' in l) - 2
print(f"")
print(f"  Tracked: {tracked}/{total} ({tracked/total*100:.0f}%)")
print(f"  Map resets: {max(0, resets)}")
print(f"  Travel: SLAM={sd:.1f}m, GT={gd:.1f}m, ratio={sd/gd:.4f}")
print(f"  Scale:  {c:.4f}")
print(f"  ATE RMSE: {np.sqrt(np.mean(errors**2)):.3f}m ({np.sqrt(np.mean(errors**2))/gd*100:.2f}%)")
print(f"  ATE Max:  {errors.max():.3f}m")
PYEOF
