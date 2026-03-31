#!/bin/bash
# Exp 45: Build VIO atlas (RGB-D-Inertial) from latest GT-fixed recording.
set -euo pipefail

BAGS=/root/bags/husky_real
SLAM=/workspace/third_party/ORB_SLAM3
EXP=/workspace/simulation/isaac/experiments/45_south_forest_smac
TH=${1:-40}  # ThDepth sweep from CLI

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

REC=$(ls -td $BAGS/isaac_slam_* | head -1)
TUM=$BAGS/tum_south_exp45_vio                      # separate TUM with aggressive trim for VIO
CFG=$EXP/config/vio_th${TH}.yaml
LOG=/tmp/vio_exp45_th${TH}.log
mkdir -p $EXP/config

echo "Source bag:  $REC"
echo "TUM dir:     $TUM (aggressive trim: move_start + 2s)"
echo "ThDepth:     $TH"

# Rebuild TUM with move_start + 2.0 to skip static period and early accel ramp
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
keep_from = (move_start + 2.0) if move_start else min(gt.keys())
print(f"move_start={move_start}, keep_from={keep_from} (aggressive trim)")

src_rgb = sorted(os.listdir(f'{SRC}/camera_rgb'), key=lambda f: float(f.replace('.jpg','')))
rgb_by_ts = {round(float(f.replace('.jpg','')),4): f for f in src_rgb}
src_dep = sorted(os.listdir(f'{SRC}/camera_depth'), key=lambda f: float(f.replace('.png','')))
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
print(f"TUM (VIO): {kept} frames after aggressive trim, stale={n_stale}")
PYEOF

# Convert IMU (Isaac ax,ay,az,gx,gy,gz,...) -> ORB-SLAM3 TUM (ts wx wy wz ax ay az)
python3 << PYEOF
import os
with open('$REC/imu.csv') as f:
    lines = f.readlines()
out = []
for l in lines[1:]:
    p = l.strip().split(',')
    ts = float(p[0])
    ax, ay, az = float(p[1]), float(p[2]), float(p[3])
    gx, gy, gz = float(p[4]), float(p[5]), float(p[6])
    out.append(f"{ts:.6f} {gx:.6f} {gy:.6f} {gz:.6f} {ax:.6f} {ay:.6f} {az:.6f}\n")
open('$REC/imu_orbslam.txt','w').writelines(out)
print(f"IMU: {len(out)} samples at {1.0/(float(out[-1].split()[0])-float(out[0].split()[0]))*(len(out)-1):.0f} Hz")
PYEOF

# Write VIO config with current ThDepth
cat > $CFG << EOF
%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: 617.201
Camera1.fy: 617.362
Camera1.cx: 324.637
Camera1.cy: 242.462
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 10
Camera.RGB: 1

Stereo.ThDepth: ${TH}.0
Stereo.b: 0.0745

RGBD.DepthMapFactor: 1000.0

# Correct T_b_c1: body FLU (X fwd, Y left, Z up) <- cam (X right, Y down, Z fwd)
# body_X = cam_Z, body_Y = -cam_X, body_Z = -cam_Y. Camera at (0.5m fwd, 0.48m up).
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0, 0.0, 1.0, 0.5,
          -1.0, 0.0, 0.0, 0.0,
           0.0,-1.0, 0.0, 0.48,
           0.0, 0.0, 0.0, 1.0 ]

IMU.InsertKFsWhenLost: 0
IMU.NoiseGyro: 1.0e-2
IMU.NoiseAcc: 1.0e-1
IMU.GyroWalk: 1.0e-6
IMU.AccWalk: 1.0e-4
IMU.Frequency: 200.0

ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
EOF

echo "=== Running VIO (ThDepth=${TH}) ==="
cd $SLAM
./Examples/RGB-D-Inertial/rgbd_inertial_offline \
    Vocabulary/ORBvoc.txt $CFG $TUM $TUM/associations.txt $REC/imu_orbslam.txt \
    2>&1 | tee $LOG

# Save traj
cp CameraTrajectory.txt $EXP/logs/exp45_vio_th${TH}_camera.txt 2>/dev/null || true
cp KeyFrameTrajectory.txt $EXP/logs/exp45_vio_th${TH}_kf.txt 2>/dev/null || true

# Evaluate
echo "=== Evaluating VIO (ThDepth=${TH}) ==="
python3 << PYEOF
import numpy as np, math, os
camfile = '$EXP/logs/exp45_vio_th${TH}_camera.txt'
if not os.path.exists(camfile) or os.path.getsize(camfile) == 0:
    print("NO TRAJECTORY - VIO crashed or did not track"); exit(0)
slam = np.array([[float(p) for p in l.split()] for l in open(camfile)])
gt = np.array([[float(p) for p in l.split()] for l in open('$TUM/groundtruth.txt')])
if len(slam) < 10:
    print(f"Too few poses: {len(slam)}"); exit(0)
gt_dict = {round(r[0],4): r[1:4] for r in gt}
ms, mg = [], []
for s in slam:
    ts = round(s[0],4)
    if ts in gt_dict: ms.append(s[1:4]); mg.append(gt_dict[ts])
ms, mg = np.array(ms), np.array(mg)
if len(ms) < 10: print("Too few matched"); exit(0)
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
resets = sum(1 for l in open('$LOG') if 'Creation of new map' in l) - 2
print(f"ThDepth=${TH}: tracked={len(slam)}/{len(gt)} ({len(slam)/len(gt)*100:.0f}%) resets={max(0,resets)} "
      f"SLAM={sd:.1f}m GT={gd:.1f}m ratio={sd/gd:.4f} scale={c:.4f} "
      f"ATE_rmse={np.sqrt(np.mean(errors**2)):.3f}m ATE_max={errors.max():.3f}m")
PYEOF
