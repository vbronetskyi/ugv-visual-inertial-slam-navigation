#!/bin/bash
# Exp 46: Fix VIO for south forest - use EXACT exp 23 calibration + enhanced ORB.
set -euo pipefail

BAGS=/root/bags/husky_real
SLAM=/workspace/third_party/ORB_SLAM3
EXP=/workspace/simulation/isaac/experiments/46_south_vio
REC=/root/bags/husky_real/isaac_slam_1776319538            # GT-fixed recording from exp 45
TH=${1:-160}
CFG=$EXP/config/vio_th${TH}.yaml
LOG=/tmp/vio_exp46_th${TH}.log

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib

# Build TUM with aggressive trim (move_start + 1s)
TUM=$BAGS/tum_south_exp46
rm -rf $TUM; mkdir -p $TUM/rgb $TUM/depth
python3 << PYEOF
import os, hashlib, math, numpy as np
SRC="$REC"; OUT="$TUM"
gt={}
for l in open(f'{SRC}/groundtruth.csv').readlines()[1:]:
    p=l.strip().split(','); t=round(float(p[0]),4)
    gt[t]=(float(p[1]),float(p[2]),float(p[3]),float(p[4]))
spawn=(-95.0,-6.0)
mv=None
for t in sorted(gt):
    if math.hypot(gt[t][0]-spawn[0],gt[t][1]-spawn[1])>0.3:
        mv=t; break
keep=(mv+1.0) if mv else 0
print(f"move_start={mv}, keep_from={keep}")
rgb={round(float(f.replace('.jpg','')),4):f for f in os.listdir(f'{SRC}/camera_rgb')}
dep={round(float(f.replace('.png','')),4):f for f in os.listdir(f'{SRC}/camera_depth')}
gtk=np.array(sorted(gt))
assoc=[]; gt_out=[]; prev=None
for ts in sorted(rgb):
    if ts<keep or ts not in dep: continue
    dp=f'{SRC}/camera_depth/{dep[ts]}'
    h=hashlib.md5(open(dp,'rb').read()).hexdigest()
    if h==prev: continue
    prev=h
    idx=np.argmin(np.abs(gtk-ts))
    if abs(gtk[idx]-ts)>0.1: continue
    x,y,z,yaw=gt[gtk[idx]]
    cx=x+0.5*math.cos(yaw); cy=y+0.5*math.sin(yaw); cz=z+0.48
    qw,qz_=math.cos(yaw/2),math.sin(yaw/2)
    s=f"{ts:.4f}"
    os.symlink(os.path.abspath(f'{SRC}/camera_rgb/{rgb[ts]}'),f'{OUT}/rgb/{s}.jpg')
    os.symlink(os.path.abspath(dp),f'{OUT}/depth/{s}.png')
    assoc.append(f"{s} rgb/{s}.jpg {s} depth/{s}.png\n")
    gt_out.append(f"{s} {cx:.6f} {cy:.6f} {cz:.6f} 0 0 {qz_:.6f} {qw:.6f}\n")
open(f'{OUT}/associations.txt','w').writelines(assoc)
open(f'{OUT}/groundtruth.txt','w').writelines(gt_out)
print(f"TUM built: {len(assoc)} frames")
PYEOF

# Convert IMU -> ORB-SLAM3 format, with gyro/accel bias subtraction from stationary start
IMU_OUT=$REC/imu_exp46.txt
python3 << PYEOF
import numpy as np, math
d=np.loadtxt('$REC/imu.csv',delimiter=',',skiprows=1)
gt=np.loadtxt('$REC/groundtruth.csv',delimiter=',',skiprows=1)
# Static period: first 2 seconds (robot spawn settling)
t=d[:,0]
t0=t[0]
static_mask = t < t0+1.5
# Remove first 0.5s (still physics settling), use 0.5..1.5 as baseline
bias_mask = (t > t0+0.5) & (t < t0+1.5)
if bias_mask.sum() < 50:
    bias_mask = t < t0+1.0
bgx = d[bias_mask,4].mean()
bgy = d[bias_mask,5].mean()
bgz = d[bias_mask,6].mean()
bax = d[bias_mask,1].mean()
bay = d[bias_mask,2].mean()
# Accel: keep gravity on Z (don't subtract az bias since it's gravity)
print(f"Bias from t=[{t0+0.5:.2f},{t0+1.5:.2f}]:")
print(f"  gyro  bias (rad/s): bgx={bgx:+.4f} bgy={bgy:+.4f} bgz={bgz:+.4f}")
print(f"  accel bias (m/s^2): bax={bax:+.4f} bay={bay:+.4f} (az kept for gravity)")
# Subtract and trim to move_start + 1
mv=None
for row in gt:
    if math.hypot(row[1]+95,row[2]+6)>0.3: mv=row[0]; break
keep=mv+1.0 if mv else 0
mask = d[:,0]>=keep
dtrim = d[mask]
with open('$IMU_OUT','w') as f:
    for r in dtrim:
        t,ax,ay,az,gx,gy,gz=r[:7]
        # Subtract gyro bias (keep accel bias only for x/y, az is gravity)
        gx -= bgx; gy -= bgy; gz -= bgz
        ax -= bax; ay -= bay
        f.write(f"{t:.6f} {gx:.6f} {gy:.6f} {gz:.6f} {ax:.6f} {ay:.6f} {az:.6f}\n")
print(f"IMU: {len(dtrim)} samples after bias-subtract + trim @ {1/np.mean(np.diff(dtrim[:,0])):.0f} Hz")
PYEOF

# Write config: exp 23 calibration + enhanced ORB + ThDepth sweep value
cat > $CFG << EOF
%YAML:1.0
Camera.type: "PinHole"
Camera.fx: 320.0
Camera.fy: 320.0
Camera.cx: 320.0
Camera.cy: 240.0
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: 640
Camera.height: 480
Camera.fps: 10
Camera.RGB: 0
Camera.bf: 40.0
ThDepth: ${TH}.0
DepthMapFactor: 1000.0

# exp 23 IMU params (filter=60)
IMU.NoiseGyro: 5.0e-3
IMU.NoiseAcc: 2.0e-2
IMU.GyroWalk: 1.0e-3
IMU.AccWalk: 5.0e-3
IMU.Frequency: 200

# exp 23 VERIFIED Tbc
Tbc: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ 0.0,  0.0,  1.0, 0.102,
          -1.0,  0.0,  0.0, 0.018,
           0.0, -1.0,  0.0, 0.132,
           0.0,  0.0,  0.0, 1.0]

# Enhanced ORB for forest - more features + more levels
ORBextractor.nFeatures: 3000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 12
ORBextractor.iniThFAST: 10
ORBextractor.minThFAST: 3

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3
Viewer.ViewpointX: 0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500
EOF

cd $SLAM
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
./Examples/RGB-D-Inertial/rgbd_inertial_offline \
    Vocabulary/ORBvoc.txt $CFG $TUM $TUM/associations.txt $IMU_OUT 2>&1 | tee $LOG

cp CameraTrajectory.txt $EXP/logs/exp46_vio_th${TH}_camera.txt 2>/dev/null || true
cp KeyFrameTrajectory.txt $EXP/logs/exp46_vio_th${TH}_kf.txt 2>/dev/null || true

python3 << PYEOF
import numpy as np, os
f='$EXP/logs/exp46_vio_th${TH}_camera.txt'
if not os.path.exists(f) or os.path.getsize(f)==0:
    print(f"TH=${TH}: NO TRAJ - VIO crashed"); exit(0)
s=np.array([[float(x) for x in l.split()] for l in open(f)])
gtd=np.loadtxt('$TUM/groundtruth.txt',usecols=(0,1,2,3))
gd={round(r[0],4):r[1:4] for r in gtd}
ms,mg=[],[]
for r in s:
    ts=round(r[0],4)
    if ts in gd: ms.append(r[1:4]); mg.append(gd[ts])
ms=np.array(ms); mg=np.array(mg)
if len(ms)<10: print(f"TH=${TH}: <10 matched poses"); exit(0)
sd=np.sum(np.sqrt(np.sum(np.diff(ms,axis=0)**2,axis=1)))
gd_=np.sum(np.sqrt(np.sum(np.diff(mg,axis=0)**2,axis=1)))
mu_m,mu_d=ms.mean(0),mg.mean(0); mz,dz=ms-mu_m,mg-mu_d
n=len(ms); H=mz.T@dz/n; U,D,Vt=np.linalg.svd(H)
S=np.eye(3)
if np.linalg.det(U)*np.linalg.det(Vt)<0: S[2,2]=-1
R=Vt.T@S@U.T; c=np.trace(np.diag(D)@S)*n/np.sum(mz**2)
t=mu_d-c*R@mu_m
al=c*(ms@R.T)+t
err=np.sqrt(np.sum((al-mg)**2,axis=1))
resets=sum(1 for l in open('$LOG') if 'Creation of new map' in l)-2
print(f"TH=${TH}: tracked={len(s)}/{len(gtd)} ({len(s)/len(gtd)*100:.0f}%) "
      f"resets={max(0,resets)} SLAM={sd:.1f}m GT={gd_:.1f}m ratio={sd/gd_:.3f} "
      f"scale={c:.3f} ATE_rmse={np.sqrt(np.mean(err**2)):.3f}m ATE_max={err.max():.3f}m")
PYEOF
