#!/bin/bash
# Exp 47: Run VIO offline on full outbound recording.
# Uses the same pipeline as exp 46 but on the live recording.
set -euo pipefail

REC=${1:-/root/bags/husky_real/isaac_slam_1776371616}
SLAM=/workspace/third_party/ORB_SLAM3
EXP=/workspace/simulation/isaac/experiments/47_nav2_vio_slam
CFG=$EXP/config/vio_th160.yaml
LOG=$EXP/logs/vio_full.log
TUM=$EXP/tum_full

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib

echo "=== Building TUM dataset from recording ==="
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

# Find turnaround: max x position (outbound endpoint)
max_x_t = max(gt.keys(), key=lambda t: gt[t][0])
max_x = gt[max_x_t][0]
print(f"Turnaround at t={max_x_t:.1f}, x={max_x:.1f}")

# Only keep outbound: up to turnaround time + 5s margin
outbound_end = max_x_t + 5.0

rgb={round(float(f.replace('.jpg','')),4):f for f in os.listdir(f'{SRC}/camera_rgb')}
dep={round(float(f.replace('.png','')),4):f for f in os.listdir(f'{SRC}/camera_depth')}
gtk=np.array(sorted(gt))
assoc=[]; gt_out=[]; prev=None
for ts in sorted(rgb):
    if ts<keep or ts>outbound_end or ts not in dep: continue
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
print(f"TUM built: {len(assoc)} frames (outbound only)")
PYEOF

echo ""
echo "=== Converting IMU ==="
IMU_OUT=$EXP/imu_full.txt
python3 << PYEOF
import numpy as np, math
d = []
for line in open('$REC/imu.csv').readlines()[1:]:
    parts = line.strip().split(',')
    if len(parts) >= 7:
        d.append([float(x) for x in parts[:11]])
d = np.array(d)
gt = np.loadtxt('$REC/groundtruth.csv', delimiter=',', skiprows=1)

t = d[:,0]; t0 = t[0]
bias_mask = (t > t0+0.5) & (t < t0+1.5)
if bias_mask.sum() < 50: bias_mask = t < t0+1.0
bgx = d[bias_mask,4].mean()
bgy = d[bias_mask,5].mean()
bgz = d[bias_mask,6].mean()
bax = d[bias_mask,1].mean()
bay = d[bias_mask,2].mean()
print(f"Bias: gyro=({bgx:+.4f},{bgy:+.4f},{bgz:+.4f}) accel=({bax:+.4f},{bay:+.4f})")

mv = None
for row in gt:
    if math.hypot(row[1]+95, row[2]+6) > 0.3: mv = row[0]; break
keep = mv + 1.0 if mv else 0
mask = d[:,0] >= keep
dtrim = d[mask]
with open('$IMU_OUT','w') as f:
    for r in dtrim:
        t,ax,ay,az,gx,gy,gz = r[:7]
        gx -= bgx; gy -= bgy; gz -= bgz
        ax -= bax; ay -= bay
        f.write(f"{t:.6f} {gx:.6f} {gy:.6f} {gz:.6f} {ax:.6f} {ay:.6f} {az:.6f}\n")
print(f"IMU: {len(dtrim)} samples @ {1/np.mean(np.diff(dtrim[:,0])):.0f} Hz")
PYEOF

echo ""
echo "=== Running ORB-SLAM3 VIO ==="
cd $SLAM
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
./Examples/RGB-D-Inertial/rgbd_inertial_offline \
    Vocabulary/ORBvoc.txt $CFG $TUM $TUM/associations.txt $IMU_OUT 2>&1 | tee $LOG

cp CameraTrajectory.txt $EXP/logs/vio_full_camera.txt 2>/dev/null || true
cp KeyFrameTrajectory.txt $EXP/logs/vio_full_kf.txt 2>/dev/null || true

echo ""
echo "=== ATE Evaluation ==="
python3 << PYEOF
import numpy as np, os
f='$EXP/logs/vio_full_camera.txt'
if not os.path.exists(f) or os.path.getsize(f)==0:
    print("NO TRAJ - VIO crashed"); exit(0)
s=np.array([[float(x) for x in l.split()] for l in open(f)])
gtd=np.loadtxt('$TUM/groundtruth.txt',usecols=(0,1,2,3))
gd={round(r[0],4):r[1:4] for r in gtd}
ms,mg=[],[]
for r in s:
    ts=round(r[0],4)
    if ts in gd: ms.append(r[1:4]); mg.append(gd[ts])
ms=np.array(ms); mg=np.array(mg)
if len(ms)<10: print(f"<10 matched poses"); exit(0)
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
print(f"tracked={len(s)}/{len(gtd)} ({len(s)/len(gtd)*100:.0f}%) "
      f"resets={max(0,resets)} SLAM={sd:.1f}m GT={gd_:.1f}m ratio={sd/gd_:.3f} "
      f"scale={c:.3f} ATE_rmse={np.sqrt(np.mean(err**2)):.3f}m ATE_max={err.max():.3f}m")

# Segment analysis
n_segs = 5
seg_size = len(err) // n_segs
for i in range(n_segs):
    s_i = i * seg_size
    e_i = (i+1)*seg_size if i < n_segs-1 else len(err)
    se = err[s_i:e_i]; sg = mg[s_i:e_i]
    print(f"  {i*20}-{(i+1)*20}%: ATE={np.sqrt(np.mean(se**2)):.3f}m max={se.max():.3f}m x=[{sg[0,0]:.0f}..{sg[-1,0]:.0f}]")
PYEOF
