#!/bin/bash
# Run ORB-SLAM3 RGB-D only (no IMU) on exp 48 recording for comparison.
set -euo pipefail

REC=${1:-/root/bags/husky_real/isaac_slam_1776423021}
EXP=/workspace/simulation/isaac/experiments/48_vio_roundtrip
SLAM=/workspace/third_party/ORB_SLAM3
TUM=/tmp/tum_exp48_rgbd

rm -rf $TUM; mkdir -p $TUM/rgb $TUM/depth

python3 << PYEOF
import os
SRC="$REC"; OUT="$TUM"
assoc = []
for fname in sorted(f for f in os.listdir(f'{SRC}/camera_rgb') if f.endswith('.jpg')):
    ts = fname.replace('.jpg','')
    dpng = f'{ts}.png'
    if not os.path.exists(f'{SRC}/camera_depth/{dpng}'): continue
    os.symlink(os.path.abspath(f'{SRC}/camera_rgb/{fname}'), f'{OUT}/rgb/{fname}')
    os.symlink(os.path.abspath(f'{SRC}/camera_depth/{dpng}'), f'{OUT}/depth/{dpng}')
    assoc.append(f"{ts} rgb/{fname} {ts} depth/{dpng}\n")
open(f'{OUT}/associations.txt','w').writelines(assoc)
print(f"TUM: {len(assoc)} pairs")
PYEOF

cd $SLAM
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt \
    $EXP/config/rgbd_only.yaml $TUM $TUM/associations.txt \
    2>&1 | tee $EXP/logs/rgbd_only.log | grep -E "Map|Reset|init|fps"

cp CameraTrajectory.txt $EXP/logs/rgbd_only_camera.txt
echo "Saved: $EXP/logs/rgbd_only_camera.txt"
