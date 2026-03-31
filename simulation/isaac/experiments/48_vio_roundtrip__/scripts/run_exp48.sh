#!/bin/bash
# Exp 48: VIO live roundtrip - pure pursuit + ORB-SLAM3 VIO tracking.
#
# Pipeline:
#   1. Install dense roundtrip route (USD-obstacle-aware, 0.5m WPs)
#   2. Isaac Sim + pure pursuit + synthetic IMU (records camera/depth/IMU/GT)
#   3. ORB-SLAM3 rgbd_inertial_live in parallel
#   4. Stop VIO when route done
#   5. Analyze with Umeyama alignment
#
# Usage: ./run_exp48.sh
set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/48_vio_roundtrip
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3

# 1. Install route (use snapshot) or rebuild from scratch
if [ "${REBUILD_ROUTE:-0}" = "1" ]; then
    python3 $EXP/scripts/build_route.py
else
    cp $EXP/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
    python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs (from snapshot)')
"
fi

# 2. Clean state
pkill -9 -f "run_husky_forest\|rgbd_inertial" 2>/dev/null || true
sleep 3
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt

# 3. Start Isaac Sim (pure pursuit + synthetic IMU + route=south)
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=73
/opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1200 \
    > $EXP/logs/isaac_final.log 2>&1 &
ISAAC_PID=$!
echo "Isaac: $ISAAC_PID"

# Wait for Isaac
for i in $(seq 1 120); do
    [ -f /tmp/isaac_pose.txt ] && break
    sleep 1
done
sleep 5

REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "Recording: $REC"

# 4. Start VIO SLAM
cd $SLAM
./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $EXP/config/vio_th160.yaml $REC \
    > $EXP/logs/vio_final.log 2>&1 &
VIO_PID=$!
echo "VIO: $VIO_PID"

echo "Monitor progress with:"
echo "  tail -f $EXP/logs/isaac_final.log"
echo "  cat /tmp/slam_pose.txt"

# 5. Wait for completion (outer script needs to signal)
# User stops with: touch /tmp/slam_stop; pkill -9 run_husky_forest
