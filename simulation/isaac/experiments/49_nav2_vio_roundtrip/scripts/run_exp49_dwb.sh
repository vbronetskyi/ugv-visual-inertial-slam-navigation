#!/bin/bash
# Exp 49 continuation: Nav2 DWB + VIO localization, VIO trajectory route.
# Baseline without obstacles - verify Nav2 actually drives robot (not pure pursuit).
set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-82}

# Install VIO trajectory as route
cp /workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv $EXP/logs/vio_reference.csv 2>/dev/null || true

# Route install (dense 0.5m) - copied from exp 49
cp /workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json 2>/dev/null || \
  cp /workspace/simulation/isaac/experiments/48_vio_roundtrip/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json

python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

# Clean
pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall\|controller_server\|planner_server\|bt_navigator\|behavior_server\|velocity_smoother\|lifecycle_manager\|map_server\|send_waypoints\|ros2 launch" 2>/dev/null || true
sleep 5
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_remove_obstacles.txt /tmp/isaac_clear_route.txt

source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN

# Phase 1: Isaac + pure pursuit warmup (no --obstacles flag)
echo "=== PHASE 1: Isaac + pure pursuit warmup ==="
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1500 \
    </dev/null > $EXP/logs/isaac_dwb.log 2>&1 &
ISAAC=$!; disown $ISAAC
echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $EXP/logs/run_info_dwb.txt

# Start VIO + GT TF
cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $EXP/config/vio_th160.yaml $REC \
    </dev/null > $EXP/logs/vio_dwb.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
source /opt/ros/jazzy/setup.bash
setsid python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt </dev/null > $EXP/logs/tf_warmup_dwb.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

# Wait for VIO warmup
for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

# Phase 2: Switch to VIO TF + disable pure pursuit + Nav2
echo "=== PHASE 2: Switch to VIO + Nav2 ==="
kill $TF 2>/dev/null; sleep 2
setsid python3 $SCRIPTS/tf_wall_clock_relay.py --slam-encoder </dev/null > $EXP/logs/tf_slam_dwb.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM): $TF"
sleep 3

touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED"
sleep 2

setsid bash -c "source /opt/ros/jazzy/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DOMAIN_ID=$DOMAIN && export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib && exec ros2 launch $EXP/config/nav2_launch_dwb.py use_sim_time:=false" </dev/null > $EXP/logs/nav2_dwb.log 2>&1 &
NAV2=$!; disown $NAV2
echo "Nav2: $NAV2"

# Wait for active
for i in $(seq 1 60); do
    grep -q "Managed nodes are active" $EXP/logs/nav2_dwb.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 active"

# Phase 3: Send waypoints
echo "=== PHASE 3: Send waypoints ==="
sleep 3
setsid python3 $EXP/scripts/send_waypoints.py \
    --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv \
    --spacing 4.0 --goal-timeout 300 \
    </dev/null > $EXP/logs/goals_dwb.log 2>&1 &
GOALS=$!; disown $GOALS
echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 Goals=$GOALS"
echo "Monitor:"
echo "  tail -f $EXP/logs/goals_dwb.log"
echo "  tail -f $EXP/logs/isaac_dwb.log | grep NAV2"
