#!/bin/bash
# Exp 49: Full roundtrip navigation using Nav2 + VIO SLAM localization.
#
# Based on exp 47 architecture (3-phase with warmup) extended to full roundtrip.
#
# Phase 1: Pure pursuit + VIO warmup (200+ frames)
# Phase 2: Switch TF to SLAM+encoder, start Nav2
# Phase 3: send_roundtrip_goals.py follows all 797 WPs @ 4m spacing (outbound+turn+return)

set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/49_nav2_vio_roundtrip
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-74}

# Install route
cp $EXP/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs installed')
"

# Clean
pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall_clock\|controller_server\|planner_server\|bt_navigator\|behavior_server\|velocity_smoother\|lifecycle_manager\|map_server\|send_roundtrip" 2>/dev/null || true
sleep 5
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt

source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN

# phase 1: isaac (pure pursuit) + vio + gt tf for warmup
echo ""; echo "=== PHASE 1: VIO warmup ==="
/opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1500 \
    > $EXP/logs/isaac.log 2>&1 &
ISAAC_PID=$!
echo "Isaac: $ISAAC_PID"

# Wait for Isaac ready
for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "Recording: $REC"
echo "REC=$REC" > $EXP/logs/run_info.txt

# Start VIO
cd $SLAM
./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $EXP/config/vio_th160.yaml $REC \
    > $EXP/logs/vio.log 2>&1 &
VIO_PID=$!
echo "VIO: $VIO_PID"

# GT TF relay during warmup
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
source /opt/ros/jazzy/setup.bash
python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt > $EXP/logs/tf_warmup.log 2>&1 &
TF_PID=$!
echo "TF (GT): $TF_PID"

# Wait for VIO to reach 200+ frames
echo "Waiting for VIO warmup (200+ frames)..."
for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    if [ "$N" -ge 200 ] 2>/dev/null; then
        echo "  VIO ready: $N frames"
        break
    fi
    sleep 2
done

# phase 2: switch tf to slam+encoder, start nav2
echo ""; echo "=== PHASE 2: Switch to VIO localization ==="
kill $TF_PID 2>/dev/null; sleep 2
python3 $SCRIPTS/tf_wall_clock_relay.py --slam-encoder > $EXP/logs/tf_slam.log 2>&1 &
TF_PID=$!
echo "TF (SLAM+encoder): $TF_PID"
sleep 5

ros2 launch $EXP/config/nav2_launch.py use_sim_time:=false > $EXP/logs/nav2.log 2>&1 &
NAV2_PID=$!
echo "Nav2: $NAV2_PID"

# Wait for Nav2 active
for i in $(seq 1 60); do
    if grep -q "Managed nodes are active" $EXP/logs/nav2.log 2>/dev/null; then
        echo "  Nav2 active"
        break
    fi
    sleep 2
done
# Set progress timeout high for slow sim
ros2 param set /controller_server progress_checker.movement_time_allowance 120.0 2>/dev/null || true

# phase 3: roundtrip goals
echo ""; echo "=== PHASE 3: Roundtrip goals ==="
python3 $EXP/scripts/send_roundtrip_goals.py \
    --route $EXP/config/south_roundtrip_route.json \
    --spacing 4.0 \
    > $EXP/logs/goals.log 2>&1 &
GOALS_PID=$!
echo "Goals: $GOALS_PID"

echo ""
echo "All started. Monitor:"
echo "  tail -f $EXP/logs/goals.log"
echo "  tail -f $EXP/logs/tf_slam.log"
echo ""
echo "PIDs: Isaac=$ISAAC_PID VIO=$VIO_PID TF=$TF_PID Nav2=$NAV2_PID Goals=$GOALS_PID"
echo "To stop: touch /tmp/slam_stop; pkill -9 -f 'run_husky_forest|rgbd|tf_wall|controller_server|send_roundtrip'"
