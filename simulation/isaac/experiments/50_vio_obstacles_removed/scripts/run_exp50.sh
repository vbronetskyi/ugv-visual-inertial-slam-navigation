#!/bin/bash
# Exp 50: Nav2+VIO roundtrip with DYNAMIC obstacles.
#
# Outbound: робот зустрічає 3 групи конусів + намет, Nav2 планує обхід
# Turnaround: supervisor автоматично прибирає всі обстаки через signal file
# Return: робот їде по чистому маршруту назад до спавну
#
# Phases:
#   1. Patch spawn_obstacles.py для south route (on-route positions)
#   2. Isaac Sim --synthetic-imu --route south --obstacles (spawns cones+tent)
#   3. VIO warmup (200+ frames)
#   4. Switch TF to SLAM+encoder, start Nav2
#   5. Launch supervisor (watches GT.x > 65)
#   6. Send roundtrip goals
#   7. At turnaround: supervisor triggers obstacle removal
#   8. Return path without obstacles
set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/50_vio_obstacles_removed
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-75}

# Phase 1: patch south obstacles (on-route positions)
python3 $EXP/scripts/patch_obstacles.py

# Install route
cp $EXP/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

# Clean
pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall_clock\|controller_server\|planner_server\|bt_navigator\|behavior_server\|velocity_smoother\|lifecycle_manager\|map_server\|send_roundtrip\|turnaround_sup" 2>/dev/null || true
sleep 5
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_remove_obstacles.txt

source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN

# Phase 2: Isaac with obstacles
echo ""; echo "=== PHASE 2: Isaac Sim + obstacles ==="
/opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --obstacles --duration 1800 \
    > $EXP/logs/isaac.log 2>&1 &
ISAAC_PID=$!
echo "Isaac: $ISAAC_PID"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "Recording: $REC"
echo "REC=$REC" > $EXP/logs/run_info.txt

# Phase 3: VIO + GT TF for warmup
echo ""; echo "=== PHASE 3: VIO warmup ==="
cd $SLAM
./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $EXP/config/vio_th160.yaml $REC \
    > $EXP/logs/vio.log 2>&1 &
VIO_PID=$!
echo "VIO: $VIO_PID"

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
source /opt/ros/jazzy/setup.bash
python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt > $EXP/logs/tf_warmup.log 2>&1 &
TF_PID=$!

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    if [ "$N" -ge 200 ] 2>/dev/null; then
        echo "  VIO ready: $N frames"; break
    fi
    sleep 2
done

# Phase 4: Switch TF, disable pure pursuit, start Nav2
echo ""; echo "=== PHASE 4: Switch to VIO + Nav2 ==="
kill $TF_PID 2>/dev/null; sleep 2
python3 $SCRIPTS/tf_wall_clock_relay.py --slam-encoder > $EXP/logs/tf_slam.log 2>&1 &
TF_PID=$!
sleep 5

# Disable pure pursuit - Nav2 takes over via /cmd_vel
touch /tmp/isaac_clear_route.txt
echo "  Pure pursuit disabled via signal"
sleep 2

ros2 launch $EXP/config/nav2_launch.py use_sim_time:=false > $EXP/logs/nav2.log 2>&1 &
NAV2_PID=$!
for i in $(seq 1 60); do
    if grep -q "Managed nodes are active" $EXP/logs/nav2.log 2>/dev/null; then
        echo "  Nav2 active"; break
    fi
    sleep 2
done
ros2 param set /controller_server progress_checker.movement_time_allowance 120.0 2>/dev/null || true

# Phase 5: Supervisor (watches for turnaround)
echo ""; echo "=== PHASE 5: Turnaround supervisor ==="
python3 $EXP/scripts/turnaround_supervisor.py --threshold 65 \
    > $EXP/logs/supervisor.log 2>&1 &
SUP_PID=$!
echo "Supervisor: $SUP_PID"

# Phase 6: Roundtrip goals
echo ""; echo "=== PHASE 6: Roundtrip goals ==="
python3 $EXP/scripts/send_roundtrip_goals.py \
    --route $EXP/config/south_roundtrip_route.json --spacing 4.0 \
    > $EXP/logs/goals.log 2>&1 &
GOALS_PID=$!

echo ""
echo "All PIDs: Isaac=$ISAAC_PID VIO=$VIO_PID TF=$TF_PID Nav2=$NAV2_PID Sup=$SUP_PID Goals=$GOALS_PID"
echo ""
echo "Monitor:"
echo "  tail -f $EXP/logs/goals.log       # Nav2 goal progress"
echo "  tail -f $EXP/logs/supervisor.log  # obstacle removal"
echo "  tail -f $EXP/logs/isaac.log       # Isaac (obstacles removed message)"
echo ""
echo "Stop: touch /tmp/slam_stop; pkill -9 -f 'run_husky_forest|rgbd_inertial|tf_wall|controller_server|send_roundtrip|turnaround'"
