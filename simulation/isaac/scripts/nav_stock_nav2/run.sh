#!/bin/bash
# Pure stock Nav2 baseline on an arbitrary route (takes ROUTE env var).
#
# Shared infrastructure with per-route run_repeat.sh:
#   - Isaac Sim (run_husky_forest.py --route X --obstacles)   - same
#   - ORB-SLAM3 RGB-D-Inertial VIO                            - same
#   - tf_wall_clock_relay.py --use-gt   (Phase 1 warmup)      - same
#   - tf_wall_clock_relay_v55.py --slam-encoder               - same (stays no_anchor)
#   - turnaround_supervisor.py                                - same (FIRE at 10 m)
#
# Replaced with stock:
#   Nav2 full stack   : map_server + planner_server + controller_server (RPP)
#                       + behavior_server + bt_navigator + waypoint_follower
#                       (launched from $E74/config/nav2_launch_stock.py)
#   WP feeder         : $E74/scripts/waypoint_follower_client.py
#                       (FollowWaypoints action, stop_on_failure: false)
#
# Removed from ours:
#   visual_landmark_matcher  - pure VIO+encoder drift (no anchor correction)
#   send_goals_hybrid        - replaced by stock waypoint_follower + client
#   pure_pursuit_path_follower - replaced by stock controller_server
set -eu

ROUTE=${ROUTE:?set ROUTE (e.g. 09_se_ne)}
E74=/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline
BASELINE_COMMON=/workspace/simulation/isaac/experiments/_baselines_common
ROUTE_REPEAT_DIR=/workspace/simulation/isaac/routes/$ROUTE/repeat
# Per-route repeat dir hosts scripts (tf_relay_v55, turnaround_supervisor,
# run_husky_forest, etc.) and the vio config.  We re-use them unchanged.
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-87}

# shellcheck disable=SC1091
source "$BASELINE_COMMON/route_params.sh"

ISAAC_ROUTE_ARG="${RP_ISAAC_ROUTE[$ROUTE]:?route $ROUTE has no params}"
SPAWN_ARGS="${RP_SPAWN_ARGS[$ROUTE]}"
SUPERVISOR_ARGS="${RP_SUPERVISOR_ARGS[$ROUTE]}"
TEACH=/root/isaac_tr_datasets/${RP_TEACH_SUBDIR[$ROUTE]}

OUT=${STOCK_OUT_DIR:-/root/isaac_tr_datasets/$ROUTE/baseline_stock_nav2}
mkdir -p $OUT
# back-compat symlink at $E74/results/run_$ROUTE
LINK=$E74/results/run_$ROUTE
if [ ! -L "$LINK" ] && [ ! -e "$LINK" ]; then
  mkdir -p "$(dirname "$LINK")"
  ln -s "$OUT" "$LINK"
fi

TEACH_MAP=$TEACH/teach_map.yaml
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$TEACH_MAP" ] || { echo "ERROR: missing $TEACH_MAP"; exit 1; }
[ -f "$TRAJ" ]      || { echo "ERROR: missing $TRAJ"; exit 1; }

# Select a vio config from the per-route dir (first .yaml matching vio)
VIO_CFG=$(ls "$ROUTE_REPEAT_DIR/config/"vio*.yaml 2>/dev/null | head -1)
[ -f "$VIO_CFG" ] || { echo "ERROR: no vio config at $ROUTE_REPEAT_DIR/config/"; exit 1; }

python3 - <<PY
import re
p='$E74/config/nav2_stock_params.yaml'
txt=open(p).read()
txt=re.sub(r'yaml_filename:.*', f'yaml_filename: "$TEACH_MAP"', txt, count=1)
open(p,'w').write(txt)
p='$E74/config/nav2_launch_stock.py'
txt=open(p).read()
txt=re.sub(r'map_yaml = "[^"]*"', f'map_yaml = "$TEACH_MAP"', txt, count=1)
open(p,'w').write(txt)
print('patched config + launch with teach_map for $ROUTE')
PY

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|rgbd_live|python.*tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|python.*pure_pursuit_path|python.*send_goals|python.*waypoint_follower_client|python.*gap_nav_teach|lifecycle_manager|python.*costmap_snapshotter|python.*turnaround_supervisor|python.*plan_logger|python.*visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/r74_*.sh

echo "=== PHASE 1: Isaac + VIO warmup (GT tf) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py || true

setsid /opt/isaac-sim-6.0.0/python.sh $ROUTE_REPEAT_DIR/scripts/run_husky_forest.py \
    --synthetic-imu --route $ISAAC_ROUTE_ARG --obstacles --duration 2400 \
    $SPAWN_ARGS \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt
echo "ROUTE=$ROUTE" >> $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt "$VIO_CFG" $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/r74_tf_gt.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/r74_tf_gt.sh
nohup bash /tmp/r74_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT warmup): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== PHASE 2: tf_slam v55 (NO matcher) + stock Nav2 + supervisor ==="
touch /tmp/isaac_clear_route.txt
sleep 5
kill $TF 2>/dev/null || true; sleep 2

cat > /tmp/r74_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/r74_tf_slam.sh
nohup bash /tmp/r74_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/r74_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E74/config/nav2_launch_stock.py
EOF
chmod +x /tmp/r74_nav2.sh
nohup bash /tmp/r74_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2; echo "Nav2(stock): $NAV2"

cat > /tmp/r74_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/turnaround_supervisor.py $SUPERVISOR_ARGS
EOF
chmod +x /tmp/r74_sup.sh
nohup bash /tmp/r74_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

for i in $(seq 1 90); do
    grep -q "bt_navigator.*active\|Managed nodes are active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== PHASE 3: waypoint_follower client ==="
sleep 3
cat > /tmp/r74_client.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E74/scripts/waypoint_follower_client.py \
    --trajectory $TRAJ --spacing 4.0
EOF
chmod +x /tmp/r74_client.sh
nohup bash /tmp/r74_client.sh > $OUT/goals.log 2>&1 &
CLIENT=$!; disown $CLIENT; echo "Client: $CLIENT"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 Sup=$SUP Client=$CLIENT"
echo "OUT: $OUT"

# WAIT_LOOP_FOR_RESULT (with watchdog early-abort)
# 7200 s wall = 2 h, comfortably above our custom's longest run (68 min on 07).
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-7200}
START_TS=$(date +%s)
cleanup_all() {
    pkill -TERM -f "waypoint_follower_client|baselines_common/watchdog" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|pure_pursuit_path|turnaround_supervisor|waypoint_follower_client|baselines_common/watchdog|lifecycle_manager|ros2 launch" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

# Launch watchdog: ABORTs run early if
#   - any critical PID dies (Isaac, VIO, Nav2)
#   - robot GT stalls (<1.5 m movement in 180 s after 240 s warmup)
#   - no new WP REACHED in last 600 s (after 240 s warmup)
nohup python3 /workspace/simulation/isaac/experiments/_baselines_common/watchdog.py \
    --pids "$ISAAC,$VIO,$NAV2" \
    --goals-log "$OUT/goals.log" \
    --warmup-s 240 --gt-stall-window-s 180 --gt-stall-min-m 1.5 \
    --no-wp-max-s 0 --tick-s 10 > $OUT/watchdog.log 2>&1 &
WDPID=$!; disown $WDPID
echo "Watchdog: $WDPID"

while true; do
    ELAPSED=$(( $(date +%s) - START_TS ))
    if grep -q "RESULT:" "$OUT/goals.log" 2>/dev/null; then
        grep "RESULT:" "$OUT/goals.log" | tail -2
        cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true
        cleanup_all; trap - EXIT; exit 0
    fi
    if [ -f /tmp/baseline_abort.txt ]; then
        REASON=$(cat /tmp/baseline_abort.txt 2>/dev/null)
        echo "WATCHDOG ABORT after ${ELAPSED}s: $REASON"
        cp /tmp/baseline_abort.txt "$OUT/watchdog_abort.txt" 2>/dev/null || true
        rm -f /tmp/baseline_abort.txt
        cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true
        cleanup_all; exit 2
    fi
    [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ] && { echo "TIMEOUT after ${ELAPSED}s"; cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true; cleanup_all; exit 1; }
    sleep 5
done
