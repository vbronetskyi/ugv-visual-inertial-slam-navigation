#!/bin/bash
# Our full T&R pipeline, but ORB-SLAM3 in pure RGB-D mode (no IMU).
# Route-parametric: takes ROUTE env var (e.g. 04_nw_se, 09_se_ne, road, ...).
#
# Same as per-route run_repeat.sh, ONLY differences:
#   - SLAM binary   : ./Examples/RGB-D/rgbd_live   (was rgbd_inertial_live)
#   - SLAM config   : $E76/config/rgbd_th160.yaml  (no IMU noise / Tbc blocks)
#   - --synthetic-imu CLI flag dropped from Isaac launch (IMU not consumed)
# Everything else (matcher, tf_relay_v55, Nav2 planner_only, pure_pursuit,
# send_goals_hybrid with detour ring, turnaround_supervisor) is identical.
set -eu

ROUTE=${ROUTE:?set ROUTE (e.g. 09_se_ne)}
E76=/workspace/simulation/isaac/experiments/76_rgbd_no_imu_ours
BASELINE_COMMON=/workspace/simulation/isaac/experiments/_baselines_common
ROUTE_REPEAT_DIR=/workspace/simulation/isaac/routes/$ROUTE/repeat
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-89}

# shellcheck disable=SC1091
source "$BASELINE_COMMON/route_params.sh"

ISAAC_ROUTE_ARG="${RP_ISAAC_ROUTE[$ROUTE]:?route $ROUTE has no params}"
SPAWN_ARGS="${RP_SPAWN_ARGS[$ROUTE]}"
SUPERVISOR_ARGS="${RP_SUPERVISOR_ARGS[$ROUTE]}"
TEACH=/root/isaac_tr_datasets/${RP_TEACH_SUBDIR[$ROUTE]}

OUT=${RGBD_OUT_DIR:-/root/isaac_tr_datasets/$ROUTE/baseline_rgbd_no_imu}
mkdir -p $OUT/plans
# back-compat symlink at $E76/results/run_$ROUTE
LINK=$E76/results/run_$ROUTE
if [ ! -L "$LINK" ] && [ ! -e "$LINK" ]; then
  mkdir -p "$(dirname "$LINK")"
  ln -s "$OUT" "$LINK"
fi

TEACH_MAP=$TEACH/teach_map.yaml
LANDMARKS=$TEACH/landmarks.pkl
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$TEACH_MAP" ] || { echo "ERROR: missing $TEACH_MAP"; exit 1; }
[ -f "$LANDMARKS" ] || { echo "ERROR: missing $LANDMARKS"; exit 1; }
[ -f "$TRAJ" ]      || { echo "ERROR: missing $TRAJ"; exit 1; }

# Reuse the per-route Nav2 launch + planner YAML; patch them to point
# at our teach_map (same as per-route run_repeat.sh does).
python3 - <<PY
import re
for p, patterns in [
    ('$ROUTE_REPEAT_DIR/config/nav2_planner_only.yaml', [(r'yaml_filename:.*', f'yaml_filename: "$TEACH_MAP"')]),
    ('$ROUTE_REPEAT_DIR/config/nav2_launch_hybrid.py',  [
        (r'config = "[^"]*"',   f'config = "$ROUTE_REPEAT_DIR/config/nav2_planner_only.yaml"'),
        (r'map_yaml = "[^"]*"', f'map_yaml = "$TEACH_MAP"'),
    ]),
]:
    try:
        txt = open(p).read()
        for pat, repl in patterns:
            txt = re.sub(pat, repl, txt, count=1)
        open(p, 'w').write(txt)
        print(f'Patched {p}')
    except FileNotFoundError:
        print(f'[skip] {p} missing')
PY

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|rgbd_live|python.*tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|python.*pure_pursuit_path|python.*send_goals|python.*waypoint_follower_client|python.*gap_nav_teach|lifecycle_manager|python.*costmap_snapshotter|python.*turnaround_supervisor|python.*plan_logger|python.*visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/r76_*.sh

echo "=== PHASE 1: Isaac + VIO warmup (GT tf) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py || true

setsid /opt/isaac-sim-6.0.0/python.sh $ROUTE_REPEAT_DIR/scripts/run_husky_forest.py \
    --route $ISAAC_ROUTE_ARG --obstacles --duration 4500 \
    $SPAWN_ARGS \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt
echo "ROUTE=$ROUTE" >> $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D/rgbd_live \
    Vocabulary/ORBvoc.txt $E76/config/rgbd_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/r76_tf_gt.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/r76_tf_gt.sh
nohup bash /tmp/r76_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT warmup): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== PHASE 2: tf_slam v55 + matcher + Nav2 + PP + supervisor ==="
touch /tmp/isaac_clear_route.txt
sleep 5
kill $TF 2>/dev/null || true; sleep 2

cat > /tmp/r76_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/r76_tf_slam.sh
nohup bash /tmp/r76_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/r76_matcher.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/visual_landmark_matcher.py \
    --landmarks $LANDMARKS --out-csv $OUT/anchor_matches.csv
EOF
chmod +x /tmp/r76_matcher.sh
nohup bash /tmp/r76_matcher.sh > $OUT/landmark_matcher.log 2>&1 &
MATCHER=$!; disown $MATCHER; echo "LandmarkMatcher: $MATCHER"

cat > /tmp/r76_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $ROUTE_REPEAT_DIR/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/r76_nav2.sh
nohup bash /tmp/r76_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2; echo "Nav2: $NAV2"

cat > /tmp/r76_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/r76_pp.sh
nohup bash /tmp/r76_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP; echo "PP: $PP"

cat > /tmp/r76_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/turnaround_supervisor.py $SUPERVISOR_ARGS
EOF
chmod +x /tmp/r76_sup.sh
nohup bash /tmp/r76_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

cat > /tmp/r76_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/r76_plan.sh
nohup bash /tmp/r76_plan.sh > $OUT/plan_logger.log 2>&1 &
PLN=$!; disown $PLN; echo "PlanLogger: $PLN"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== PHASE 3: Send goals (teach trajectory, spacing=4m) ==="
sleep 3
cat > /tmp/r76_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $ROUTE_REPEAT_DIR/scripts/send_goals_hybrid.py \
    --trajectory $TRAJ \
    --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/r76_goals.sh
nohup bash /tmp/r76_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS; echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Matcher=$MATCHER Nav2=$NAV2 PP=$PP Sup=$SUP Plan=$PLN Goals=$GOALS"
echo "OUT: $OUT"

# WAIT_LOOP_FOR_RESULT (with watchdog early-abort)
# 7200 s wall = 2 h, comfortably above our custom's longest run (68 min on 07).
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-7200}
START_TS=$(date +%s)
cleanup_all() {
    pkill -TERM -f "visual_landmark_matcher|send_goals_hybrid|plan_logger|baselines_common/watchdog" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall_clock|planner_server|map_server|pure_pursuit_path|send_goals|turnaround_supervisor|visual_landmark|plan_logger|baselines_common/watchdog|lifecycle_manager|ros2 launch" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

# Watchdog: abort early if robot stalls, WP progress freezes, or Isaac/VIO/Nav2 die.
nohup python3 /workspace/simulation/isaac/experiments/_baselines_common/watchdog.py \
    --pids "$ISAAC,$VIO,$NAV2" \
    --goals-log "$OUT/goals.log" \
    --warmup-s 240 --gt-stall-window-s 180 --gt-stall-min-m 1.5 \
    --no-wp-max-s 0 --tick-s 10 > $OUT/watchdog.log 2>&1 &
WDPID=$!; disown $WDPID
echo "Watchdog: $WDPID"

while true; do
    ELAPSED=$(( $(date +%s) - START_TS ))
    if grep -q "RESULT: reached" "$OUT/goals.log" 2>/dev/null; then
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
