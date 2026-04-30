#!/bin/bash
#generic our-custom T&R repeat for any route - takes ROUTE env var and
#uses canonical scripts/common + scripts/nav_our_custom (mirrors the
#phase-2 routes/<R>/repeat/scripts/run_repeat.sh pattern).  without
#--obstacles it's the clean-map sanity run.  outputs go to
#/root/isaac_tr_datasets/$ROUTE/repeat/results/repeat_run/
set -eu

ROUTE=${ROUTE:?set ROUTE (e.g. 10_nmid_smid)}
DOMAIN=${ROS_DOMAIN_ID:-86}
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-5400}
USE_OBSTACLES=${USE_OBSTACLES:-0}

SCRIPTS=/workspace/simulation/isaac/scripts/common
NAV_OURS=/workspace/simulation/isaac/scripts/nav_our_custom
BASELINE_COMMON=/workspace/simulation/isaac/experiments/_baselines_common
ROUTE_REPEAT=/workspace/simulation/isaac/routes/$ROUTE/repeat
SLAM=/workspace/third_party/ORB_SLAM3

# shellcheck disable=SC1091
source "$BASELINE_COMMON/route_params.sh"
ISAAC_ROUTE_ARG="${RP_ISAAC_ROUTE[$ROUTE]:?no params for $ROUTE}"
SPAWN_ARGS="${RP_SPAWN_ARGS[$ROUTE]}"
SUPERVISOR_ARGS="${RP_SUPERVISOR_ARGS[$ROUTE]}"
# Extract explicit turnaround + spawn xy (same numbers supervisor uses -
# nothing GT about them, just the canonical mission endpoints).
FINAL_X=$(echo "$SUPERVISOR_ARGS" | grep -oE -- '--final-x [-0-9.]+' | awk '{print $2}')
FINAL_Y=$(echo "$SUPERVISOR_ARGS" | grep -oE -- '--final-y [-0-9.]+' | awk '{print $2}')
RETURN_X=$(echo "$SPAWN_ARGS"    | grep -oE -- '--spawn-x [-0-9.]+' | awk '{print $2}')
RETURN_Y=$(echo "$SPAWN_ARGS"    | grep -oE -- '--spawn-y [-0-9.]+' | awk '{print $2}')
GOAL_ENDPOINTS=""
[ -n "$FINAL_X" ]  && [ -n "$FINAL_Y" ]  && GOAL_ENDPOINTS="$GOAL_ENDPOINTS --final-x $FINAL_X --final-y $FINAL_Y"
[ -n "$RETURN_X" ] && [ -n "$RETURN_Y" ] && GOAL_ENDPOINTS="$GOAL_ENDPOINTS --return-x $RETURN_X --return-y $RETURN_Y"
TEACH=/root/isaac_tr_datasets/${RP_TEACH_SUBDIR[$ROUTE]}

OUT=${REPEAT_OUT_DIR:-/root/isaac_tr_datasets/$ROUTE/repeat/results/repeat_run}
mkdir -p "$OUT/plans"

# Back-compat symlink from legacy path so older scripts keep working
LEGACY_LINK=$ROUTE_REPEAT/results/repeat_run
if [ ! -e "$LEGACY_LINK" ] && [ ! -L "$LEGACY_LINK" ]; then
  mkdir -p "$(dirname "$LEGACY_LINK")"
  ln -s "$OUT" "$LEGACY_LINK"
fi

TEACH_MAP=$TEACH/teach_map.yaml
LANDMARKS=$TEACH/landmarks.pkl
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$TEACH_MAP" ] || { echo "ERROR: missing $TEACH_MAP"; exit 1; }
[ -f "$LANDMARKS" ] || { echo "ERROR: missing $LANDMARKS"; exit 1; }
[ -f "$TRAJ" ]      || { echo "ERROR: missing $TRAJ"; exit 1; }

VIO_CFG=$ROUTE_REPEAT/config/vio_th160.yaml
NAV_LAUNCH=$ROUTE_REPEAT/config/nav2_launch_hybrid.py
[ -f "$VIO_CFG" ]   || { echo "ERROR: missing $VIO_CFG"; exit 1; }
[ -f "$NAV_LAUNCH" ]|| { echo "ERROR: missing $NAV_LAUNCH"; exit 1; }

OBS_FLAG=""
if [ "$USE_OBSTACLES" = "1" ]; then
  OBS_FLAG="--obstacles"
fi

echo "=== REPEAT-OURS $ROUTE  timeout=${ROUTE_TIMEOUT_S}s  obstacles=${USE_OBSTACLES} ==="
echo "  out:      $OUT"
echo "  spawn:    $SPAWN_ARGS"
echo "  superv:   $SUPERVISOR_ARGS"
echo "  teach:    $TEACH"

pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager|costmap_snapshotter|turnaround_supervisor|plan_logger|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt \
      /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/ro_*.sh

echo "--- PHASE 1: Isaac + VIO warmup (GT tf)"
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py || true

setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route $ISAAC_ROUTE_ARG $OBS_FLAG --duration 4500 \
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

cat > /tmp/ro_tf_gt.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/ro_tf_gt.sh
nohup bash /tmp/ro_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT warmup): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "--- PHASE 2: tf_slam v55 + matcher + Nav2 + PP + supervisor"
touch /tmp/isaac_clear_route.txt
sleep 5
kill $TF 2>/dev/null || true; sleep 2

cat > /tmp/ro_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/ro_tf_slam.sh
nohup bash /tmp/ro_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/ro_matcher.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/visual_landmark_matcher.py \
    --landmarks $LANDMARKS --out-csv $OUT/anchor_matches.csv
EOF
chmod +x /tmp/ro_matcher.sh
nohup bash /tmp/ro_matcher.sh > $OUT/landmark_matcher.log 2>&1 &
MATCHER=$!; disown $MATCHER; echo "Matcher: $MATCHER"

cat > /tmp/ro_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $NAV_LAUNCH
EOF
chmod +x /tmp/ro_nav2.sh
nohup bash /tmp/ro_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2; echo "Nav2: $NAV2"

cat > /tmp/ro_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/ro_pp.sh
nohup bash /tmp/ro_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP; echo "PP: $PP"

cat > /tmp/ro_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/turnaround_supervisor.py $SUPERVISOR_ARGS
EOF
chmod +x /tmp/ro_sup.sh
nohup bash /tmp/ro_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

cat > /tmp/ro_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/ro_plan.sh
nohup bash /tmp/ro_plan.sh > $OUT/plan_logger.log 2>&1 &
PLAN=$!; disown $PLAN; echo "PlanLogger: $PLAN"

# Wait for Nav2 to be ready
for i in $(seq 1 90); do
    grep -q "bt_navigator.*active\|Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready (or timed out)"

echo "--- PHASE 3: Send goals"
sleep 3

cat > /tmp/ro_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $NAV_OURS/send_goals_hybrid.py \
    --trajectory $TRAJ --spacing 4.0 --goal-timeout 300 --tolerance 3.0 \
    --final-tolerance 2.0 $GOAL_ENDPOINTS
EOF
chmod +x /tmp/ro_goals.sh
nohup bash /tmp/ro_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS; echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Matcher=$MATCHER Nav2=$NAV2 PP=$PP Sup=$SUP Plan=$PLAN Goals=$GOALS"
echo "OUT: $OUT"

# Wait-loop: done when goals.log has RESULT line, or supervisor FIRED and 10 s pass with robot back at spawn, or timeout
cleanup_all () {
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager|costmap_snapshotter|turnaround_supervisor|plan_logger|ros2 launch" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

START_TS=$(date +%s)
while true; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - START_TS ))
    if grep -q "^.*RESULT: reached" $OUT/goals.log 2>/dev/null; then
        echo "=== goals-log RESULT line seen after ${ELAPSED}s ==="
        # Copy GT trajectory before shutting down
        cp /tmp/isaac_trajectory.csv $OUT/traj_gt.csv 2>/dev/null || true
        sleep 5
        cleanup_all
        trap - EXIT
        exit 0
    fi
    if [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ]; then
        echo "!!! REPEAT TIMEOUT after ${ELAPSED}s"
        cp /tmp/isaac_trajectory.csv $OUT/traj_gt.csv 2>/dev/null || true
        cleanup_all
        exit 1
    fi
    # bail if Isaac died
    kill -0 $ISAAC 2>/dev/null || {
        echo "!!! ISAAC died after ${ELAPSED}s"
        cp /tmp/isaac_trajectory.csv $OUT/traj_gt.csv 2>/dev/null || true
        cleanup_all
        exit 1
    }
    sleep 5
done
