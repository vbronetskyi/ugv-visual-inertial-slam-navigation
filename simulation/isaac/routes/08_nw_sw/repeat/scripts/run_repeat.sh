#!/bin/bash
# Exp 67 REPEAT run on road route -
# exp 59 pipeline + exp 66 teach artifacts + accel-noise IMU.
#
# Consumes:
#   /workspace/.../71_teach_08_nw_sw_accel_noise/teach/08_nw_sw/
#       teach_map.yaml, teach_map.pgm, landmarks.pkl, vio_pose_dense.csv
#
# Produces:
#   results/repeat_run/{isaac,vio,tf_slam,nav2,pp_follower,supervisor,plan_logger,
#                       landmark_matcher,goals,anchor_matches.csv}.log
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
E66=/workspace/simulation/isaac/routes/08_nw_sw/teach
E72=/workspace/simulation/isaac/routes/08_nw_sw/repeat
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=/root/isaac_tr_datasets/08_nw_sw/teach/teach_outputs
OUT=${REPEAT_OUT_DIR:-$E72/results/repeat_run}
mkdir -p $OUT/plans

TEACH_MAP=$TEACH/teach_map.yaml
LANDMARKS=$TEACH/landmarks.pkl
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$TEACH_MAP" ]  || { echo "ERROR: missing $TEACH_MAP"; exit 1; }
[ -f "$LANDMARKS" ]  || { echo "ERROR: missing $LANDMARKS"; exit 1; }
[ -f "$TRAJ" ]       || { echo "ERROR: missing $TRAJ"; exit 1; }

# Patch Nav2 launch + planner config to point at exp 66 teach_map and exp 67 planner cfg
python3 - <<PY
import re
for p, patterns in [
    ('$E72/config/nav2_planner_only.yaml', [(r'yaml_filename:.*', f'yaml_filename: "$TEACH_MAP"')]),
    ('$E72/config/nav2_launch_hybrid.py',  [
        (r'config = "[^"]*"',   f'config = "$E72/config/nav2_planner_only.yaml"'),
        (r'map_yaml = "[^"]*"', f'map_yaml = "$TEACH_MAP"'),
    ]),
]:
    txt = open(p).read()
    for pat, repl in patterns:
        txt = re.sub(pat, repl, txt, count=1)
    open(p, 'w').write(txt)
    print(f'Patched {p}')
PY

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|python.*tf_wall_clock|planner_server|map_server|python.*pure_pursuit_path|python.*send_goals_hybrid|lifecycle_manager|python.*costmap_snapshotter|python.*turnaround_supervisor|python.*plan_logger|python.*visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/r04r_*.sh

echo "=== PHASE 1: Isaac + VIO warmup (GT tf) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

# ensure /tmp/slam_routes.json has 08_nw_sw
python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py

# --obstacles enables spawn_obstacles["08_nw_sw"] - 3 cone groups + tent
setsid /opt/isaac-sim-6.0.0/python.sh /workspace/simulation/isaac/scripts/common/run_husky_forest.py \
    --synthetic-imu --route 08_nw_sw --obstacles --duration 4500 \
    --spawn-x -90.00 --spawn-y 35.00 --spawn-yaw -2.0921 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E72/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/r04r_tf_gt.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/r04r_tf_gt.sh
nohup bash /tmp/r04r_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
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

cat > /tmp/r04r_tf_slam.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/r04r_tf_slam.sh
nohup bash /tmp/r04r_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55): $TF"
sleep 3

cat > /tmp/r04r_matcher.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/visual_landmark_matcher.py \
    --landmarks $LANDMARKS --out-csv $OUT/anchor_matches.csv
EOF
chmod +x /tmp/r04r_matcher.sh
nohup bash /tmp/r04r_matcher.sh > $OUT/landmark_matcher.log 2>&1 &
MATCHER=$!; disown $MATCHER; echo "LandmarkMatcher: $MATCHER"

cat > /tmp/r04r_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E72/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/r04r_nav2.sh
nohup bash /tmp/r04r_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2; echo "Nav2: $NAV2"

cat > /tmp/r04r_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/r04r_pp.sh
nohup bash /tmp/r04r_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP; echo "PP: $PP"

cat > /tmp/r04r_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/turnaround_supervisor.py --final-x -90.0 --final-y -35.0 --near-radius 10.0
EOF
chmod +x /tmp/r04r_sup.sh
nohup bash /tmp/r04r_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

cat > /tmp/r04r_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/r04r_plan.sh
nohup bash /tmp/r04r_plan.sh > $OUT/plan_logger.log 2>&1 &
PLN=$!; disown $PLN; echo "PlanLogger: $PLN"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== PHASE 3: Send goals (teach trajectory, spacing=4m) ==="
sleep 3
cat > /tmp/r04r_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:\${PYTHONPATH:-}
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/nav_our_custom/send_goals_hybrid.py \
    --trajectory $TRAJ \
    --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/r04r_goals.sh
nohup bash /tmp/r04r_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS; echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Matcher=$MATCHER Nav2=$NAV2 PP=$PP Sup=$SUP Plan=$PLN Goals=$GOALS"
echo "OUT: $OUT"
echo ""
echo "Monitor:"
echo "  tail -f $OUT/tf_slam.log        # regime + anchor_age + err"
echo "  tail -f $OUT/anchor_matches.csv # every matcher attempt"
echo "  tail -f $OUT/goals.log          # WP progress"

# WAIT_LOOP_FOR_RESULT - block until goals.log emits RESULT line or timeout.
# Without this the orchestrator would see run_repeat.sh exit immediately and
# kill all background sim processes.
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-8400}
START_TS=$(date +%s)
cleanup_all() {
    pkill -TERM -f "visual_landmark_matcher|send_goals_hybrid|plan_logger" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|turnaround_supervisor|planner_server|map_server|lifecycle_manager|plan_logger|costmap_snapshotter" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT
while true; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - START_TS ))
    if grep -q "RESULT: reached" "$OUT/goals.log" 2>/dev/null; then
        echo ""
        echo "=== REPEAT done ==="
        grep "RESULT:" "$OUT/goals.log" | tail -2
        cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true
        cleanup_all
        trap - EXIT
        exit 0
    fi
    if [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ]; then
        echo "!!! ROUTE TIMEOUT after ${ELAPSED}s"
        cleanup_all
        exit 1
    fi
    sleep 5
done
