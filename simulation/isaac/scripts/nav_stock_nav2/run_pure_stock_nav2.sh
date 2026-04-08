#!/bin/bash
# Pure stock Nav2 baseline - removes our anchor-correction contribution.
#
# Keeps from our pipeline:
#   tf_wall_clock_relay_v55 (--slam-encoder)  - publishes map->base_link from
#       VIO + encoder fusion; stays in `no_anchor` regime the whole time since
#       the landmark matcher is NOT started (pure VIO drift).
#   turnaround_supervisor  - writes /tmp/isaac_remove_obstacles.txt when
#       robot is <10 m from the final goal; fair to let stock Nav2 have a
#       clean return leg.
#
# Removes:
#   visual_landmark_matcher  - no anchor correction, VIO drifts uncorrected.
#
# Stock:
#   Nav2 full stack (planner + controller_server RPP + bt_navigator with
#   stock navigate_through_poses BT + behavior_server recoveries + waypoint_follower).
#   RPP configured with rotate_to_heading=true, allow_reversing=true (v3 fix
#   for the turnaround overshoot observed in exp 73 v2).
set -eu

ROUTE=09_se_ne
E74=/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline
E72=/workspace/simulation/isaac/routes/$ROUTE/repeat
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-87}

TEACH=/root/isaac_tr_datasets/$ROUTE/teach/teach_outputs
OUT=${PURE_OUT_DIR:-$E74/results/run_09}
mkdir -p $OUT

TEACH_MAP=$TEACH/teach_map.yaml
TRAJ=$TEACH/vio_pose_dense.csv
[ -f "$TEACH_MAP" ] || { echo "ERROR: missing $TEACH_MAP"; exit 1; }
[ -f "$TRAJ" ]      || { echo "ERROR: missing $TRAJ"; exit 1; }

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

pkill -9 -f "python.*run_husky_forest|rgbd_inertial|python.*tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|python.*pure_pursuit_path|python.*send_goals|python.*waypoint_follower_client|lifecycle_manager|python.*costmap_snapshotter|python.*turnaround_supervisor|python.*plan_logger|python.*visual_landmark|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/r74_*.sh

echo "=== PHASE 1: Isaac + VIO warmup (GT tf) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py

setsid /opt/isaac-sim-6.0.0/python.sh $E72/scripts/run_husky_forest.py \
    --synthetic-imu --route $ROUTE --obstacles --duration 2400 \
    --spawn-x 65.00 --spawn-y -35.00 --spawn-yaw 1.0496 \
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
exec python3 $E72/scripts/tf_wall_clock_relay_v55.py --slam-encoder
EOF
chmod +x /tmp/r74_tf_slam.sh
nohup bash /tmp/r74_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(SLAM v55, no matcher): $TF"
sleep 3

# no visual_landmark_matcher - tf stays in no_anchor regime

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
exec python3 $E72/scripts/turnaround_supervisor.py --final-x 65.0 --final-y 35.0 --near-radius 10.0
EOF
chmod +x /tmp/r74_sup.sh
nohup bash /tmp/r74_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP; echo "Supervisor: $SUP"

echo "Waiting for stock Nav2 bt_navigator to become active..."
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

# WAIT_LOOP_FOR_RESULT
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-2400}
START_TS=$(date +%s)
cleanup_all() {
    pkill -TERM -f "waypoint_follower_client" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall_clock|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|pure_pursuit_path|turnaround_supervisor|waypoint_follower_client|lifecycle_manager|ros2 launch" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT
while true; do
    ELAPSED=$(( $(date +%s) - START_TS ))
    if grep -q "RESULT:" "$OUT/goals.log" 2>/dev/null; then
        grep "RESULT:" "$OUT/goals.log" | tail -2
        cp /tmp/isaac_trajectory.csv "$OUT/traj_gt.csv" 2>/dev/null || true
        cleanup_all; trap - EXIT; exit 0
    fi
    [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ] && { echo "TIMEOUT after ${ELAPSED}s"; cleanup_all; exit 1; }
    sleep 5
done
