#!/bin/bash
# Exp 51 v2: Hybrid Nav2 planner + PP follower, with ScaleCorrector in tf_relay.
# Scale correction: compare encoder dist vs VIO dist over 60s window,
# apply scale to VIO deltas before blending (fixes VIO scale drift during Nav2 stop-and-go).
set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/51_hybrid_nav2_pp
V2=$EXP/v2_scale_correction
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

# Install VIO trajectory as route (for pure pursuit during warmup)
cp $V2/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

# Clean
pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall\|planner_server\|map_server\|pure_pursuit_path\|send_goals_hybrid\|lifecycle_manager" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt

# PHASE 1: Isaac + warmup
echo "=== PHASE 1: Isaac + VIO warmup ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1500 \
    </dev/null > $V2/logs/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC
echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $V2/logs/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $V2/config/vio_th160.yaml $REC \
    </dev/null > $V2/logs/vio.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

# GT TF for warmup
cat > /tmp/exp51v2_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp51v2_tf_gt.sh
nohup bash /tmp/exp51v2_tf_gt.sh > $V2/logs/tf_warmup.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

# Wait VIO warmup
for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

# PHASE 2: Switch to SLAM+encoder (with scale correction) + start Nav2 planner + PP
echo "=== PHASE 2: Switch to VIO+scale, start Nav2 planner + PP follower ==="
kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp51v2_tf_slam.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --slam-encoder
EOF
chmod +x /tmp/exp51v2_tf_slam.sh
nohup bash /tmp/exp51v2_tf_slam.sh > $V2/logs/tf_slam.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM+SCALE): $TF"
sleep 3

touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED in Isaac"
sleep 2

# Start Nav2 planner-only
cat > /tmp/exp51v2_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $V2/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/exp51v2_nav2.sh
nohup bash /tmp/exp51v2_nav2.sh > $V2/logs/nav2.log 2>&1 &
NAV2=$!; disown $NAV2
echo "Nav2 planner: $NAV2"

cat > /tmp/exp51v2_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $V2/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/exp51v2_pp.sh
nohup bash /tmp/exp51v2_pp.sh > $V2/logs/pp_follower.log 2>&1 &
PP=$!; disown $PP
echo "PP follower: $PP"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $V2/logs/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 planner active"

# PHASE 3: Send goals
echo "=== PHASE 3: Send goals ==="
sleep 3
cat > /tmp/exp51v2_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $V2/scripts/send_goals_hybrid.py --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv --spacing 4.0 --goal-timeout 300 --tolerance 1.5
EOF
chmod +x /tmp/exp51v2_goals.sh
nohup bash /tmp/exp51v2_goals.sh > $V2/logs/goals.log 2>&1 &
GOALS=$!; disown $GOALS
echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 PP=$PP Goals=$GOALS"
echo "Monitor:"
echo "  tail -f $V2/logs/tf_slam.log"
echo "  tail -f $V2/logs/goals.log"
