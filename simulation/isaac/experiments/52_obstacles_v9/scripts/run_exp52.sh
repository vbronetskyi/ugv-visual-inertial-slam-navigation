#!/bin/bash
# Exp 52: v9 hybrid pipeline (IMU fix + no loop + NoiseAcc 0.275 + tolerance 3m
# + anti-spin + home-zone collapse) + 3 static obstacles on south route.
# Obstacles also baked into nav map (map_with_obstacles.pgm).
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

# Ensure spawn_obstacles has exp 52 positions (physical cones in Isaac scene)
python3 $E52/scripts/patch_obstacles_exp52.py

# Nav2 starts with a BLANK static map - robot does not know the obstacle or
# tree positions in advance (honest teach-and-repeat: real robot can't survey
# the forest). Nav2 obstacle_layer reads /depth_points at runtime to discover
# obstacles locally as the robot approaches them.

cp $E52/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall\|planner_server\|map_server\|pure_pursuit_path\|send_goals_hybrid\|lifecycle_manager" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/exp52*.sh

echo "=== PHASE 1: Isaac + VIO warmup (with obstacles spawned) ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --obstacles --duration 1500 \
    </dev/null > $E52/logs/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC
echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $E52/logs/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E52/config/vio_th160.yaml $REC \
    </dev/null > $E52/logs/vio.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

cat > /tmp/exp52_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp52_tf_gt.sh
nohup bash /tmp/exp52_tf_gt.sh > $E52/logs/tf_warmup.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== PHASE 2: Switch to SLAM+encoder + Nav2 (with inflated obstacle map) + PP ==="
kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp52_tf_slam.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --slam-encoder
EOF
chmod +x /tmp/exp52_tf_slam.sh
nohup bash /tmp/exp52_tf_slam.sh > $E52/logs/tf_slam.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM): $TF"
sleep 3

touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED"
sleep 2

cat > /tmp/exp52_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E52/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/exp52_nav2.sh
nohup bash /tmp/exp52_nav2.sh > $E52/logs/nav2.log 2>&1 &
NAV2=$!; disown $NAV2
echo "Nav2: $NAV2"

cat > /tmp/exp52_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/exp52_pp.sh
nohup bash /tmp/exp52_pp.sh > $E52/logs/pp_follower.log 2>&1 &
PP=$!; disown $PP
echo "PP: $PP"

# Turnaround supervisor: on turnaround, drops obstacles in Isaac AND
# hot-swaps Nav2 map_server to the blank map so planner sees clear path
# on the return leg.
cat > /tmp/exp52_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/turnaround_supervisor.py \\
    --turnaround-x 60.0 --past-margin 2.0
EOF
chmod +x /tmp/exp52_sup.sh
nohup bash /tmp/exp52_sup.sh > $E52/logs/supervisor.log 2>&1 &
SUP=$!; disown $SUP
echo "Supervisor: $SUP"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $E52/logs/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== PHASE 3: Send goals (tolerance 3m, home-zone collapse) ==="
sleep 3
cat > /tmp/exp52_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/send_goals_hybrid.py --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip/logs/exp48_vio_traj.csv --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/exp52_goals.sh
nohup bash /tmp/exp52_goals.sh > $E52/logs/goals.log 2>&1 &
GOALS=$!; disown $GOALS
echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 PP=$PP Sup=$SUP Goals=$GOALS"
echo "Monitor:"
echo "  tail -f $E52/logs/tf_slam.log"
echo "  tail -f $E52/logs/goals.log"
