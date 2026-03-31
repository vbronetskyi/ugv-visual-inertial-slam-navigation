#!/bin/bash
# Exp 52 TEACH run: drive south route once without obstacles, using pure
# pursuit from the existing anchors file. VIO localizes, depth camera feeds
# teach_run_depth_mapper.py which builds a 2D occupancy grid from only
# what the depth camera observes. Output:
#   teach/south_teach_map.yaml + .pgm
#   teach/teach_trajectory.csv (from tf_slam log)
#   teach/teach_depth_mapper.log
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=$E52/teach
mkdir -p $TEACH

cp $E52/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall\|planner_server\|map_server\|pure_pursuit_path\|send_goals_hybrid\|lifecycle_manager\|teach_run_depth_mapper" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/exp52*.sh

echo "=== TEACH PHASE 1: Isaac (no obstacles) + VIO warmup ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
# NOTE: no --obstacles flag - teach run sees the ORIGINAL environment
# (forest + houses + rocks only). Cones appear only in repeat run.
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --duration 1500 \
    </dev/null > $TEACH/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC
echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $TEACH/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E52/config/vio_th160.yaml $REC \
    </dev/null > $TEACH/vio.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

# Initial tf_relay in GT mode while VIO warms up
cat > /tmp/exp52t_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp52t_tf_gt.sh
nohup bash /tmp/exp52t_tf_gt.sh > $TEACH/tf_warmup.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

# Wait for VIO warmup
for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== TEACH PHASE 2: Switch to VIO TF (SLAM+encoder) + start depth mapper ==="
# Teach phase keeps Isaac's internal pure pursuit driving the route
# (no Nav2, no goal sender). We just swap tf_relay to VIO-based TF and
# launch the depth mapper. Align averaging will retry while robot moves.
kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp52t_tf_slam.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --slam-encoder
EOF
chmod +x /tmp/exp52t_tf_slam.sh
nohup bash /tmp/exp52t_tf_slam.sh > $TEACH/tf_slam.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM): $TF"
sleep 3

# Launch depth mapper - subscribes /depth_points + TF, builds teach map
cat > /tmp/exp52t_mapper.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/teach_run_depth_mapper.py \\
    --out-prefix $TEACH/south_teach_map \\
    --origin-x -110 --origin-y -50 \\
    --width-m 200 --height-m 60 \\
    --res 0.1
EOF
chmod +x /tmp/exp52t_mapper.sh
nohup bash /tmp/exp52t_mapper.sh > $TEACH/teach_depth_mapper.log 2>&1 &
MAPPER=$!; disown $MAPPER
echo "DepthMapper: $MAPPER"

# Isaac's internal pure pursuit is already driving (we never disabled it).
# No Nav2, no goal sender - teach phase uses the existing anchor-based
# controller to drive the south route end-to-end.

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Mapper=$MAPPER"
echo "Monitor:"
echo "  tail -f $TEACH/teach_depth_mapper.log"
echo "  tail -f $TEACH/tf_slam.log"
