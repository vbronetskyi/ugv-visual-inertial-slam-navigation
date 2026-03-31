#!/bin/bash
# Exp 53 REPEAT run: v9 hybrid + teach map + live obstacle_layer + v53 additions:
#   - PP follower with proximity speed limiter (slows v_max based on
#     costmap cost ahead of robot)
#   - Goal sender with proactive WP projection (BFS each future WP out
#     of inflated zones as soon as obstacle_layer sees them)
# Obstacle layout taken from exp 52 run 4 (3/2/4 cones + tent on route).
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
E53=/workspace/simulation/isaac/experiments/53_proactive_reroute
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=$E53/teach
OUT=${REPEAT_OUT_DIR:-$E53/results/repeat_run}
mkdir -p $OUT/snapshots $OUT/plans

TEACH_MAP=$TEACH/south_teach_map.yaml
if [ ! -f "$TEACH_MAP" ]; then
    echo "ERROR: teach map not found at $TEACH_MAP - copy from exp 52 teach/"
    exit 1
fi

# Patch Nav2 yaml to point static_layer at teach map
python3 - <<PY
p = '$E53/config/nav2_planner_only.yaml'
txt = open(p).read()
import re
txt = re.sub(r'yaml_filename:.*',
             'yaml_filename: "$TEACH_MAP"', txt, count=1)
open(p, 'w').write(txt)
print(f'Patched Nav2 yaml: map_server -> $TEACH_MAP')
PY

# Patch Nav2 launch
python3 - <<PY
p = '$E53/config/nav2_launch_hybrid.py'
txt = open(p).read()
import re
txt = re.sub(r'map_yaml = "[^"]*"',
             'map_yaml = "$TEACH_MAP"', txt, count=1)
# Also update config path to E53
txt = re.sub(r'config = "[^"]*"',
             'config = "$E53/config/nav2_planner_only.yaml"', txt, count=1)
open(p, 'w').write(txt)
print(f'Patched Nav2 launch: map_yaml -> $TEACH_MAP')
PY

# Obstacle layout patch - same layout as exp 52 run 4 (reuse its patcher)
python3 $E52/scripts/patch_obstacles_exp52.py

cp $E53/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
python3 -c "
import json
routes = json.load(open('/tmp/slam_routes.json'))
routes['south'] = json.load(open('/workspace/simulation/isaac/route_memory/south/anchors.json'))
json.dump(routes, open('/tmp/slam_routes.json','w'))
print(f'Route: {len(routes[\"south\"])} WPs')
"

pkill -9 -f "run_husky_forest\|rgbd_inertial\|tf_wall\|planner_server\|map_server\|pure_pursuit_path\|send_goals_hybrid\|lifecycle_manager\|costmap_snapshotter\|turnaround_supervisor" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/exp53*.sh

echo "=== REPEAT PHASE 1: Isaac + VIO warmup ==="
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route south --obstacles --duration 1500 \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC
echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E53/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

cat > /tmp/exp53r_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp53r_tf_gt.sh
nohup bash /tmp/exp53r_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== REPEAT PHASE 2: Stop robot, switch to VIO TF, start Nav2 + PP ==="
touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED - waiting 5s for robot to settle"
sleep 5

kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp53r_tf_slam.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --slam-encoder
EOF
chmod +x /tmp/exp53r_tf_slam.sh
nohup bash /tmp/exp53r_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM): $TF"
sleep 3

cat > /tmp/exp53r_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E53/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/exp53r_nav2.sh
nohup bash /tmp/exp53r_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2
echo "Nav2: $NAV2"

cat > /tmp/exp53r_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E53/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/exp53r_pp.sh
nohup bash /tmp/exp53r_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP
echo "PP (v53 proximity): $PP"

cat > /tmp/exp53r_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E53/scripts/turnaround_supervisor.py --turnaround-x 60.0 --past-margin 2.0
EOF
chmod +x /tmp/exp53r_sup.sh
nohup bash /tmp/exp53r_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP
echo "Supervisor: $SUP"

cat > /tmp/exp53r_cms.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E53/scripts/costmap_snapshotter.py --out-dir $OUT/snapshots --period 5.0 --topic /global_costmap/costmap
EOF
chmod +x /tmp/exp53r_cms.sh
nohup bash /tmp/exp53r_cms.sh > $OUT/costmap_snapshotter.log 2>&1 &
CMS=$!; disown $CMS
echo "CostmapSnapshotter: $CMS"

cat > /tmp/exp53r_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E53/scripts/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/exp53r_plan.sh
nohup bash /tmp/exp53r_plan.sh > $OUT/plan_logger.log 2>&1 &
PLN=$!; disown $PLN
echo "PlanLogger: $PLN"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== REPEAT PHASE 3: Send goals (v53 proactive projection) ==="
sleep 3
cat > /tmp/exp53r_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E53/scripts/send_goals_hybrid.py --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip__/logs/exp48_vio_traj.csv --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/exp53r_goals.sh
nohup bash /tmp/exp53r_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS
echo "Goals (v53): $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 PP=$PP Sup=$SUP CMS=$CMS Plan=$PLN Goals=$GOALS"
echo "OUT: $OUT"
