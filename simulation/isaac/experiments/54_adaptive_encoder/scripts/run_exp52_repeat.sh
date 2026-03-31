#!/bin/bash
# Exp 52 REPEAT run: v9 hybrid + teach-built map + live depth obstacle_layer.
# Robot drives same south route autonomously, this time with 3 cone walls
# added to Isaac scene. Cones are NOT in any config Nav2 reads - they must
# be discovered by the depth camera feeding obstacle_layer at runtime.
#
# Records to $E52/results/repeat_run/:
#   - costmap snapshots (pgm+yaml every 5s via costmap_snapshotter)
#   - ros2 bag of /plan, /cmd_vel, /tf, /slam_pose, sampled /depth_points
#   - tf_slam log, goals log, pp_follower log, etc.
set -eu

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
DOMAIN=${ROS_DOMAIN_ID:-85}

TEACH=$E52/teach
OUT=${REPEAT_OUT_DIR:-$E52/results/repeat_run}
mkdir -p $OUT/snapshots $OUT/plans

TEACH_MAP=$TEACH/south_teach_map.yaml
if [ ! -f "$TEACH_MAP" ]; then
    echo "ERROR: teach map not found at $TEACH_MAP - run run_exp52_teach.sh first"
    exit 1
fi

# Patch Nav2 yaml to point static_layer at teach map (instead of blank)
python3 - <<PY
import yaml, os
p = '$E52/config/nav2_planner_only.yaml'
txt = open(p).read()
# map_server yaml_filename -> teach map
new_txt = txt.replace(
    'yaml_filename: "/workspace/simulation/isaac/experiments/52_obstacles_v9/config/blank_south_map.yaml"',
    'yaml_filename: "$TEACH_MAP"'
)
open(p, 'w').write(new_txt)
print(f'Patched Nav2 yaml: map_server -> {"$TEACH_MAP"}')
PY

# Patch Nav2 launch to same teach map
python3 - <<PY
p = '$E52/config/nav2_launch_hybrid.py'
txt = open(p).read()
new_txt = txt.replace(
    'map_yaml = "/workspace/simulation/isaac/experiments/52_obstacles_v9/config/blank_south_map.yaml"',
    'map_yaml = "$TEACH_MAP"'
)
open(p, 'w').write(new_txt)
print(f'Patched Nav2 launch: map_yaml -> {"$TEACH_MAP"}')
PY

# Ensure cones are in Isaac scene spawn - they were patched already
python3 $E52/scripts/patch_obstacles_exp52.py

cp $E52/config/south_roundtrip_route.json /workspace/simulation/isaac/route_memory/south/anchors.json
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
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt /tmp/exp52*.sh

echo "=== REPEAT PHASE 1: Isaac (WITH cone obstacles) + VIO warmup ==="
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
    Vocabulary/ORBvoc.txt $E52/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO
echo "VIO: $VIO"

cat > /tmp/exp52r_tf_gt.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/exp52r_tf_gt.sh
nohup bash /tmp/exp52r_tf_gt.sh > $OUT/tf_warmup.log 2>&1 &
TF=$!; disown $TF
echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "=== REPEAT PHASE 2: Stop robot, switch to VIO TF, start Nav2 + PP + recorders ==="
# v10 ordering: stop robot BEFORE tf_relay switch (stationary ALIGN window)
touch /tmp/isaac_clear_route.txt
echo "Pure pursuit DISABLED - waiting 5s for robot to settle"
sleep 5

kill $TF 2>/dev/null; sleep 2

cat > /tmp/exp52r_tf_slam.sh <<'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --slam-encoder
EOF
chmod +x /tmp/exp52r_tf_slam.sh
nohup bash /tmp/exp52r_tf_slam.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF
echo "TF(SLAM): $TF"
sleep 3

cat > /tmp/exp52r_nav2.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec ros2 launch $E52/config/nav2_launch_hybrid.py
EOF
chmod +x /tmp/exp52r_nav2.sh
nohup bash /tmp/exp52r_nav2.sh > $OUT/nav2.log 2>&1 &
NAV2=$!; disown $NAV2
echo "Nav2: $NAV2"

cat > /tmp/exp52r_pp.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/pure_pursuit_path_follower.py
EOF
chmod +x /tmp/exp52r_pp.sh
nohup bash /tmp/exp52r_pp.sh > $OUT/pp_follower.log 2>&1 &
PP=$!; disown $PP
echo "PP: $PP"

# Turnaround supervisor - physically removes cones from Isaac on turnaround
cat > /tmp/exp52r_sup.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/turnaround_supervisor.py --turnaround-x 60.0 --past-margin 2.0
EOF
chmod +x /tmp/exp52r_sup.sh
nohup bash /tmp/exp52r_sup.sh > $OUT/supervisor.log 2>&1 &
SUP=$!; disown $SUP
echo "Supervisor: $SUP"

# Costmap snapshotter - pgm+yaml every 5s
cat > /tmp/exp52r_cms.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/costmap_snapshotter.py --out-dir $OUT/snapshots --period 5.0 --topic /global_costmap/costmap
EOF
chmod +x /tmp/exp52r_cms.sh
nohup bash /tmp/exp52r_cms.sh > $OUT/costmap_snapshotter.log 2>&1 &
CMS=$!; disown $CMS
echo "CostmapSnapshotter: $CMS"

# Plan logger - saves every /plan as CSV (numeric data, not rosbag)
cat > /tmp/exp52r_plan.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/plan_logger.py --out-dir $OUT/plans
EOF
chmod +x /tmp/exp52r_plan.sh
nohup bash /tmp/exp52r_plan.sh > $OUT/plan_logger.log 2>&1 &
PLN=$!; disown $PLN
echo "PlanLogger: $PLN"

for i in $(seq 1 60); do
    grep -q "Managed nodes are active\|planner_server.*active" $OUT/nav2.log 2>/dev/null && break
    sleep 2
done
echo "Nav2 ready"

echo "=== REPEAT PHASE 3: Send goals (v9 pipeline) ==="
sleep 3
cat > /tmp/exp52r_goals.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=85
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $E52/scripts/send_goals_hybrid.py --trajectory /workspace/simulation/isaac/experiments/48_vio_roundtrip__/logs/exp48_vio_traj.csv --spacing 4.0 --goal-timeout 300 --tolerance 3.0
EOF
chmod +x /tmp/exp52r_goals.sh
nohup bash /tmp/exp52r_goals.sh > $OUT/goals.log 2>&1 &
GOALS=$!; disown $GOALS
echo "Goals: $GOALS"

echo ""
echo "PIDs: Isaac=$ISAAC VIO=$VIO TF=$TF Nav2=$NAV2 PP=$PP Sup=$SUP CMS=$CMS Plan=$PLN Goals=$GOALS"
echo "Monitor:"
echo "  tail -f $OUT/tf_slam.log"
echo "  tail -f $OUT/goals.log"
echo "  ls -la $OUT/snapshots/ | tail"
echo "  ls -la $OUT/plans/ | tail"
echo ""
echo "Recorded numeric data layout:"
echo "  $OUT/tf_slam.log                   - full tf_relay (err, nav, gt over time)"
echo "  $OUT/goals.log                     - WP progress + REACHED/TIMEOUT"
echo "  $OUT/pp_follower.log               - PP cmds (v,w,err) + ANTI-SPIN events"
echo "  $OUT/supervisor.log                - turnaround fire event"
echo "  $OUT/snapshots/costmap_XXXX.npy    - int8 occupancy array (W×H, -1/0/1-100)"
echo "  $OUT/snapshots/costmap_XXXX.json   - {ts,origin,res,robot_xy,n_occ,n_free}"
echo "  $OUT/snapshots/snapshots_summary.csv  - one row per snapshot"
echo "  $OUT/plans/plan_XXXX.csv           - (seq,ts,x,y) per Nav2 replan"
echo "  $OUT/plans/plans_summary.csv       - one row per plan (length, goal, etc.)"
