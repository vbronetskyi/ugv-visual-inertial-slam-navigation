#!/usr/bin/env bash
# Exp 45: South forest with Nav2 SmacPlanner2D + velocity smoother
#
# Goal: test SmacPlanner2D (footprint-aware planner) against NavFn (exp 44)
# on south forest route. Both planners had ~47 WP reach; SmacPlanner got
# 4m further into the NE forest (peak 93m vs 89m) before stalling.
#
# Usage:
#   cd /workspace/simulation/isaac/experiments/45_south_forest_smac
#   ./scripts/run_exp45.sh [run_tag]
#
# run_tag defaults to "v1" (creates logs/exp45_<tag>_*.log)
set -euo pipefail

TAG=${1:-v1}
EXP=/workspace/simulation/isaac/experiments/45_south_forest_smac
LOGS=$EXP/logs
DBG=$EXP/results/costmap_debug_$TAG
mkdir -p $LOGS $DBG

echo "=== Exp 45 ($TAG) - killing zombies ==="
ps aux | grep -E "isaacsim|nav2|trajectory_follower|map_server|behavior_server|controller_server|planner_server|bt_navigator|lifecycle_manager|tf_wall_clock|velocity_smoother" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 2

echo "=== [1/4] Isaac Sim (south forest) ==="
cd /workspace/simulation/isaac/scripts
nohup bash -c 'source /tmp/ros_env.sh && \
  /opt/isaac-sim-6.0.0/python.sh run_husky_forest.py \
    --route south --duration 1200 --nav2-bridge' \
  > $LOGS/exp45_${TAG}_isaac.log 2>&1 &
ISAAC=$!
echo "  Isaac PID=$ISAAC, waiting 60s for boot..."
sleep 60

if ! grep -q "SLAM recording" $LOGS/exp45_${TAG}_isaac.log; then
  echo "  Isaac failed to start (see log). Exiting."
  exit 1
fi
BAG=$(ls -td /root/bags/husky_real/isaac_slam_* | head -1)
echo "  Isaac up, bag: $BAG"

echo "=== [2/4] TF relay (--use-gt) + Nav2 (SmacPlanner2D) ==="
nohup bash -c 'source /opt/ros/jazzy/setup.bash && \
  export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
  python3 /workspace/simulation/isaac/scripts/tf_wall_clock_relay.py --use-gt' \
  > $LOGS/exp45_${TAG}_tf.log 2>&1 &
echo "  TF PID=$!"

nohup bash -c "source /opt/ros/jazzy/setup.bash && \
  export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
  ros2 launch $EXP/config/nav2_launch.py use_sim_time:=false" \
  > $LOGS/exp45_${TAG}_nav2.log 2>&1 &
echo "  Nav2 PID=$!, waiting 60s for activation..."
sleep 60

if ! grep -q "SmacPlanner2D" $LOGS/exp45_${TAG}_nav2.log; then
  echo "  WARN: SmacPlanner2D plugin not loaded (see nav2 log)"
fi

echo "=== [3/4] Trajectory goals (99 dense WPs, 2m spacing) ==="
nohup bash -c "source /opt/ros/jazzy/setup.bash && \
  export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && \
  export DEBUG_DIR=$DBG && \
  python3 /workspace/simulation/isaac/scripts/send_trajectory_goals.py \
    --anchors $EXP/config/south_anchors_dense.json \
    --spacing 2.0 --direction outbound" \
  > $LOGS/exp45_${TAG}_goals.log 2>&1 &
echo "  Goals PID=$!"

echo "=== [4/4] Monitoring (Ctrl+C to stop) ==="
echo "  Watch:   tail -f $LOGS/exp45_${TAG}_goals.log"
echo "  GT traj: tail -f $BAG/groundtruth.csv"
echo "  When done: cp $BAG/groundtruth.csv $LOGS/exp45_${TAG}_traj.csv"
