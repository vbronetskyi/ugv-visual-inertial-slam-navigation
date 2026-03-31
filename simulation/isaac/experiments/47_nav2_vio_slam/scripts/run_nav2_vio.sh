#!/bin/bash
# Exp 47: Nav2 navigation with VIO SLAM localization on south forest route.
#
# Architecture:
#   Isaac Sim (nav2-bridge + synthetic-imu) -> camera + IMU + GT pose
#   ORB-SLAM3 VIO (rgbd_inertial_live) -> /tmp/slam_pose.txt
#   tf_wall_clock_relay --slam-encoder -> TF for Nav2
#   Nav2 (SmacPlanner2D + MPPI + velocity_smoother) -> cmd_vel
#   send_trajectory_goals.py -> sequential WP following
#
# Speed: cmd 0.25 -> ~0.85 m/s actual (Husky 3.4× scaling)
# VIO validated at 0.76 m/s with ATE 0.103m in forest (exp 46)
set -euo pipefail

EXP=/workspace/simulation/isaac/experiments/47_nav2_vio_slam
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3
VIO_CFG=$EXP/config/vio_th160.yaml
WP=$EXP/config/south_anchors_fixed.json
DURATION=700
LOG_DIR=$EXP/logs
DEBUG_DIR=$EXP/results/nav2_debug

mkdir -p $LOG_DIR $DEBUG_DIR

echo "=== Exp 47: Nav2 + VIO SLAM ==="
echo "Killing zombie processes..."

# Kill zombies
pkill -9 -f "run_husky_forest" 2>/dev/null || true
pkill -9 -f "rgbd_inertial_live" 2>/dev/null || true
pkill -9 -f "tf_wall_clock_relay" 2>/dev/null || true
pkill -9 -f "send_trajectory_goals" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true
pkill -9 -f "controller_server" 2>/dev/null || true
pkill -9 -f "planner_server" 2>/dev/null || true
pkill -9 -f "bt_navigator" 2>/dev/null || true
pkill -9 -f "behavior_server" 2>/dev/null || true
pkill -9 -f "velocity_smoother" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
sleep 3

# Clean state
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt
rm -f /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_trajectory.csv

# Set ROS2 environment
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=47
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib

echo ""
echo "=== Step 1: Starting Isaac Sim (nav2-bridge + synthetic-imu) ==="
source /tmp/ros_env.sh 2>/dev/null || true
/opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --nav2-bridge --synthetic-imu --route south --duration $DURATION \
    > $LOG_DIR/isaac_sim.log 2>&1 &
ISAAC_PID=$!
echo "Isaac PID: $ISAAC_PID"

# Wait for Isaac to be ready (recording dir appears + pose file)
echo "Waiting for Isaac Sim to initialize..."
for i in $(seq 1 120); do
    if [ -f /tmp/isaac_pose.txt ]; then
        echo "  Isaac pose file ready after ${i}s"
        break
    fi
    sleep 1
done
if [ ! -f /tmp/isaac_pose.txt ]; then
    echo "ERROR: Isaac Sim failed to start (no pose file after 120s)"
    kill $ISAAC_PID 2>/dev/null
    exit 1
fi

# Find the recording directory
sleep 5  # wait for first camera frame
REC_DIR=$(ls -dt /root/bags/husky_real/isaac_slam_* 2>/dev/null | head -1)
if [ -z "$REC_DIR" ]; then
    echo "ERROR: No recording directory found"
    kill $ISAAC_PID 2>/dev/null
    exit 1
fi
echo "Recording dir: $REC_DIR"

# Wait for first camera frame
echo "Waiting for first camera frame..."
for i in $(seq 1 60); do
    if ls $REC_DIR/camera_rgb/*.jpg 1>/dev/null 2>&1; then
        N_RGB=$(ls $REC_DIR/camera_rgb/*.jpg | wc -l)
        echo "  Camera frames: $N_RGB after ${i}s"
        if [ $N_RGB -ge 5 ]; then
            break
        fi
    fi
    sleep 1
done

# Wait for IMU data
echo "Waiting for IMU data..."
for i in $(seq 1 30); do
    if [ -f /tmp/isaac_imu.txt ] && [ -s /tmp/isaac_imu.txt ]; then
        IMU_LINES=$(wc -l < /tmp/isaac_imu.txt)
        echo "  IMU buffer: $IMU_LINES lines after ${i}s"
        if [ $IMU_LINES -ge 20 ]; then
            break
        fi
    fi
    sleep 1
done

echo ""
echo "=== Step 2: Starting ORB-SLAM3 VIO (rgbd_inertial_live) ==="
# Copy VIO config
cp $EXP/config/vio_th160.yaml $EXP/config/vio_live.yaml 2>/dev/null || true

cd $SLAM
./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $VIO_CFG $REC_DIR \
    > $LOG_DIR/vio_slam.log 2>&1 &
VIO_PID=$!
echo "VIO PID: $VIO_PID"

# Wait for SLAM to initialize
echo "Waiting for SLAM initialization..."
for i in $(seq 1 60); do
    if [ -f /tmp/slam_status.txt ] && grep -q "ready" /tmp/slam_status.txt 2>/dev/null; then
        echo "  SLAM ready after ${i}s"
        break
    fi
    sleep 1
done

# Wait for first SLAM pose
echo "Waiting for first SLAM pose..."
for i in $(seq 1 60); do
    if [ -f /tmp/slam_pose.txt ] && [ -s /tmp/slam_pose.txt ]; then
        echo "  SLAM pose available after ${i}s"
        cat /tmp/slam_pose.txt
        break
    fi
    sleep 1
done

echo ""
echo "=== Step 3: Starting TF relay (SLAM+encoder fusion) ==="
cd $SCRIPTS
python3 tf_wall_clock_relay.py --slam-encoder \
    > $LOG_DIR/tf_relay.log 2>&1 &
TF_PID=$!
echo "TF relay PID: $TF_PID"
sleep 2

echo ""
echo "=== Step 4: Starting Nav2 ==="
cd $EXP/config
ros2 launch $EXP/config/nav2_launch.py use_sim_time:=false \
    > $LOG_DIR/nav2.log 2>&1 &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"

# Wait for Nav2 to be ready
echo "Waiting for Nav2 lifecycle..."
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q "/navigate_to_pose"; then
        echo "  Nav2 topics available after ${i}s"
        break
    fi
    sleep 2
done
sleep 5  # extra time for lifecycle transitions

echo ""
echo "=== Step 5: Starting waypoint follower ==="
export DEBUG_DIR=$DEBUG_DIR
cd $SCRIPTS
python3 send_trajectory_goals.py $WP \
    > $LOG_DIR/goals.log 2>&1 &
GOALS_PID=$!
echo "Goals PID: $GOALS_PID"

echo ""
echo "=== All processes started ==="
echo "  Isaac Sim: $ISAAC_PID"
echo "  VIO SLAM:  $VIO_PID"
echo "  TF relay:  $TF_PID"
echo "  Nav2:      $NAV2_PID"
echo "  Goals:     $GOALS_PID"
echo ""
echo "PIDs saved to $LOG_DIR/pids.txt"
echo "ISAAC_PID=$ISAAC_PID" > $LOG_DIR/pids.txt
echo "VIO_PID=$VIO_PID" >> $LOG_DIR/pids.txt
echo "TF_PID=$TF_PID" >> $LOG_DIR/pids.txt
echo "NAV2_PID=$NAV2_PID" >> $LOG_DIR/pids.txt
echo "GOALS_PID=$GOALS_PID" >> $LOG_DIR/pids.txt
echo "REC_DIR=$REC_DIR" >> $LOG_DIR/pids.txt

echo ""
echo "Monitor with:"
echo "  tail -f $LOG_DIR/goals.log"
echo "  tail -f $LOG_DIR/tf_relay.log"
echo "  tail -f $LOG_DIR/vio_slam.log"
echo "  cat /tmp/slam_pose.txt"
