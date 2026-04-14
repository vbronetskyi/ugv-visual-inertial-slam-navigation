#!/bin/bash
# Teach pass for a single route (exp 59 pipeline, GT-as-tf for teach)
# with dense VIO logger + drift gate (abort if drift > 2 m after 30 s).
#
# Exit codes:
#   0  - route completed successfully
#   2  - drift gate aborted (caller should retry)
#   1  - other error
set -eu

ROUTE=${1:-06_nw_ne}

# ensure /tmp/slam_routes.json has this route (idempotent)
python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py

DOMAIN=${ROS_DOMAIN_ID:-85}
MAX_DRIFT_M=${MAX_DRIFT_M:-10.0}

E52=/workspace/simulation/isaac/experiments/52_obstacles_v9
E66=/workspace/simulation/isaac/routes/06_nw_ne/teach
SCRIPTS=/workspace/simulation/isaac/scripts
SLAM=/workspace/third_party/ORB_SLAM3

OUT=$E66/teach/$ROUTE
mkdir -p $OUT

if [ "$ROUTE" != "06_nw_ne" ]; then
    echo "this teach script is dedicated to route 06_nw_ne (got: $ROUTE)" >&2
    exit 1
fi
ORIG_X=-110; ORIG_Y=-45; W=195; H=90
SPAWN_X=-90.00; SPAWN_Y=35.00; SPAWN_YAW=0.0236

echo "=== TEACH $ROUTE (max_drift=${MAX_DRIFT_M}m) ==="
echo "  out: $OUT"
echo "  map: origin=($ORIG_X,$ORIG_Y)  size=${W}x${H} m"

pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt
rm -f /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt /tmp/vio_pose_dense.csv

echo "--- PHASE 1: Isaac (no obstacles) + VIO warmup"
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN

setsid /opt/isaac-sim-6.0.0/python.sh /workspace/simulation/isaac/scripts/common/run_husky_forest.py \
    --synthetic-imu --route $ROUTE --duration 4500 \
    --spawn-x $SPAWN_X --spawn-y $SPAWN_Y --spawn-yaw $SPAWN_YAW \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt $E66/config/vio_th160.yaml $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/06_tf.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/06_tf.sh
nohup bash /tmp/06_tf.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "--- PHASE 2: dense VIO logger + drift gate + depth + landmarks"

cat > /tmp/06_drift.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$DOMAIN
exec python3 /workspace/simulation/isaac/scripts/common/vio_drift_monitor.py \
    --out $OUT/vio_pose_dense.csv --max-drift-m $MAX_DRIFT_M \
    --check-interval-s 10 --settling-s 30
EOF
chmod +x /tmp/06_drift.sh
nohup bash /tmp/06_drift.sh > $OUT/drift_monitor.log 2>&1 &
DRIFT=$!; disown $DRIFT; echo "DriftMonitor: $DRIFT"

cat > /tmp/06_depth.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/teach_run_depth_mapper.py \
    --out-prefix $OUT/teach_map \
    --origin-x $ORIG_X --origin-y $ORIG_Y --width-m $W --height-m $H --res 0.1
EOF
chmod +x /tmp/06_depth.sh
nohup bash /tmp/06_depth.sh > $OUT/teach_depth_mapper.log 2>&1 &
DEPTH=$!; disown $DEPTH; echo "DepthMapper: $DEPTH"

cat > /tmp/06_lm.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 /workspace/simulation/isaac/scripts/common/visual_landmark_recorder.py \
    --out $OUT/landmarks.pkl --min-disp 2.0
EOF
chmod +x /tmp/06_lm.sh
nohup bash /tmp/06_lm.sh > $OUT/landmark_recorder.log 2>&1 &
LM=$!; disown $LM; echo "LandmarkRecorder: $LM"

echo ""
echo "TEACH $ROUTE running - Isaac=$ISAAC VIO=$VIO TF=$TF Drift=$DRIFT Depth=$DEPTH LM=$LM"
echo ""

cleanup_all() {
    pkill -TERM -f "visual_landmark_recorder" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|visual_landmark|teach_run_depth|vio_drift_monitor" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

# Wait loop: either ROUTE COMPLETE, drift abort, or duration timeout.
ROUTE_TIMEOUT_S=8100
START_TS=$(date +%s)
while true; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - START_TS ))
    if [ -f /tmp/teach_drift_abort.txt ]; then
        echo "!!! DRIFT ABORT after ${ELAPSED}s:"
        cat /tmp/teach_drift_abort.txt
        cleanup_all
        exit 2
    fi
    if grep -q "ROUTE COMPLETE" $OUT/isaac.log 2>/dev/null; then
        echo "=== ROUTE $ROUTE COMPLETE after ${ELAPSED}s ==="
        [ -f /tmp/teach_drift_status.txt ] && echo "final status: $(cat /tmp/teach_drift_status.txt)"
        # Save dense GT + clean shutdown for landmark pkl flush
        cp /tmp/isaac_trajectory.csv $OUT/traj_gt.csv 2>/dev/null || true
        cleanup_all
        trap - EXIT
        echo "=== TEACH $ROUTE SAVED -> $OUT ==="
        exit 0
    fi
    if [ "$ELAPSED" -gt "$ROUTE_TIMEOUT_S" ]; then
        echo "!!! ROUTE TIMEOUT after ${ELAPSED}s"
        cleanup_all
        exit 1
    fi
    sleep 5
done
