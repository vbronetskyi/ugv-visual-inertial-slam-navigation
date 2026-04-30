#!/bin/bash
#generic teach pass for any route, takes $ROUTE env var and reads spawn
#from _baselines_common/route_params.sh.  outputs go to
#/root/isaac_tr_datasets/$ROUTE/teach/teach_outputs/
set -eu

ROUTE=${ROUTE:?set ROUTE (e.g. 10_nmid_smid)}
DOMAIN=${ROS_DOMAIN_ID:-85}
MAX_DRIFT_M=${MAX_DRIFT_M:-10.0}
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-5400}

SCRIPTS=/workspace/simulation/isaac/scripts/common
SLAM=/workspace/third_party/ORB_SLAM3
BASELINE_COMMON=/workspace/simulation/isaac/experiments/_baselines_common
ROUTE_TEACH_CFG=/workspace/simulation/isaac/routes/$ROUTE/teach/config

# shellcheck disable=SC1091
source "$BASELINE_COMMON/route_params.sh"
ISAAC_ROUTE_ARG="${RP_ISAAC_ROUTE[$ROUTE]:?no params for $ROUTE}"
SPAWN_ARGS="${RP_SPAWN_ARGS[$ROUTE]}"

OUT=/root/isaac_tr_datasets/$ROUTE/teach/teach_outputs
mkdir -p "$OUT"

VIO_CFG="$ROUTE_TEACH_CFG/vio_th160.yaml"
[ -f "$VIO_CFG" ] || { echo "ERROR: missing $VIO_CFG"; exit 1; }

# Map extent: full scene (same origin across all routes)
ORIG_X=-110; ORIG_Y=-45; W=195; H=90

echo "=== TEACH $ROUTE  timeout=${ROUTE_TIMEOUT_S}s  max_drift=${MAX_DRIFT_M}m ==="
echo "  out: $OUT"
echo "  spawn: $SPAWN_ARGS"

pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt \
      /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt /tmp/vio_pose_dense.csv \
      /tmp/teach_*.sh

echo "--- PHASE 1: Isaac (no obstacles) + VIO warmup"
source /tmp/ros_env.sh 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export PYTHONPATH=/workspace/simulation/isaac/scripts:${PYTHONPATH:-}

python3 /workspace/simulation/isaac/routes/_common/scripts/register_routes.py || true

setsid /opt/isaac-sim-6.0.0/python.sh $SCRIPTS/run_husky_forest.py \
    --synthetic-imu --route $ISAAC_ROUTE_ARG --duration 4500 \
    $SPAWN_ARGS \
    </dev/null > $OUT/isaac.log 2>&1 &
ISAAC=$!; disown $ISAAC; echo "Isaac: $ISAAC"

for i in $(seq 1 120); do [ -f /tmp/isaac_pose.txt ] && break; sleep 1; done
sleep 5
REC=$(ls -dt /root/bags/husky_real/isaac_slam_* | head -1)
echo "REC=$REC" > $OUT/run_info.txt
echo "ROUTE=$ROUTE" >> $OUT/run_info.txt

cd $SLAM
setsid ./Examples/RGB-D-Inertial/rgbd_inertial_live \
    Vocabulary/ORBvoc.txt "$VIO_CFG" $REC \
    </dev/null > $OUT/vio.log 2>&1 &
VIO=$!; disown $VIO; echo "VIO: $VIO"

cat > /tmp/teach_tf.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/tf_wall_clock_relay.py --use-gt
EOF
chmod +x /tmp/teach_tf.sh
nohup bash /tmp/teach_tf.sh > $OUT/tf_slam.log 2>&1 &
TF=$!; disown $TF; echo "TF(GT): $TF"

for i in $(seq 1 300); do
    N=$(head -2 /tmp/slam_pose.txt 2>/dev/null | tail -1 | grep -oP 'frames=\K\d+' || echo 0)
    [ "$N" -ge 200 ] 2>/dev/null && break
    sleep 2
done
echo "VIO warmup: $N frames"

echo "--- PHASE 2: dense VIO logger + drift gate + depth mapper + landmark recorder"

cat > /tmp/teach_drift.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$DOMAIN
exec python3 $SCRIPTS/vio_drift_monitor.py \
    --out $OUT/vio_pose_dense.csv --max-drift-m $MAX_DRIFT_M \
    --check-interval-s 10 --settling-s 30
EOF
chmod +x /tmp/teach_drift.sh
nohup bash /tmp/teach_drift.sh > $OUT/drift_monitor.log 2>&1 &
DRIFT=$!; disown $DRIFT; echo "DriftMonitor: $DRIFT"

cat > /tmp/teach_depth.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/teach_run_depth_mapper.py \
    --out-prefix $OUT/teach_map \
    --origin-x $ORIG_X --origin-y $ORIG_Y --width-m $W --height-m $H --res 0.1
EOF
chmod +x /tmp/teach_depth.sh
nohup bash /tmp/teach_depth.sh > $OUT/teach_depth_mapper.log 2>&1 &
DEPTH=$!; disown $DEPTH; echo "DepthMapper: $DEPTH"

cat > /tmp/teach_lm.sh <<EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$DOMAIN
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
exec python3 $SCRIPTS/visual_landmark_recorder.py \
    --out $OUT/landmarks.pkl --min-disp 2.0
EOF
chmod +x /tmp/teach_lm.sh
nohup bash /tmp/teach_lm.sh > $OUT/landmark_recorder.log 2>&1 &
LM=$!; disown $LM; echo "LandmarkRecorder: $LM"

echo ""
echo "TEACH $ROUTE running - Isaac=$ISAAC VIO=$VIO TF=$TF Drift=$DRIFT Depth=$DEPTH LM=$LM"
echo ""

cleanup_all() {
    pkill -TERM -f "visual_landmark_recorder" 2>/dev/null || true
    sleep 3
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|vio_drift_monitor" 2>/dev/null || true
    sleep 2
}
trap cleanup_all EXIT

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
