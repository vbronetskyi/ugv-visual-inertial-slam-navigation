#!/bin/bash
# Run RTAB-Map on recorded rosbag (offline replay)
# Plays bag and runs rtabmap_slam at the same time
# Output: rtabmap.db (3D map + trajectory)
#
# Usage: bash 05_run_rtabmap.sh

set -e
source /opt/ros/jazzy/setup.bash
source /workspace/simulation/install/setup.bash
cd /workspace/simulation

BAG=bags/route_1_clean
OUT_DIR=experiments/02_slam_comparison
DB_FILE=/tmp/rtabmap.db

echo "=== RTAB-Map Offline Replay ==="
echo "Bag: $BAG"

# clean previous db
rm -f $DB_FILE

# launch rtabmap in background
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/color/image_raw \
    depth_topic:=/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera_info \
    frame_id:=base_link \
    odom_frame_id:=odom \
    approx_sync:=true \
    approx_sync_max_interval:=0.1 \
    queue_size:=100 \
    database_path:=$DB_FILE \
    use_sim_time:=true \
    rviz:=false \
    rtabmap_viz:=false \
    2>&1 &
RTAB_PID=$!
echo "RTAB-Map PID: $RTAB_PID"

sleep 10

# play rosbag with sim time
echo "Playing rosbag..."
ros2 bag play $BAG --clock --rate 0.5 2>&1 | tail -3
echo "Playback done"

sleep 10

# kill rtabmap
kill $RTAB_PID 2>/dev/null
wait $RTAB_PID 2>/dev/null

echo ""
echo "=== Results ==="
if [ -f $DB_FILE ]; then
    ls -lh $DB_FILE
    cp $DB_FILE $OUT_DIR/rtabmap.db
    echo "Saved to $OUT_DIR/rtabmap.db"

    # export 2D map
    rtabmap-export --poses $DB_FILE 2>/dev/null | tail -5 || true
else
    echo "No database created"
fi
