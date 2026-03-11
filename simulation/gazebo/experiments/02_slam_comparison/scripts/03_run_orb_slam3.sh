#!/bin/bash
# Run ORB-SLAM3 RGB-D on extracted frames
# prerequisite: 01_extract_frames.py + 02_fix_timestamps.py
#
# Usage: bash 03_run_orb_slam3.sh [sparse|full]
# Default: sparse (every 3rd frame)

set -e
cd /workspace/simulation/orb_slam3_data

MODE=${1:-sparse}
if [ "$MODE" = "sparse" ]; then
    ASSOC="associations_sparse.txt"
else
    ASSOC="associations.txt"
fi

echo "=== ORB-SLAM3 RGB-D ==="
echo "Mode: $MODE"
echo "Associations: $ASSOC"
echo "Frames: $(wc -l < $ASSOC)"

# clean previous output
rm -f CameraTrajectory.txt KeyFrameTrajectory.txt

# run with 10min timeout
timeout 600 /workspace/third_party/ORB_SLAM3/Examples/RGB-D/rgbd_tum \
    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /workspace/simulation/orb_slam3_data/gazebo_d435i.yaml \
    . \
    $ASSOC \
    2>&1 | tee /tmp/orb_slam3_run.log

echo ""
echo "=== Results ==="
if [ -f CameraTrajectory.txt ]; then
    echo "Trajectory: $(wc -l < CameraTrajectory.txt) poses"
    echo "KeyFrames: $(wc -l < KeyFrameTrajectory.txt) keyframes"
    # copy to experiments dir
    cp CameraTrajectory.txt /workspace/simulation/experiments/02_slam_comparison/
    cp KeyFrameTrajectory.txt /workspace/simulation/experiments/02_slam_comparison/
    echo "Saved to experiments/02_slam_comparison/"
else
    echo "No trajectory output (tracking failed)"
fi
