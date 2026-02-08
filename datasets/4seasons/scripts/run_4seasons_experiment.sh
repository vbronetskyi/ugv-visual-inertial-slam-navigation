#!/bin/bash
# Run ORB-SLAM3 Stereo-Inertial on 4Seasons dataset
# Usage: bash run_4seasons_experiment.sh office_loop_1
set -e

SEQ_NAME=${1:?Usage: $0 <sequence_name> (e.g. office_loop_1)}
DATA_DIR="/workspace/data/4seasons"
SCRIPTS_DIR="/workspace/datasets/robotcar/scripts"
CONFIG="/workspace/datasets/robotcar/configs/4Seasons_Stereo_Inertial.yaml"
VOCAB="/workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt"
ORBSLAM="/workspace/third_party/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc"
RESULTS_DIR="/workspace/datasets/robotcar/results/4seasons/${SEQ_NAME}"

SEQ_DIR="${DATA_DIR}/${SEQ_NAME}"
EUROC_DIR="${DATA_DIR}/${SEQ_NAME}_euroc"

echo "4Seasons ORB-SLAM3 Stereo-Inertial Experiment, seq=${SEQ_NAME}"

# Step 0: Check stereo images exist
if [ ! -f "${SEQ_DIR}/stereo.zip" ]; then
    echo "ERROR: stereo.zip not found in ${SEQ_DIR}"
    exit 1
fi

# Step 1: Extract stereo.zip if not already extracted
REC_DIR=$(ls -d ${SEQ_DIR}/recording_* 2>/dev/null | head -1)
if [ -z "$REC_DIR" ]; then
    echo "ERROR: No recording_* directory found in ${SEQ_DIR}"
    exit 1
fi

REC_NAME=$(basename "$REC_DIR")
if [ ! -d "${REC_DIR}/undistorted_images/cam0" ] || [ -z "$(ls ${REC_DIR}/undistorted_images/cam0/*.png 2>/dev/null | head -1)" ]; then
    echo "[Step 1] Extracting stereo.zip..."
    # Extract only undistorted images (skip distorted if present)
    unzip -q -o "${SEQ_DIR}/stereo.zip" "${REC_NAME}/undistorted_images/*" -d "${SEQ_DIR}/"
    echo "  Done. $(ls ${REC_DIR}/undistorted_images/cam0/*.png 2>/dev/null | wc -l) left images extracted."
else
    echo "[Step 1] Stereo images already extracted."
    echo "  $(ls ${REC_DIR}/undistorted_images/cam0/*.png 2>/dev/null | wc -l) left images found."
fi

# Step 2: Convert to EuRoC format
echo "[Step 2] Converting to EuRoC format..."
python3 "${SCRIPTS_DIR}/convert_4seasons_to_euroc.py" "${SEQ_DIR}" --output "${EUROC_DIR}"

# Step 3: Run ORB-SLAM3 Stereo-Inertial
echo ""
echo "[Step 3] Running ORB-SLAM3 Stereo-Inertial..."
mkdir -p "${RESULTS_DIR}"
OUTPUT_NAME="4seasons_${SEQ_NAME}"

# Run with xvfb (headless display)
cd /tmp
xvfb-run -a "${ORBSLAM}" \
    "${VOCAB}" \
    "${CONFIG}" \
    "${EUROC_DIR}" \
    "${EUROC_DIR}/times.txt" \
    "${OUTPUT_NAME}" 2>&1 | tee "${RESULTS_DIR}/orbslam3_log.txt"

# Move output files to results
for f in kf_${OUTPUT_NAME}.txt f_${OUTPUT_NAME}.txt; do
    if [ -f "/tmp/${f}" ]; then
        mv "/tmp/${f}" "${RESULTS_DIR}/"
        echo "  Saved: ${RESULTS_DIR}/${f}"
    fi
done

# Step 4: Evaluate
echo ""
echo "[Step 4] Evaluating trajectory..."
TOTAL_FRAMES=$(wc -l < "${EUROC_DIR}/times.txt")
python3 "${SCRIPTS_DIR}/evaluate_4seasons.py" \
    --traj "${RESULTS_DIR}/kf_${OUTPUT_NAME}.txt" \
    --gt "${EUROC_DIR}/gt_tum.txt" \
    --output "${RESULTS_DIR}" \
    --total-frames "${TOTAL_FRAMES}"

echo ""
echo "Done, results: ${RESULTS_DIR}/"
