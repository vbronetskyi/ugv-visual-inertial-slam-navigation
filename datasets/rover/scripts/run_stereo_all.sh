#!/bin/bash
# Wait for reconversion, then run stereo-inertial + stereo on all 22 recordings
set -e

LOG="/workspace/datasets/rover/stereo_run.log"
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG"; }

# Wait for reconversion to finish
log "Waiting for reconversion to finish..."
while pgrep -f reconvert_garden_park > /dev/null 2>&1; do
    sleep 30
done
log "Reconversion done!"

# Verify all pinhole_euroc exist
MISSING=0
for rec in \
    garden_large_autumn_2023-12-21 garden_large_day_2024-05-29_1 \
    garden_large_dusk_2024-05-29_2 garden_large_night-light_2024-05-30_2 \
    garden_large_night_2024-05-30_1 garden_large_spring_2024-04-11 \
    garden_large_summer_2023-08-18 garden_large_winter_2024-01-13 \
    park_autumn_2023-11-07 park_day_2024-05-08 park_dusk_2024-05-13_1 \
    park_night-light_2024-05-24_2 park_night_2024-05-13_2 \
    park_spring_2024-04-14 park_summer_2023-07-31 \
    campus_large_autumn_2023-11-07 campus_large_day_2024-09-25 \
    campus_large_dusk_2024-09-24_2 campus_large_night_2024-09-24_3 \
    campus_large_spring_2024-04-14 campus_large_summer_2023-07-20 \
    campus_large_winter_2024-01-27; do
    if [ ! -f "/workspace/data/rover/${rec}_pinhole_euroc/times.txt" ]; then
        log "MISSING: ${rec}_pinhole_euroc"
        MISSING=$((MISSING+1))
    fi
done
log "Missing pinhole_euroc: $MISSING"

# Start Xvfb
kill $(pgrep Xvfb) 2>/dev/null || true
sleep 1
Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
export DISPLAY=:99
sleep 2
log "Xvfb started"

# Run stereo-inertial + stereo (skip rgbd, already done)
DISPLAY=:99 python3 -u /workspace/datasets/rover/scripts/run_stereo_all.py 2>&1 | tee -a "$LOG"

# Cleanup
kill $(pgrep Xvfb) 2>/dev/null || true
log "ALL DONE"
