#!/bin/bash
# Re-create pinhole_euroc for garden_large and park (accidentally deleted)
set -e

SCRIPT="/workspace/datasets/rover/scripts/rectify_t265_stereo.py"
DATA="/workspace/data/rover"
LOG="$DATA/reconvert.log"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG"; }

log "Re-creating pinhole_euroc for garden_large + park"

for rec in \
    garden_large_autumn_2023-12-21 \
    garden_large_day_2024-05-29_1 \
    garden_large_dusk_2024-05-29_2 \
    garden_large_night-light_2024-05-30_2 \
    garden_large_night_2024-05-30_1 \
    garden_large_spring_2024-04-11 \
    garden_large_summer_2023-08-18 \
    garden_large_winter_2024-01-13 \
    park_autumn_2023-11-07 \
    park_day_2024-05-08 \
    park_dusk_2024-05-13_1 \
    park_night-light_2024-05-24_2 \
    park_night_2024-05-13_2 \
    park_spring_2024-04-14 \
    park_summer_2023-07-31; do

    out_dir="$DATA/${rec}_pinhole_euroc"
    if [ -d "$out_dir/mav0/cam0/data" ] && [ "$(ls "$out_dir/mav0/cam0/data/" 2>/dev/null | wc -l)" -gt 1000 ]; then
        log "SKIP $rec (already exists)"
        continue
    fi

    log "Converting $rec ..."
    python3 "$SCRIPT" "$DATA/$rec" 2>&1 | tee -a "$LOG"
    frames=$(ls "$out_dir/mav0/cam0/data/" 2>/dev/null | wc -l)
    log "Done: $rec -> $frames frames"
done

log "ALL DONE. Free space: $(df -h / | tail -1 | awk '{print $4}')"
