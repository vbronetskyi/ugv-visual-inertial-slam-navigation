#!/bin/bash
# Convert all campus_large recordings to pinhole_euroc format
set -e

SCRIPT="/workspace/datasets/rover/scripts/rectify_t265_stereo.py"
DATA="/workspace/data/rover"
LOG="$DATA/convert_campus.log"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG"
}

log "=========================================="
log "Converting all campus_large to pinhole_euroc"
log "=========================================="

for rec in \
    campus_large_night_2024-09-24_3 \
    campus_large_winter_2024-01-27 \
    campus_large_day_2024-09-25 \
    campus_large_dusk_2024-09-24_2 \
    campus_large_night-light_2024-09-24_4 \
    campus_large_spring_2024-04-14 \
    campus_large_summer_2023-07-20 \
    campus_large_autumn_2023-11-07; do

    out_dir="$DATA/${rec}_pinhole_euroc"
    if [ -d "$out_dir/mav0/cam0/data" ] && [ "$(ls "$out_dir/mav0/cam0/data/" 2>/dev/null | wc -l)" -gt 1000 ]; then
        log "SKIP $rec (already converted: $(ls "$out_dir/mav0/cam0/data/" | wc -l) frames)"
        continue
    fi

    log "Converting $rec ..."
    python3 "$SCRIPT" "$DATA/$rec" 2>&1 | tee -a "$LOG"

    frames=$(ls "$out_dir/mav0/cam0/data/" 2>/dev/null | wc -l)
    log "Done: $rec -> $frames frames"
    log "Free space: $(df -h / | tail -1 | awk '{print $4}')"
    log ""
done

log "=========================================="
log "ALL CONVERSIONS COMPLETE"
log "Free space: $(df -h / | tail -1 | awk '{print $4}')"
log "=========================================="
