#!/bin/bash
# ROVER overnight runner: start Xvfb, run all experiments, save log
# Usage: nohup bash scripts/run_overnight.sh &

set -e

LOG="/workspace/datasets/rover/overnight_run.log"

echo "[$(date)] Starting overnight run..." | tee "$LOG"

# Kill old Xvfb and start fresh
kill $(pgrep Xvfb) 2>/dev/null || true
sleep 1
Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
XVFB_PID=$!
export DISPLAY=:99
sleep 2

echo "[$(date)] Xvfb started (PID=$XVFB_PID, DISPLAY=:99)" | tee -a "$LOG"

# Run all experiments
DISPLAY=:99 python3 -u /workspace/datasets/rover/scripts/run_overnight.py 2>&1 | tee -a "$LOG"

EXIT_CODE=${PIPESTATUS[0]}

echo "[$(date)] Python script exited with code $EXIT_CODE" | tee -a "$LOG"

# Cleanup
kill $XVFB_PID 2>/dev/null || true

echo "[$(date)] Done! Results in /workspace/datasets/rover/results/" | tee -a "$LOG"
echo "[$(date)] Summary: /workspace/datasets/rover/results/summary_final.txt" | tee -a "$LOG"
