#!/bin/bash
# Wait for current NCLT SfM (every 3rd/5th) to finish, then run with ALL frames.

CURRENT_PID=911201
echo "Waiting for NCLT SfM (PID $CURRENT_PID) to finish..."
while kill -0 $CURRENT_PID 2>/dev/null; do
    sleep 30
done
echo "Previous run finished at $(date). Starting full-frame NCLT run..."

cd /workspace/datasets/nclt
python3 scripts/run_week0_hloc.py > results/week0_hloc/allframes_run.log 2>&1
echo "Full-frame run finished at $(date)"
