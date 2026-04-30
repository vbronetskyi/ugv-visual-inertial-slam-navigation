#!/bin/bash
#overnight teach orchestrator - teaches every route in $@ (default the
#10-15 batch) sequentially.  between routes it kills all pipeline
#processes, cleans /dev/shm + /tmp, then sleeps a cooldown so Isaac
#fully releases GPU/CPU.  per-route wall-clock budget is
#ROUTE_TIMEOUT_S (default 5400 = 90 min); one retry on drift-abort
set -u

DEFAULT_ROUTES=(10_nmid_smid 11_nw_mid 12_ne_mid 13_cross_nws 14_se_mid 15_wmid_smid)
ROUTES=("$@")
[ ${#ROUTES[@]} -eq 0 ] && ROUTES=("${DEFAULT_ROUTES[@]}")

COOLDOWN_S=${COOLDOWN_S:-20}
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-5400}
RETRY_ON_DRIFT=${RETRY_ON_DRIFT:-1}

SUMMARY=/tmp/run_teach_all_summary.txt
: > "$SUMMARY"

BANNER () { echo "================================================================"; }
STAMP () { date '+%F %T'; }

BANNER
echo "RUN_TEACH_ALL   started $(STAMP)"
echo "routes: ${ROUTES[*]}"
echo "timeout per route: ${ROUTE_TIMEOUT_S}s   cooldown: ${COOLDOWN_S}s   retry-on-drift: ${RETRY_ON_DRIFT}"
echo "summary: $SUMMARY"
BANNER

cleanup_all () {
    echo "  [cleanup] pkill -9 pipeline procs"
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|ros2 launch|isaac-sim|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager|costmap_snapshotter|turnaround_supervisor|plan_logger" 2>/dev/null
    sleep 3
    rm -rf /dev/shm/fastrtps_* 2>/dev/null
    rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt \
          /tmp/teach_drift_abort.txt /tmp/teach_drift_status.txt /tmp/vio_pose_dense.csv \
          /tmp/teach_*.sh /tmp/isaac_trajectory.csv 2>/dev/null
    echo "  [cleanup] done (remaining procs: $(pgrep -f 'run_husky_forest|rgbd_inertial|isaac-sim' 2>/dev/null | wc -l))"
}

run_one () {
    local R=$1
    local ATTEMPT=$2
    local OUT_LOG=/root/isaac_tr_datasets/$R/teach/teach_outputs/_orchestrator.log
    mkdir -p "$(dirname "$OUT_LOG")"

    BANNER
    echo "=== TEACH $R  attempt=$ATTEMPT  start $(STAMP) ==="
    BANNER
    local T0=$(date +%s)

    ROUTE="$R" ROUTE_TIMEOUT_S="$ROUTE_TIMEOUT_S" \
        setsid --wait bash -c "set -o pipefail; bash /workspace/simulation/isaac/scripts/common/run_teach.sh 2>&1 | tee '$OUT_LOG'"
    local RC=$?
    local DUR=$(( $(date +%s) - T0 ))
    local STATUS=""
    case $RC in
      0) STATUS="OK" ;;
      2) STATUS="DRIFT_ABORT" ;;
      *) STATUS="FAIL(exit=$RC)" ;;
    esac
    printf "%-20s %-18s %5ds  exit=%d  %s\n" "$R" "$STATUS" "$DUR" "$RC" "$OUT_LOG" | tee -a "$SUMMARY"
    return $RC
}

for R in "${ROUTES[@]}"; do
    cleanup_all
    sleep "$COOLDOWN_S"
    run_one "$R" 1
    RC=$?
    if [ $RC -eq 2 ] && [ "$RETRY_ON_DRIFT" -ge 1 ]; then
        echo "  [retry] drift abort on $R, retrying once"
        cleanup_all
        sleep "$COOLDOWN_S"
        run_one "$R" 2 || true
    fi
done

cleanup_all
BANNER
echo "RUN_TEACH_ALL   done $(STAMP)"
cat "$SUMMARY"
BANNER
