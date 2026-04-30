#!/bin/bash
#orchestrator for our custom T&R repeat across many routes, defaults to
#10..15 (the newly-taught batch).  cleans processes between runs.
#`bash run_repeat_all_ours.sh` for the default, or pass route ids as
#args to subset
set -u

DEFAULT=(10_nmid_smid 11_nw_mid 12_ne_mid 13_cross_nws 14_se_mid 15_wmid_smid)
ROUTES=("$@")
[ ${#ROUTES[@]} -eq 0 ] && ROUTES=("${DEFAULT[@]}")

COOLDOWN_S=${COOLDOWN_S:-20}
ROUTE_TIMEOUT_S=${ROUTE_TIMEOUT_S:-5400}
USE_OBSTACLES=${USE_OBSTACLES:-0}

SUMMARY=/tmp/run_repeat_all_ours_summary.txt
: > "$SUMMARY"

BANNER () { echo "================================================================"; }
STAMP () { date '+%F %T'; }

BANNER
echo "RUN_REPEAT_ALL_OURS   started $(STAMP)"
echo "routes: ${ROUTES[*]}"
echo "timeout per route: ${ROUTE_TIMEOUT_S}s   cooldown: ${COOLDOWN_S}s   obstacles=${USE_OBSTACLES}"
echo "summary: $SUMMARY"
BANNER

cleanup_all () {
    echo "  [cleanup] pkill -9 pipeline procs"
    pkill -9 -f "run_husky_forest|rgbd_inertial|rgbd_live|tf_wall|visual_landmark|teach_run_depth|pure_pursuit|send_goals|vio_drift_monitor|ros2 launch|isaac-sim|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager|costmap_snapshotter|turnaround_supervisor|plan_logger" 2>/dev/null
    sleep 3
    rm -rf /dev/shm/fastrtps_* 2>/dev/null
    rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt /tmp/isaac_pose.txt /tmp/isaac_imu.txt \
          /tmp/isaac_clear_route.txt /tmp/isaac_remove_obstacles.txt \
          /tmp/ro_*.sh /tmp/isaac_trajectory.csv 2>/dev/null
    echo "  [cleanup] done (remaining: $(pgrep -f 'run_husky_forest|rgbd_inertial|isaac-sim' 2>/dev/null | wc -l))"
}

for R in "${ROUTES[@]}"; do
    cleanup_all
    sleep "$COOLDOWN_S"
    OUT_LOG=/root/isaac_tr_datasets/$R/repeat/results/repeat_run/_orchestrator.log
    mkdir -p "$(dirname "$OUT_LOG")"
    BANNER
    echo "=== REPEAT_OURS $R  start $(STAMP) ==="
    BANNER
    T0=$(date +%s)
    ROUTE="$R" ROUTE_TIMEOUT_S="$ROUTE_TIMEOUT_S" USE_OBSTACLES="$USE_OBSTACLES" \
        setsid --wait bash -c "set -o pipefail; bash /workspace/simulation/isaac/scripts/common/run_repeat_ours.sh 2>&1 | tee '$OUT_LOG'"
    RC=$?
    DUR=$(( $(date +%s) - T0 ))
    case $RC in
      0) STATUS="OK" ;;
      *) STATUS="FAIL(exit=$RC)" ;;
    esac
    printf "%-20s %-16s %5ds  exit=%d  %s\n" "$R" "$STATUS" "$DUR" "$RC" "$OUT_LOG" | tee -a "$SUMMARY"
done

cleanup_all
BANNER
echo "RUN_REPEAT_ALL_OURS   done $(STAMP)"
cat "$SUMMARY"
BANNER
