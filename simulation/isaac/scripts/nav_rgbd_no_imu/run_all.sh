#!/bin/bash
# Orchestrator for exp 76 (our pipeline with RGB-D only SLAM) across all 9 routes.
#
# Usage:
#   bash scripts/run_all.sh                   # all 9 routes
#   bash scripts/run_all.sh 04_nw_se 09_se_ne # subset
#
# Per route: cleanup -> run scripts/run.sh with ROUTE env -> 2h40 timeout ->
# append OK/TIMEOUT/FAIL to summary.  Results in results/run_<ROUTE>/.
set -u

EXP_DIR=/workspace/simulation/isaac/experiments/76_rgbd_no_imu_ours
RUN_SH=$EXP_DIR/scripts/run.sh
[ -x "$RUN_SH" ] || { echo "ERROR: $RUN_SH not executable"; exit 2; }

# shellcheck disable=SC1091
source /workspace/simulation/isaac/experiments/_baselines_common/route_params.sh
if [ "$#" -gt 0 ]; then
    ROUTES=( "$@" )
else
    ROUTES=( "${ALL_ROUTES[@]}" )
fi

SHORT_ROUTES=(08_nw_sw 09_se_ne)
TIMEOUT_SHORT_S=${TIMEOUT_SHORT_S:-3600}   # 60 min for short routes (08, 09)
TIMEOUT_LONG_S=${TIMEOUT_LONG_S:-6000}     # 100 min for everything else
COOLDOWN_S=${COOLDOWN_S:-15}
KILL_PATTERN='run_husky_forest|rgbd_inertial|rgbd_live|tf_wall_clock|visual_landmark|teach_run_depth|pure_pursuit|send_goals|send_goals_hybrid|waypoint_follower_client|gap_nav_teach|vio_drift_monitor|turnaround_supervisor|planner_server|map_server|controller_server|behavior_server|bt_navigator|waypoint_follower|lifecycle_manager|plan_logger|costmap_snapshotter|ros2 launch'

SUMMARY_FILE=${SUMMARY_FILE:-/tmp/run_all_baseline_76_summary.txt}
: > "$SUMMARY_FILE"

kill_all_sim() {
    echo "  [cleanup] pkill -9 baseline pipeline procs"
    pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
    sleep 3
    for attempt in 1 2 3 4 5; do
        REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
        [ "$REM" -eq 0 ] && break
        echo "  [cleanup] attempt $attempt: $REM still alive, retrying"
        pkill -9 -f "$KILL_PATTERN" 2>/dev/null || true
        sleep 3
    done
    REM=$(pgrep -fa "$KILL_PATTERN" 2>/dev/null | wc -l)
    [ "$REM" -gt 0 ] && { echo "  [cleanup] WARNING: $REM procs still alive"; pgrep -fa "$KILL_PATTERN" >&2 || true; }
    rm -rf /dev/shm/fastrtps_* 2>/dev/null || true
    rm -f /tmp/slam_pose.txt /tmp/slam_stop /tmp/slam_status.txt 2>/dev/null
    rm -f /tmp/isaac_pose.txt /tmp/isaac_imu.txt /tmp/isaac_trajectory.csv 2>/dev/null
    rm -f /tmp/isaac_remove_obstacles.txt /tmp/isaac_clear_route.txt 2>/dev/null
    rm -f /tmp/anchor_correction.csv 2>/dev/null
    echo "  [cleanup] done (remaining: $REM)"
}

RUNNER_PID=""
RUNNER_PGID=""
on_interrupt() {
    echo ""
    echo "!!! exp 76 orchestrator received signal - cleanup"
    if [ -n "$RUNNER_PID" ] && kill -0 "$RUNNER_PID" 2>/dev/null; then
        kill -TERM -- "-$RUNNER_PGID" 2>/dev/null || true
        sleep 2
        kill -KILL -- "-$RUNNER_PGID" 2>/dev/null || true
    fi
    kill_all_sim
    echo "orchestrator interrupted" | tee -a "$SUMMARY_FILE"
    exit 130
}
trap on_interrupt INT TERM

echo "================================================================"
echo "RUN_ALL_BASELINE (exp 76 RGB-D no IMU)   started $(date +'%F %T')"
echo "routes: ${ROUTES[*]}"
echo "timeout:  short (08,09) ${TIMEOUT_SHORT_S}s   long ${TIMEOUT_LONG_S}s   cooldown: ${COOLDOWN_S}s"
echo "summary: $SUMMARY_FILE"
echo "================================================================"

for R in "${ROUTES[@]}"; do
    if [ -z "${RP_ISAAC_ROUTE[$R]:-}" ]; then
        echo "!!! $R not in route_params.sh - skipping"
        echo "$R  SKIP  (unknown)" >> "$SUMMARY_FILE"
        continue
    fi
    OUT=$EXP_DIR/results/run_$R
    mkdir -p "$OUT"
    LOG=$OUT/_orchestrator.log

    echo ""
    echo "================================================================"
    echo "=== EXP 76 / $R  start $(date +'%T') ==="
    echo "================================================================"
    kill_all_sim
    sleep "$COOLDOWN_S"

    IS_SHORT=0
    for S in "${SHORT_ROUTES[@]}"; do [ "$S" = "$R" ] && IS_SHORT=1; done
    if [ "$IS_SHORT" = 1 ]; then RTO=$TIMEOUT_SHORT_S; else RTO=$TIMEOUT_LONG_S; fi
    echo "=== EXP 76 / $R  timeout=${RTO}s ==="
    START=$(date +%s)
    setsid --wait bash -c "set -o pipefail; ROUTE='$R' ROUTE_TIMEOUT_S=$RTO timeout ${RTO}s bash '$RUN_SH' 2>&1 | tee '$LOG'" &
    RUNNER_PID=$!
    RUNNER_PGID=$(ps -o pgid= -p "$RUNNER_PID" | tr -d ' ')
    set +e
    wait "$RUNNER_PID"
    CODE=$?
    set -e
    RUNNER_PID=""; RUNNER_PGID=""
    DUR=$(( $(date +%s) - START ))

    case "$CODE" in
        0)    STATUS="OK" ;;
        2)    STATUS="WATCHDOG_ABORT" ;;
        124)  STATUS="TIMEOUT" ;;
        130)  STATUS="INTERRUPTED" ;;
        *)    STATUS="FAIL(exit=$CODE)" ;;
    esac
    echo ""
    echo "=== EXP 76 / $R  $STATUS  ${DUR}s  at $(date +'%T') ==="
    printf "%-22s  %-16s  %6ds  exit=%-3s  %s\n" "$R" "$STATUS" "$DUR" "$CODE" "$LOG" >> "$SUMMARY_FILE"
done

kill_all_sim
echo ""
echo "================================================================"
echo "EXP 76 ALL RUNS DONE   $(date +'%F %T')"
echo "================================================================"
cat "$SUMMARY_FILE"
