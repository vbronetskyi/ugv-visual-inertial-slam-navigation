#!/bin/bash
#both baseline stacks (exp 74 stock Nav2 + exp 76 RGB-D-only) on the
#mid-range 10-15 campaign.  uses the same obstacles as our custom
#stack's canonical repeat for each route so comparison is on the same
#scene.  stage 1 = exp 74 on 10..15 sequentially with cleanup between,
#stage 2 = exp 76 same.  outputs go to
#/root/isaac_tr_datasets/<route>/baseline_{stock_nav2,rgbd_no_imu}/
#
# Usage:
#   nohup bash /workspace/simulation/isaac/scripts/common/run_baselines_10_15.sh \
#        > /tmp/baselines_10_15_all.log 2>&1 &
#
# Override timeouts / cooldown / route list:
#   TIMEOUT_LONG_S=4500 COOLDOWN_S=30 bash run_baselines_10_15.sh 10_nmid_smid 14_se_mid
set -u

ROUTES=("$@")
if [ ${#ROUTES[@]} -eq 0 ]; then
    ROUTES=(10_nmid_smid 11_nw_mid 12_ne_mid 13_cross_nws 14_se_mid 15_wmid_smid)
fi

TIMEOUT_LONG_S=${TIMEOUT_LONG_S:-6000}
COOLDOWN_S=${COOLDOWN_S:-20}

EXP74_RUNALL=/workspace/simulation/isaac/experiments/74_pure_stock_nav2_baseline/scripts/run_all.sh
EXP76_RUNALL=/workspace/simulation/isaac/experiments/76_rgbd_no_imu_ours/scripts/run_all.sh

SUM=/tmp/run_baselines_10_15_summary.txt
: > "$SUM"

BANNER () { echo "================================================================"; }
STAMP () { date '+%F %T'; }
say () { echo "$@" | tee -a "$SUM"; }

BANNER | tee -a "$SUM"
say "RUN_BASELINES_10_15   started $(STAMP)"
say "routes:         ${ROUTES[*]}"
say "per-route timeout: ${TIMEOUT_LONG_S}s   cooldown: ${COOLDOWN_S}s"
say "summary:        $SUM"
BANNER | tee -a "$SUM"

run_stage () {
    local TAG="$1"
    local RUN_SH="$2"
    BANNER | tee -a "$SUM"
    say "STAGE: $TAG   start $(STAMP)"
    BANNER | tee -a "$SUM"
    local T0=$(date +%s)
    TIMEOUT_LONG_S="$TIMEOUT_LONG_S" COOLDOWN_S="$COOLDOWN_S" \
        bash "$RUN_SH" "${ROUTES[@]}"
    local RC=$?
    local DUR=$(( $(date +%s) - T0 ))
    say "STAGE $TAG finished in ${DUR}s with exit=$RC"
}

run_stage "exp_74_stock_nav2" "$EXP74_RUNALL"
run_stage "exp_76_rgbd_no_imu" "$EXP76_RUNALL"

BANNER | tee -a "$SUM"
say "RUN_BASELINES_10_15   done $(STAMP)"
BANNER | tee -a "$SUM"

# Final roll-up of per-route status (read from each experiment's own summary)
say ""
say "=== per-route status (from each run_all.sh summary) ==="
if [ -f /tmp/run_all_baseline_74_summary.txt ]; then
    say "--- exp 74 stock Nav2 ---"
    cat /tmp/run_all_baseline_74_summary.txt | tee -a "$SUM"
fi
if [ -f /tmp/run_all_baseline_76_summary.txt ]; then
    say "--- exp 76 RGB-D only ---"
    cat /tmp/run_all_baseline_76_summary.txt | tee -a "$SUM"
fi
