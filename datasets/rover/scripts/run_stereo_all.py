#!/usr/bin/env python3
"""run stereo_pinhole + stereo_inertial_pinhole on all 22 ROVER recordings

skips recordings that already have valid results
waits for reconversion to finish if needed

Prerequisites (handled by wrapper script):
    Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
    export DISPLAY=:99
"""

import sys
import os

# add scripts dir to path so we can import from run_overnight
sys.path.insert(0, os.path.dirname(__file__))

from run_overnight import (
    ALL_RECORDINGS, RESULTS_DIR, log,
    has_valid_result, run_orbslam3, make_summary
)

MODES = ["stereo_inertial_pinhole", "stereo_pinhole"]


def main():
    import time
    log("=" * 70)
    log("ROVER Stereo Batch Runner: Stereo-Inertial + Stereo PinHole")
    log(f"  Recordings: {len(ALL_RECORDINGS)}")
    log(f"  Modes: {MODES}")
    log("=" * 70)

    display = os.environ.get("DISPLAY")
    if not display:
        log("ERROR: DISPLAY not set!")
        sys.exit(1)
    log(f"DISPLAY={display}")

    total_start = time.time()
    results_log = []

    # build experiment list
    experiments = []
    for rec in ALL_RECORDINGS:
        for mode in MODES:
            if not has_valid_result(rec, mode):
                experiments.append((rec, mode))

    skip_count = len(ALL_RECORDINGS) * len(MODES) - len(experiments)
    log(f"Experiments to run: {len(experiments)} (skipping {skip_count} already done)")

    for i, (rec, mode) in enumerate(experiments):
        log(f"\n[{i+1}/{len(experiments)}] {rec}/{mode}")

        # remove old failed result
        old_eval = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
        if os.path.exists(old_eval):
            os.remove(old_eval)

        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # summary
    log("\nGenerating summary...")
    make_summary()

    total_elapsed = time.time() - total_start
    log("")
    log("=" * 70)
    log(f"ALL DONE! Total: {total_elapsed/60:.1f} min ({total_elapsed/3600:.1f} hours)")
    log("=" * 70)

    n_ok = sum(1 for r in results_log if r.get("ate_sim3", {}).get("rmse") is not None)
    n_fail = len(results_log) - n_ok
    log(f"New experiments: {n_ok} succeeded, {n_fail} failed out of {len(results_log)}")

    for r in results_log:
        ate = r.get("ate_sim3", {}).get("rmse")
        if ate is not None:
            log(f"  OK   {r['recording']}/{r['mode']}: ATE={ate:.3f}m")
        else:
            log(f"  FAIL {r.get('recording', '?')}/{r.get('mode', '?')}: {r.get('error', '?')}")


if __name__ == "__main__":
    main()
