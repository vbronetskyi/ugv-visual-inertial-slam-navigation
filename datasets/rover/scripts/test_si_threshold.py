#!/usr/bin/env python3
"""test SI with relaxed BAD LOOP threshold (0.008 -> 0.1 rad)

tests two configs:
  1. Original IMU params + relaxed threshold (result_subdir: si_relaxThresh)
  2. AccWalk=0.001 + relaxed threshold (result_subdir: si_fixAW_relaxTh)
"""

import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
from run_overnight import (
    DATA_DIR, RESULTS_DIR, ORBSLAM3_DIR, VOCAB,
    evaluate_trajectory, log
)

ORIG_CONFIG = "/workspace/datasets/rover/configs/ROVER_T265_PinHole_Stereo_Inertial.yaml"
FIX_AW_CONFIG = "/workspace/datasets/rover/configs/ROVER_T265_PinHole_SI_fixAccWalk.yaml"
EXE = os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc")

RECORDINGS = [
    "campus_large_day_2024-09-25",
    "campus_large_autumn_2023-11-07",
]


def run_si(rec_name, config_path, result_subdir):
    euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
    times_file = os.path.join(euroc_dir, "times.txt")
    gt_path = os.path.join(euroc_dir, "gt_tum.txt")

    if not os.path.exists(times_file) or not os.path.exists(gt_path):
        log(f"  ERROR: missing data for {rec_name}")
        return None

    with open(times_file) as f:
        total_frames = sum(1 for _ in f)

    out_dir = os.path.join(RESULTS_DIR, rec_name, result_subdir)
    os.makedirs(out_dir, exist_ok=True)

    output_name = f"rover_{rec_name}_{result_subdir}"
    cmd = [EXE, VOCAB, config_path, euroc_dir, times_file, output_name]

    log(f"  RUN {rec_name}/{result_subdir} ({total_frames} frames)")

    env = os.environ.copy()
    env.pop("XAUTHORITY", None)
    env.pop("QT_PLUGIN_PATH", None)

    t0 = time.time()
    try:
        ret = subprocess.run(cmd, capture_output=True, text=True, timeout=3600, env=env, cwd="/tmp")
        elapsed = time.time() - t0

        log_path = os.path.join(out_dir, "orbslam3_log.txt")
        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\nRETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (first 5000) ===\n")
            f.write(ret.stdout[:5000])
            f.write("\n\n=== STDOUT (last 5000) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else "")
            f.write("\n=== STDERR ===\n")
            f.write(ret.stderr[-3000:] if len(ret.stderr) > 3000 else ret.stderr)

        log(f"  RC={ret.returncode}, elapsed={elapsed:.0f}s")

        # count accepted vs BAD loops
        bad_count = ret.stdout.count("BAD LOOP")
        loop_detected = ret.stdout.count("*Loop detected")
        good_loops = loop_detected - bad_count
        log(f"  Loops: {loop_detected} detected, {good_loops} ACCEPTED, {bad_count} BAD")

    except subprocess.TimeoutExpired:
        log(f"  TIMEOUT")
        return {"error": "timeout"}

    # find trajectory
    traj_file = None
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt"]:
        if os.path.exists(pattern):
            traj_file = pattern
            break
    if traj_file is None:
        for candidate in ["/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
            if os.path.exists(candidate):
                traj_file = candidate
                break

    if traj_file is None:
        log(f"  NO TRAJECTORY")
        return {"error": "no trajectory"}

    traj_dst = os.path.join(out_dir, "trajectory_si.txt")
    shutil.copy2(traj_file, traj_dst)

    for p in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
              "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(p):
            os.remove(p)

    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, result_subdir, rec_name,
        total_frames=total_frames, max_diff=0.5)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  RESULT: ATE={ate}m, scale={scale}")
    return result


def main():
    log("=" * 70)
    log("SI Test: Relaxed BAD LOOP threshold (0.008 -> 0.1 rad)")
    log("=" * 70)

    all_results = []

    for rec in RECORDINGS:
        log(f"\n{'='*60}")
        log(f"{rec}")
        log(f"{'='*60}")

        # test 1: original config + relaxed threshold
        r1 = run_si(rec, ORIG_CONFIG, "si_relaxThresh")
        all_results.append((rec, "si_relaxThresh", r1))

        # test 2: AccWalk=0.001 + relaxed threshold
        r2 = run_si(rec, FIX_AW_CONFIG, "si_fixAW_relaxTh")
        all_results.append((rec, "si_fixAW_relaxTh", r2))

    # summary
    log(f"\n{'='*70}")
    log("FULL COMPARISON")
    log(f"{'='*70}")
    log(f"{'Recording':<35} {'Variant':<20} {'ATE':>8} {'Scale':>8}")
    log("-" * 75)

    for rec in RECORDINGS:
        # load original
        orig_json = os.path.join(RESULTS_DIR, rec, "stereo_inertial_pinhole", "eval_results.json")
        if os.path.exists(orig_json):
            with open(orig_json) as f:
                orig = json.load(f)
            ate = orig.get("ate_sim3", {}).get("rmse")
            s = orig.get("sim3_scale")
            log(f"{rec:<35} {'original':<20} {ate:>8.2f} {s:>8.4f}")

        # load AccWalk-only fix
        aw_json = os.path.join(RESULTS_DIR, rec, "si_fixAccWalk", "eval_results.json")
        if os.path.exists(aw_json):
            with open(aw_json) as f:
                aw = json.load(f)
            ate = aw.get("ate_sim3", {}).get("rmse")
            s = aw.get("sim3_scale")
            log(f"{'':35} {'AccWalk=0.001':<20} {ate:>8.2f} {s:>8.4f}")

        # show new variants
        for _, variant, result in all_results:
            if _ != rec:
                continue
            ate = result.get("ate_sim3", {}).get("rmse", 0) if result and not result.get("error") else 0
            s = result.get("sim3_scale", 0) if result and not result.get("error") else 0
            err = result.get("error", "") if result else "FAIL"
            if ate:
                log(f"{'':35} {variant:<20} {ate:>8.2f} {s:>8.4f}")
            else:
                log(f"{'':35} {variant:<20} {'FAIL':>8} {err}")
        log("")


if __name__ == "__main__":
    main()
