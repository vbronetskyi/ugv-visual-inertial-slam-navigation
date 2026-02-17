#!/usr/bin/env python3
"""quick test: Stereo-Inertial with fixed AccWalk on selected recordings

runs ORB-SLAM3 stereo_inertial_euroc with new config that has
AccWalk=0.001 instead of 0.01239, compares with original results

Usage:
    DISPLAY=:99 python3 test_si_accwalk.py [recording_name]
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

NEW_CONFIG = "/workspace/datasets/rover/configs/ROVER_T265_PinHole_SI_fixAccWalk.yaml"
ORIG_CONFIG = "/workspace/datasets/rover/configs/ROVER_T265_PinHole_Stereo_Inertial.yaml"
EXE = os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc")

# test recordings: CL/day (best: 2.02m) + CL/autumn (7.41m)
DEFAULT_RECORDINGS = [
    "campus_large_day_2024-09-25",
    "campus_large_autumn_2023-11-07",
]


def run_si_experiment(rec_name, config_path, result_subdir):
    """Run one SI experiment and evaluate"""
    euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
    times_file = os.path.join(euroc_dir, "times.txt")
    gt_path = os.path.join(euroc_dir, "gt_tum.txt")

    if not os.path.exists(times_file):
        log(f"  ERROR: no pinhole data for {rec_name}")
        return None
    if not os.path.exists(gt_path):
        log(f"  ERROR: no GT for {rec_name}")
        return None

    with open(times_file) as f:
        total_frames = sum(1 for _ in f)

    out_dir = os.path.join(RESULTS_DIR, rec_name, result_subdir)
    os.makedirs(out_dir, exist_ok=True)

    output_name = f"rover_{rec_name}_si_test"
    cmd = [EXE, VOCAB, config_path, euroc_dir, times_file, output_name]

    log(f"  RUN {rec_name} ({total_frames} frames)")
    log(f"  Config: {os.path.basename(config_path)}")

    env = os.environ.copy()
    env.pop("XAUTHORITY", None)
    env.pop("QT_PLUGIN_PATH", None)

    t0 = time.time()
    try:
        ret = subprocess.run(
            cmd, capture_output=True, text=True, timeout=3600,
            env=env, cwd="/tmp")
        elapsed = time.time() - t0

        # save full log (not truncated!)
        log_path = os.path.join(out_dir, "orbslam3_log.txt")
        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\n")
            f.write(f"RETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (first 5000) ===\n")
            f.write(ret.stdout[:5000])
            f.write("\n\n=== STDOUT (last 5000) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else "")
            f.write("\n=== STDERR (last 5000) ===\n")
            f.write(ret.stderr[-5000:] if len(ret.stderr) > 5000 else ret.stderr)

        log(f"  RC={ret.returncode}, elapsed={elapsed:.0f}s")

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log(f"  TIMEOUT after {elapsed:.0f}s")
        return {"error": "timeout", "recording": rec_name}

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
        return {"error": "no trajectory", "recording": rec_name}

    traj_dst = os.path.join(out_dir, "trajectory_si.txt")
    shutil.copy2(traj_file, traj_dst)

    # cleanup /tmp
    for p in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
              "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(p):
            os.remove(p)

    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, "si_fixAccWalk", rec_name,
        total_frames=total_frames, max_diff=0.5)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  RESULT: ATE={ate}m, scale={scale}")
    return result


def main():
    recordings = sys.argv[1:] if len(sys.argv) > 1 else DEFAULT_RECORDINGS

    log("=" * 70)
    log("SI AccWalk Fix Test")
    log(f"  AccWalk: 0.01239 -> 0.001 (BMI055 datasheet)")
    log(f"  Freq: 264 (unchanged, verified correct)")
    log(f"  Recordings: {recordings}")
    log("=" * 70)

    display = os.environ.get("DISPLAY")
    if not display:
        log("ERROR: DISPLAY not set!")
        sys.exit(1)

    results = []
    for rec in recordings:
        log(f"\n{'='*50}")
        log(f"Testing: {rec}")
        log(f"{'='*50}")

        result = run_si_experiment(rec, NEW_CONFIG, "si_fixAccWalk")
        results.append((rec, result))

        # load original for comparison
        orig_json = os.path.join(RESULTS_DIR, rec, "stereo_inertial_pinhole", "eval_results.json")
        if os.path.exists(orig_json):
            with open(orig_json) as f:
                orig = json.load(f)
            orig_ate = orig.get("ate_sim3", {}).get("rmse", "FAIL")
            orig_scale = orig.get("sim3_scale", "?")
            log(f"  ORIGINAL: ATE={orig_ate}, scale={orig_scale}")

    # summary
    log(f"\n{'='*70}")
    log("COMPARISON SUMMARY")
    log(f"{'='*70}")
    log(f"{'Recording':<45} {'Original ATE':>12} {'New ATE':>12} {'Orig Scale':>10} {'New Scale':>10}")
    log("-" * 89)

    for rec, result in results:
        orig_json = os.path.join(RESULTS_DIR, rec, "stereo_inertial_pinhole", "eval_results.json")
        orig_ate = "?"
        orig_scale = "?"
        if os.path.exists(orig_json):
            with open(orig_json) as f:
                orig = json.load(f)
            orig_ate = orig.get("ate_sim3", {}).get("rmse")
            orig_scale = orig.get("sim3_scale")
            orig_ate = f"{orig_ate:.4f}" if orig_ate else "FAIL"
            orig_scale = f"{orig_scale:.4f}" if orig_scale else "?"

        new_ate = "?"
        new_scale = "?"
        if result and result.get("ate_sim3", {}).get("rmse"):
            new_ate = f"{result['ate_sim3']['rmse']:.4f}"
            new_scale = f"{result['sim3_scale']:.4f}"
        elif result:
            new_ate = result.get("error", "FAIL")

        log(f"{rec:<45} {orig_ate:>12} {new_ate:>12} {orig_scale:>10} {new_scale:>10}")


if __name__ == "__main__":
    main()
