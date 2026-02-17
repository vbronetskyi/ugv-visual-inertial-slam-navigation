#!/usr/bin/env python3
"""test two new SI variants:
1. D435i IMU + T265 PinHole Stereo
2. KB8 Stereo-Inertial (original fisheye, no undistortion)

Usage:
    DISPLAY=:99 python3 test_si_variants.py
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

EXE_SI = os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc")

RECORDINGS = [
    "campus_large_day_2024-09-25",
    "campus_large_autumn_2023-11-07",
]


def run_si(rec_name, config_path, euroc_dir, result_subdir, imu_override=None):
    """Run one SI experiment"""
    times_file = os.path.join(euroc_dir, "times.txt")
    gt_path = os.path.join(euroc_dir, "gt_tum.txt")

    # for D435i IMU: temporarily swap imu0 data
    imu_backup = None
    if imu_override:
        orig_imu = os.path.join(euroc_dir, "mav0", "imu0", "data.csv")
        backup_imu = os.path.join(euroc_dir, "mav0", "imu0", "data.csv.bak_t265")
        if not os.path.exists(backup_imu):
            shutil.copy2(orig_imu, backup_imu)
        shutil.copy2(imu_override, orig_imu)
        imu_backup = backup_imu
        log(f"  Swapped IMU: {os.path.basename(imu_override)}")

    if not os.path.exists(times_file) or not os.path.exists(gt_path):
        log(f"  ERROR: missing data")
        if imu_backup:
            shutil.copy2(imu_backup, os.path.join(euroc_dir, "mav0", "imu0", "data.csv"))
        return None

    with open(times_file) as f:
        total_frames = sum(1 for _ in f)

    out_dir = os.path.join(RESULTS_DIR, rec_name, result_subdir)
    os.makedirs(out_dir, exist_ok=True)

    output_name = f"rover_{rec_name}_{result_subdir}"
    cmd = [EXE_SI, VOCAB, config_path, euroc_dir, times_file, output_name]

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
            f.write(f"CMD: {' '.join(cmd)}\nELAPSED: {elapsed:.1f}s\nRETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (first 5000) ===\n")
            f.write(ret.stdout[:5000])
            f.write("\n\n=== STDOUT (last 5000) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else "")
            f.write("\n=== STDERR ===\n")
            f.write(ret.stderr[-3000:] if len(ret.stderr) > 3000 else ret.stderr)

        bad = ret.stdout.count("BAD LOOP")
        loops = ret.stdout.count("*Loop detected")
        log(f"  RC={ret.returncode}, {elapsed:.0f}s, loops: {loops-bad} OK / {bad} BAD")

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log(f"  TIMEOUT after {elapsed:.0f}s")
        if imu_backup:
            shutil.copy2(imu_backup, os.path.join(euroc_dir, "mav0", "imu0", "data.csv"))
        return {"error": "timeout"}

    # restore original IMU
    if imu_backup:
        shutil.copy2(imu_backup, os.path.join(euroc_dir, "mav0", "imu0", "data.csv"))
        log(f"  Restored T265 IMU")

    # find trajectory
    traj_file = None
    for p in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
              "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(p):
            traj_file = p
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
    log("SI Variants Test: D435i IMU + KB8 Fisheye")
    log("=" * 70)

    d435i_config = "/workspace/datasets/rover/configs/ROVER_T265_PinHole_SI_D435iIMU.yaml"
    kb8_config = "/workspace/datasets/rover/configs/ROVER_T265_Stereo_Inertial.yaml"

    all_results = []

    for rec in RECORDINGS:
        log(f"\n{'='*60}")
        log(f"{rec}")
        log(f"{'='*60}")

        # test 1: D435i IMU + T265 PinHole
        euroc_ph = os.path.join(DATA_DIR, f"{rec}_pinhole_euroc")
        d435i_imu = os.path.join(euroc_ph, "mav0", "imu0_d435i", "data.csv")
        r1 = run_si(rec, d435i_config, euroc_ph, "si_d435i_imu", imu_override=d435i_imu)
        all_results.append((rec, "si_d435i_imu", r1))

        # test 2: KB8 SI (original fisheye + T265 IMU)
        euroc_kb8 = os.path.join(DATA_DIR, f"{rec}_fisheye_euroc")
        r2 = run_si(rec, kb8_config, euroc_kb8, "si_kb8_fisheye")
        all_results.append((rec, "si_kb8_fisheye", r2))

    # summary
    log(f"\n{'='*70}")
    log("FULL COMPARISON (all SI variants)")
    log(f"{'='*70}")
    log(f"{'Recording':<35} {'Variant':<25} {'ATE':>8} {'Scale':>8}")
    log("-" * 80)

    variants_to_show = [
        "stereo_inertial_pinhole",
        "si_d435i_imu",
        "si_kb8_fisheye",
    ]
    labels = {
        "stereo_inertial_pinhole": "Original (T265 IMU PH)",
        "si_d435i_imu": "D435i IMU + T265 PH",
        "si_kb8_fisheye": "KB8 fisheye + T265 IMU",
    }

    for rec in RECORDINGS:
        short = rec.replace("campus_large_", "CL/").replace("_2024-09-25", "").replace("_2023-11-07", "")
        for var in variants_to_show:
            f = os.path.join(RESULTS_DIR, rec, var, "eval_results.json")
            label = labels.get(var, var)
            if os.path.exists(f):
                d = json.load(open(f))
                ate = d["ate_sim3"]["rmse"]
                s = d["sim3_scale"]
                log(f"{short:<35} {label:<25} {ate:>8.2f} {s:>8.4f}")
            else:
                log(f"{short:<35} {label:<25} {'N/A':>8}")
        log("")


if __name__ == "__main__":
    main()
