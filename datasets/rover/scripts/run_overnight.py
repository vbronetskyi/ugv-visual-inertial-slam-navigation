#!/usr/bin/env python3
"""overnight batch runner for ALL ROVER ORB-SLAM3 experiments

runs on all 3 routes: garden_large (8), park (7), campus_large (7 with GT)
modes: stereo_pinhole, stereo_inertial_pinhole, rgbd
skips completed experiments, re-runs failed ones

Prerequisites (handled by wrapper script):
    kill $(pgrep Xvfb) 2>/dev/null; sleep 1
    Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &
    export DISPLAY=:99
"""

import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# === Paths ===
DATA_DIR = "/workspace/data/rover"
RESULTS_DIR = "/workspace/datasets/rover/results"
SCRIPTS_DIR = "/workspace/datasets/rover/scripts"
CONFIGS_DIR = "/workspace/datasets/rover/configs"
ORBSLAM3_DIR = "/workspace/third_party/ORB_SLAM3"
VOCAB = os.path.join(ORBSLAM3_DIR, "Vocabulary", "ORBvoc.txt")

CONFIGS = {
    "stereo_pinhole": os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo.yaml"),
    "stereo_inertial_pinhole": os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo_Inertial.yaml"),
    "rgbd": os.path.join(CONFIGS_DIR, "ROVER_D435i_RGBD.yaml"),
}

EXECUTABLES = {
    "stereo_pinhole": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo", "stereo_euroc"),
    "stereo_inertial_pinhole": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc"),
    "rgbd": os.path.join(ORBSLAM3_DIR, "Examples", "RGB-D", "rgbd_tum"),
}

# all recordings with GT (22 total)
ALL_RECORDINGS = [
    # garden_large (8)
    "garden_large_autumn_2023-12-21",
    "garden_large_day_2024-05-29_1",
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night-light_2024-05-30_2",
    "garden_large_night_2024-05-30_1",
    "garden_large_spring_2024-04-11",
    "garden_large_summer_2023-08-18",
    "garden_large_winter_2024-01-13",
    # park (7)
    "park_autumn_2023-11-07",
    "park_day_2024-05-08",
    "park_dusk_2024-05-13_1",
    "park_night-light_2024-05-24_2",
    "park_night_2024-05-13_2",
    "park_spring_2024-04-14",
    "park_summer_2023-07-31",
    # campus_large (7 with GT, night-light excluded because no GT)
    "campus_large_autumn_2023-11-07",
    "campus_large_day_2024-09-25",
    "campus_large_dusk_2024-09-24_2",
    "campus_large_night_2024-09-24_3",
    "campus_large_spring_2024-04-14",
    "campus_large_summer_2023-07-20",
    "campus_large_winter_2024-01-27",
]

ALL_MODES = ["stereo_pinhole", "stereo_inertial_pinhole", "rgbd"]


def log(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def has_valid_result(rec_name, mode):
    eval_json = os.path.join(RESULTS_DIR, rec_name, mode, "eval_results.json")
    if not os.path.exists(eval_json):
        return False
    try:
        with open(eval_json) as f:
            r = json.load(f)
        ate = r.get("ate_sim3", {}).get("rmse")
        return ate is not None and r.get("num_estimated", 0) > 100
    except Exception:
        return False


# === RGB-D Preparation ===

def prepare_rgbd(rec_name):
    """prepare RGB-D data for a recording (symlinks + associations)"""
    rec_dir = os.path.join(DATA_DIR, rec_name)
    rgbd_dir = os.path.join(DATA_DIR, f"{rec_name}_rgbd")

    if os.path.exists(os.path.join(rgbd_dir, "associations.txt")):
        n = sum(1 for _ in open(os.path.join(rgbd_dir, "associations.txt")))
        log(f"  RGB-D already exists: {rec_name} ({n} associations)")
        return True

    ret = subprocess.run(
        [sys.executable, os.path.join(SCRIPTS_DIR, "prepare_rover_rgbd.py"),
         rec_dir, "-o", rgbd_dir],
        capture_output=True, text=True)
    if ret.returncode != 0:
        log(f"  ERROR RGB-D prep {rec_name}: {ret.stderr[-300:]}")
        return False
    log(f"  RGB-D prepared: {rec_name}")
    return True


# === Evaluation ===

def load_tum_trajectory(path):
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            data.append([float(p) for p in parts[:8]])
    if not data:
        return np.array([]), np.array([]), np.array([])
    data = np.array(data)
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def load_orbslam_trajectory(path):
    ts, pos, quat = load_tum_trajectory(path)
    if len(ts) > 0 and ts[0] > 1e15:
        ts = ts / 1e9
    return ts, pos, quat


def associate_trajectories(ts_est, ts_gt, max_diff=0.5):
    idx_est, idx_gt = [], []
    gt_idx = 0
    for i, te in enumerate(ts_est):
        while gt_idx < len(ts_gt) - 1 and ts_gt[gt_idx + 1] <= te:
            gt_idx += 1
        best_j = gt_idx
        best_diff = abs(te - ts_gt[gt_idx])
        if gt_idx + 1 < len(ts_gt):
            d = abs(te - ts_gt[gt_idx + 1])
            if d < best_diff:
                best_j = gt_idx + 1
                best_diff = d
        if best_diff <= max_diff:
            idx_est.append(i)
            idx_gt.append(best_j)
    return np.array(idx_est), np.array(idx_gt)


def umeyama_alignment(src, dst, with_scale=True):
    n, d = src.shape
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    var_src = np.sum(src_c ** 2) / n
    H = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(H)
    D_mat = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D_mat[d - 1, d - 1] = -1
    R = U @ D_mat @ Vt
    s = np.trace(np.diag(S) @ D_mat) / var_src if with_scale else 1.0
    t = mu_dst - s * R @ mu_src
    return R, t, s


def evaluate_trajectory(traj_path, gt_path, output_dir, mode_name, rec_name,
                        total_frames=0, max_diff=0.5):
    os.makedirs(output_dir, exist_ok=True)
    ts_est, pos_est, _ = load_orbslam_trajectory(traj_path)
    ts_gt, pos_gt, _ = load_tum_trajectory(gt_path)

    if len(ts_est) < 3:
        result = {"error": "Too few estimated poses", "recording": rec_name,
                  "mode": mode_name, "num_estimated": int(len(ts_est))}
        with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    idx_e, idx_g = associate_trajectories(ts_est, ts_gt, max_diff=max_diff)
    if len(idx_e) < 3:
        result = {"error": "Too few matched pairs", "recording": rec_name,
                  "mode": mode_name, "num_estimated": int(len(ts_est)),
                  "num_matched": int(len(idx_e))}
        with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    pos_e, pos_g = pos_est[idx_e], pos_gt[idx_g]
    ts_matched = ts_est[idx_e]

    R, t, s = umeyama_alignment(pos_e, pos_g, with_scale=True)
    pos_aligned = s * (R @ pos_e.T).T + t
    ate_errors = np.linalg.norm(pos_aligned - pos_g, axis=1)

    R_se3, t_se3, _ = umeyama_alignment(pos_e, pos_g, with_scale=False)
    pos_se3 = (R_se3 @ pos_e.T).T + t_se3
    ate_se3 = np.linalg.norm(pos_se3 - pos_g, axis=1)

    rpe = []
    for i in range(len(pos_aligned) - 1):
        dp_est = pos_aligned[i + 1] - pos_aligned[i]
        dp_gt = pos_g[i + 1] - pos_g[i]
        rpe.append(np.linalg.norm(dp_gt - dp_est))
    rpe = np.array(rpe) if rpe else np.array([0.0])

    tracking_rate = len(ts_est) / total_frames * 100 if total_frames > 0 else -1

    results = {
        "recording": rec_name,
        "mode": mode_name,
        "num_estimated": int(len(ts_est)),
        "num_gt": int(len(ts_gt)),
        "num_matched": int(len(idx_e)),
        "tracking_rate_pct": round(tracking_rate, 1) if tracking_rate > 0 else "N/A",
        "sim3_scale": round(float(s), 6),
        "ate_sim3": {
            "rmse": round(float(np.sqrt(np.mean(ate_errors**2))), 4),
            "mean": round(float(np.mean(ate_errors)), 4),
            "median": round(float(np.median(ate_errors)), 4),
            "std": round(float(np.std(ate_errors)), 4),
            "max": round(float(np.max(ate_errors)), 4),
        },
        "ate_se3": {
            "rmse": round(float(np.sqrt(np.mean(ate_se3**2))), 4),
            "mean": round(float(np.mean(ate_se3)), 4),
            "median": round(float(np.median(ate_se3)), 4),
        },
        "rpe": {
            "rmse": round(float(np.sqrt(np.mean(rpe**2))), 4),
            "mean": round(float(np.mean(rpe)), 4),
        },
    }

    with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
        json.dump(results, f, indent=2)

    try:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        ax = axes[0]
        ax.plot(pos_g[:, 0], pos_g[:, 1], 'b-', lw=1, alpha=0.7, label='GT')
        ax.plot(pos_aligned[:, 0], pos_aligned[:, 1], 'r-', lw=1, alpha=0.7,
                label=f'{mode_name} (s={s:.3f})')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'{rec_name}'); ax.legend(fontsize=8)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        ax = axes[1]
        sc = ax.scatter(pos_aligned[:, 0], pos_aligned[:, 1], c=ate_errors,
                        cmap='hot', s=3, vmin=0)
        plt.colorbar(sc, ax=ax, label='ATE (m)')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'ATE RMSE={results["ate_sim3"]["rmse"]:.2f}m')
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        ax = axes[2]
        t_rel = ts_matched - ts_matched[0]
        ax.plot(t_rel, ate_errors, 'r-', lw=0.5, alpha=0.7)
        ax.axhline(results['ate_sim3']['rmse'], color='b', ls='--', alpha=0.5,
                    label=f'RMSE={results["ate_sim3"]["rmse"]:.2f}m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('ATE (m)')
        ax.set_title(f'{mode_name} ATE Over Time')
        ax.legend(); ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, "trajectory_comparison.png"), dpi=120)
        plt.close()
    except Exception as e:
        log(f"  Plot error: {e}")

    return results


# === ORB-SLAM3 Runner ===

def run_orbslam3(rec_name, mode):
    """run ORB-SLAM3 directly, uses pre-started Xvfb"""
    out_dir = os.path.join(RESULTS_DIR, rec_name, mode)
    os.makedirs(out_dir, exist_ok=True)

    exe = EXECUTABLES[mode]
    config = CONFIGS[mode]

    if mode in ("stereo_pinhole", "stereo_inertial_pinhole"):
        euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
        times_file = os.path.join(euroc_dir, "times.txt")
        gt_path = os.path.join(euroc_dir, "gt_tum.txt")
        if not os.path.exists(times_file):
            return {"error": "no pinhole data", "recording": rec_name, "mode": mode}
        if not os.path.exists(gt_path):
            return {"error": "no GT in pinhole_euroc", "recording": rec_name, "mode": mode}
        with open(times_file) as f:
            total_frames = sum(1 for _ in f)
        output_name = f"rover_{rec_name}_{mode}"
        cmd = [exe, VOCAB, config, euroc_dir, times_file, output_name]

    elif mode == "rgbd":
        rgbd_dir = os.path.join(DATA_DIR, f"{rec_name}_rgbd")
        assoc_file = os.path.join(rgbd_dir, "associations.txt")
        gt_path = os.path.join(rgbd_dir, "gt_tum.txt")
        if not os.path.exists(assoc_file):
            return {"error": "no rgbd data", "recording": rec_name, "mode": mode}
        if not os.path.exists(gt_path):
            return {"error": "no GT in rgbd", "recording": rec_name, "mode": mode}
        with open(assoc_file) as f:
            total_frames = sum(1 for _ in f)
        output_name = f"rover_{rec_name}_rgbd"
        cmd = [exe, VOCAB, config, rgbd_dir, assoc_file]

    log(f"  RUN {rec_name}/{mode} ({total_frames} frames)")
    log_path = os.path.join(out_dir, "orbslam3_log.txt")

    t0 = time.time()
    env = os.environ.copy()
    env.pop("XAUTHORITY", None)
    env.pop("QT_PLUGIN_PATH", None)

    try:
        ret = subprocess.run(
            cmd, capture_output=True, text=True, timeout=1800,
            env=env, cwd="/tmp")
        elapsed = time.time() - t0

        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\n")
            f.write(f"RETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (last 5000 chars) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else ret.stdout)
            f.write("\n=== STDERR (last 5000 chars) ===\n")
            f.write(ret.stderr[-5000:] if len(ret.stderr) > 5000 else ret.stderr)

        if ret.returncode != 0:
            log(f"  WARN {rec_name}/{mode}: rc={ret.returncode} ({elapsed:.0f}s)")

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log(f"  TIMEOUT {rec_name}/{mode} (>{elapsed:.0f}s)")
        return {"error": "timeout", "recording": rec_name, "mode": mode}

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
        log(f"  NO TRAJECTORY {rec_name}/{mode}")
        result = {"error": "no trajectory file", "recording": rec_name, "mode": mode}
        with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    traj_dst = os.path.join(out_dir, f"trajectory_{mode}.txt")
    shutil.copy2(traj_file, traj_dst)

    # clean up /tmp
    for p in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
              "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(p):
            os.remove(p)

    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, mode, rec_name,
        total_frames=total_frames, max_diff=0.5)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  DONE {rec_name}/{mode}: ATE={ate}m, scale={scale} ({elapsed:.0f}s)")
    return result


# === Summary ===

def make_summary():
    """generate combined summary for all 22 recordings"""
    all_results = []
    for rec in ALL_RECORDINGS:
        for mode in ALL_MODES:
            eval_json = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
            if os.path.exists(eval_json):
                with open(eval_json) as f:
                    all_results.append(json.load(f))

    rows = []
    for r in all_results:
        if "error" in r:
            rows.append({"recording": r.get("recording", "?"), "mode": r.get("mode", "?"),
                          "ate_rmse": None, "error": r["error"]})
        else:
            rows.append({"recording": r["recording"], "mode": r["mode"],
                          "ate_rmse": r["ate_sim3"]["rmse"],
                          "ate_mean": r["ate_sim3"]["mean"],
                          "ate_median": r["ate_sim3"]["median"],
                          "scale": r["sim3_scale"], "matched": r["num_matched"],
                          "tracking_rate": r.get("tracking_rate_pct", "N/A")})

    with open(os.path.join(RESULTS_DIR, "all_results_final.json"), 'w') as f:
        json.dump(all_results, f, indent=2)

    with open(os.path.join(RESULTS_DIR, "summary_final.txt"), 'w') as f:
        f.write("ROVER ORB-SLAM3 Baseline Results (All 3 Routes)\n")
        f.write("=" * 110 + "\n")
        f.write(f"garden_large: 8 recordings | park: 7 recordings | campus_large: 7 recordings\n")
        f.write(f"Modes: Stereo PinHole, Stereo-Inertial PinHole, RGB-D\n")
        f.write("=" * 110 + "\n\n")

        for route in ["garden_large", "park", "campus_large"]:
            route_rows = [r for r in sorted(rows, key=lambda x: (x["recording"], x["mode"]))
                         if r["recording"].startswith(route)]
            if not route_rows:
                continue
            f.write(f"\n--- {route.upper()} ---\n")
            f.write(f"{'Recording':<45} {'Mode':<25} {'ATE RMSE':>10} {'Scale':>8} {'Matched':>8}\n")
            f.write("-" * 100 + "\n")
            for r in route_rows:
                if r.get("ate_rmse") is not None:
                    f.write(f"{r['recording']:<45} {r['mode']:<25} "
                            f"{r['ate_rmse']:>10.4f} {r['scale']:>8.4f} {r['matched']:>8}\n")
                else:
                    f.write(f"{r['recording']:<45} {r['mode']:<25} "
                            f"{'FAILED':>10} {'':>8} {'':>8}  ({r['error']})\n")

        # per-mode summary
        f.write("\n" + "=" * 110 + "\n")
        f.write("SUMMARY PER MODE (all routes combined):\n\n")
        for mode in ALL_MODES:
            vals = [r["ate_rmse"] for r in rows if r["mode"] == mode and r.get("ate_rmse") is not None]
            n_fail = sum(1 for r in rows if r["mode"] == mode and r.get("ate_rmse") is None)
            if vals:
                f.write(f"  {mode:<25}: mean={np.mean(vals):.4f}m, "
                        f"median={np.median(vals):.4f}m, "
                        f"success={len(vals)}/{len(vals)+n_fail}\n")
            else:
                f.write(f"  {mode:<25}: no successful runs\n")

        # per-route per-mode summary
        f.write("\nSUMMARY PER ROUTE PER MODE:\n\n")
        for route in ["garden_large", "park", "campus_large"]:
            f.write(f"  {route}:\n")
            for mode in ALL_MODES:
                vals = [r["ate_rmse"] for r in rows
                        if r["recording"].startswith(route) and r["mode"] == mode
                        and r.get("ate_rmse") is not None]
                n_fail = sum(1 for r in rows
                            if r["recording"].startswith(route) and r["mode"] == mode
                            and r.get("ate_rmse") is None)
                if vals:
                    f.write(f"    {mode:<25}: mean={np.mean(vals):.4f}m, "
                            f"median={np.median(vals):.4f}m, "
                            f"success={len(vals)}/{len(vals)+n_fail}\n")
                else:
                    f.write(f"    {mode:<25}: no successful runs (0/{n_fail})\n")

    log(f"Summary saved to {RESULTS_DIR}/summary_final.txt")

    # plots
    try:
        mode_labels = ["Stereo PH", "Stereo-Inertial PH", "RGB-D"]
        recordings = sorted(set(r["recording"] for r in rows))

        # bar chart
        fig, ax = plt.subplots(figsize=(max(20, len(recordings) * 0.8), 8))
        x = np.arange(len(recordings))
        width = 0.25
        for i, (mode, label) in enumerate(zip(ALL_MODES, mode_labels)):
            vals = []
            for rec in recordings:
                r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
                vals.append(r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else 0)
            ax.bar(x + i * width, vals, width, label=label, alpha=0.8)
        ax.set_xlabel('Recording'); ax.set_ylabel('ATE RMSE (m)')
        ax.set_title('ROVER ORB-SLAM3 Baseline: All 3 Routes, 3 Modes')
        ax.set_xticks(x + width)
        ax.set_xticklabels([r.replace('garden_large_', 'GL/')
                             .replace('park_', 'P/')
                             .replace('campus_large_', 'CL/')
                            for r in recordings], rotation=45, ha='right', fontsize=7)
        ax.legend(); ax.grid(True, alpha=0.3, axis='y')
        plt.tight_layout()
        plt.savefig(os.path.join(RESULTS_DIR, "comparison_bar_final.png"), dpi=150)
        plt.close()

        # heatmap
        fig, ax = plt.subplots(figsize=(8, max(10, len(recordings) * 0.45)))
        data = np.zeros((len(recordings), len(ALL_MODES)))
        for i, rec in enumerate(recordings):
            for j, mode in enumerate(ALL_MODES):
                r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
                data[i, j] = r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else np.nan
        vmax = np.nanpercentile(data, 90) if not np.all(np.isnan(data)) else 5
        im = ax.imshow(data, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=vmax)
        ax.set_xticks(range(len(ALL_MODES))); ax.set_xticklabels(mode_labels)
        ax.set_yticks(range(len(recordings)))
        ax.set_yticklabels([r.replace('garden_large_', 'GL/')
                             .replace('park_', 'P/')
                             .replace('campus_large_', 'CL/')
                            for r in recordings], fontsize=7)
        median_val = np.nanmedian(data)
        for i in range(len(recordings)):
            for j in range(len(ALL_MODES)):
                v = data[i, j]
                if not np.isnan(v):
                    ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=6,
                            color='white' if v > median_val else 'black')
                else:
                    ax.text(j, i, 'X', ha='center', va='center', fontsize=6, color='gray')
        plt.colorbar(im, label='ATE RMSE (m)')
        ax.set_title('ORB-SLAM3 ATE RMSE Heatmap (All Routes)')
        plt.tight_layout()
        plt.savefig(os.path.join(RESULTS_DIR, "comparison_heatmap_final.png"), dpi=150)
        plt.close()
        log("Plots saved")
    except Exception as e:
        log(f"Plot error: {e}")


# === Main ===

def main():
    log("=" * 70)
    log("ROVER Overnight Batch Runner: All Routes, All Modes")
    log(f"  Recordings: {len(ALL_RECORDINGS)}")
    log(f"  Modes: {ALL_MODES}")
    log(f"  Max experiments: {len(ALL_RECORDINGS) * len(ALL_MODES)}")
    log("=" * 70)

    display = os.environ.get("DISPLAY")
    if not display:
        log("ERROR: DISPLAY not set!")
        sys.exit(1)
    log(f"DISPLAY={display}")

    total_start = time.time()
    results_log = []

    # ==============================
    # PHASE 1: Prepare RGB-D data
    # ==============================
    log("\nPHASE 1: Preparing RGB-D data for all recordings")
    log("-" * 50)
    for rec in ALL_RECORDINGS:
        prepare_rgbd(rec)

    # ==============================
    # PHASE 2: Run all experiments
    # ==============================
    log("\nPHASE 2: Running ORB-SLAM3 experiments")
    log("-" * 50)

    experiments = []
    for rec in ALL_RECORDINGS:
        for mode in ALL_MODES:
            if not has_valid_result(rec, mode):
                experiments.append((rec, mode))

    log(f"Experiments to run: {len(experiments)} "
        f"(skipping {len(ALL_RECORDINGS) * len(ALL_MODES) - len(experiments)} already done)")

    for i, (rec, mode) in enumerate(experiments):
        log(f"\n[{i+1}/{len(experiments)}] {rec}/{mode}")

        # remove old failed result
        old_eval = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
        if os.path.exists(old_eval):
            os.remove(old_eval)

        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # ==============================
    # PHASE 3: Summary
    # ==============================
    log("\nPHASE 3: Generating summary")
    log("-" * 50)
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
