#!/usr/bin/env python3
"""run ORB-SLAM3 on all ROVER recordings in 3 modes: Stereo, Stereo-Inertial, RGB-D

handles data conversion, ORB-SLAM3 execution, evaluation, and result aggregation

Usage:
  # run all 15 recordings, 3 modes each, 3 parallel:
  python3 run_rover_orbslam3.py --parallel 3

  # single recording, single mode:
  python3 run_rover_orbslam3.py --recording garden_large_day_2024-05-29_1 --mode stereo

  # convert data only (no SLAM):
  python3 run_rover_orbslam3.py --convert-only
"""

import argparse
# TODO: add --resume flag so overnight runs can be restarted without redoing done sessions
import glob
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

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
    "stereo": os.path.join(CONFIGS_DIR, "ROVER_T265_Stereo.yaml"),
    "stereo_inertial": os.path.join(CONFIGS_DIR, "ROVER_T265_Stereo_Inertial.yaml"),
    "rgbd": os.path.join(CONFIGS_DIR, "ROVER_D435i_RGBD.yaml"),
}

EXECUTABLES = {
    "stereo": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo", "stereo_euroc"),
    "stereo_inertial": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc"),
    "rgbd": os.path.join(ORBSLAM3_DIR, "Examples", "RGB-D", "rgbd_tum"),
}


def find_recordings():
    """find all ROVER recordings in DATA_DIR"""
    recordings = []
    for d in sorted(glob.glob(os.path.join(DATA_DIR, "garden_large_*"))):
        if os.path.isdir(d) and not d.endswith(("_euroc", "_rgbd")):
            if os.path.exists(os.path.join(d, "groundtruth.txt")):
                recordings.append(os.path.basename(d))
    for d in sorted(glob.glob(os.path.join(DATA_DIR, "park_*"))):
        if os.path.isdir(d) and not d.endswith(("_euroc", "_rgbd")):
            if os.path.exists(os.path.join(d, "groundtruth.txt")):
                recordings.append(os.path.basename(d))
    return recordings


def convert_recording(rec_name):
    """convert recording to EuRoC and RGB-D formats"""
    rec_dir = os.path.join(DATA_DIR, rec_name)
    euroc_dir = rec_dir + "_euroc"
    rgbd_dir = rec_dir + "_rgbd"

    results = {"name": rec_name, "euroc": False, "rgbd": False}

    # EuRoC conversion (for stereo and stereo-inertial)
    if not os.path.exists(os.path.join(euroc_dir, "times.txt")):
        print(f"  Converting {rec_name} -> EuRoC...")
        ret = subprocess.run(
            [sys.executable, os.path.join(SCRIPTS_DIR, "convert_rover_to_euroc.py"),
             rec_dir, "-o", euroc_dir],
            capture_output=True, text=True)
        if ret.returncode != 0:
            print(f"  ERROR EuRoC conversion: {ret.stderr[-500:]}")
            return results
        results["euroc"] = True
    else:
        print(f"  EuRoC already exists: {rec_name}")
        results["euroc"] = True

    # RGB-D prep (for D435i)
    if not os.path.exists(os.path.join(rgbd_dir, "associations.txt")):
        print(f"  Converting {rec_name} -> RGB-D...")
        ret = subprocess.run(
            [sys.executable, os.path.join(SCRIPTS_DIR, "prepare_rover_rgbd.py"),
             rec_dir, "-o", rgbd_dir],
            capture_output=True, text=True)
        if ret.returncode != 0:
            print(f"  ERROR RGB-D conversion: {ret.stderr[-500:]}")
            return results
        results["rgbd"] = True
    else:
        print(f"  RGB-D already exists: {rec_name}")
        results["rgbd"] = True

    return results


# === evaluation (from evaluate_4seasons.py) ===

def load_tum_trajectory(path):
    """load TUM-format trajectory: timestamp tx ty tz qx qy qz qw"""
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
    """load ORB-SLAM3 output, auto-detect ns vs s timestamps"""
    ts, pos, quat = load_tum_trajectory(path)
    if len(ts) > 0 and ts[0] > 1e15:
        ts = ts / 1e9
    return ts, pos, quat


def associate_trajectories(ts_est, ts_gt, max_diff=0.5):
    """associate by nearest timestamp"""
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
    """Umeyama Sim3/SE3 alignment"""
    n, d = src.shape
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    var_src = np.sum(src_c ** 2) / n
    H = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[d - 1, d - 1] = -1
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / var_src if with_scale else 1.0
    t = mu_dst - s * R @ mu_src
    return R, t, s


def evaluate_trajectory(traj_path, gt_path, output_dir, mode_name, rec_name,
                        total_frames=0, max_diff=0.5):
    """evaluate one ORB-SLAM3 trajectory against GT"""
    os.makedirs(output_dir, exist_ok=True)

    ts_est, pos_est, quat_est = load_orbslam_trajectory(traj_path)
    ts_gt, pos_gt, quat_gt = load_tum_trajectory(gt_path)

    if len(ts_est) < 3:
        return {"error": "Too few estimated poses", "num_estimated": len(ts_est)}

    idx_e, idx_g = associate_trajectories(ts_est, ts_gt, max_diff=max_diff)
    if len(idx_e) < 3:
        return {"error": "Too few matched pairs", "num_estimated": len(ts_est),
                "num_matched": len(idx_e)}

    pos_e, pos_g = pos_est[idx_e], pos_gt[idx_g]
    ts_matched = ts_est[idx_e]

    # sim3 alignment
    R, t, s = umeyama_alignment(pos_e, pos_g, with_scale=True)
    pos_aligned = s * (R @ pos_e.T).T + t
    ate_errors = np.linalg.norm(pos_aligned - pos_g, axis=1)

    # SE3 alignment (no scale)
    R_se3, t_se3, _ = umeyama_alignment(pos_e, pos_g, with_scale=False)
    pos_se3 = (R_se3 @ pos_e.T).T + t_se3
    ate_se3 = np.linalg.norm(pos_se3 - pos_g, axis=1)

    # rpe
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

    # save JSON
    json_path = os.path.join(output_dir, "eval_results.json")
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2)

    # plot
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    ax = axes[0]
    ax.plot(pos_g[:, 0], pos_g[:, 1], 'b-', lw=1, alpha=0.7, label='GT')
    ax.plot(pos_aligned[:, 0], pos_aligned[:, 1], 'r-', lw=1, alpha=0.7,
            label=f'{mode_name} (s={s:.3f})')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Trajectory: {rec_name}')
    ax.legend(fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    sc = ax.scatter(pos_aligned[:, 0], pos_aligned[:, 1], c=ate_errors,
                    cmap='hot', s=3, vmin=0)
    plt.colorbar(sc, ax=ax, label='ATE (m)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'ATE Heatmap (RMSE={results["ate_sim3"]["rmse"]:.2f}m)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    t_rel = ts_matched - ts_matched[0]
    ax.plot(t_rel, ate_errors, 'r-', lw=0.5, alpha=0.7)
    ax.axhline(results['ate_sim3']['rmse'], color='b', ls='--', alpha=0.5,
               label=f'RMSE={results["ate_sim3"]["rmse"]:.2f}m')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('ATE (m)')
    ax.set_title(f'{mode_name} ATE Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "trajectory_comparison.png"), dpi=120)
    plt.close()

    return results


def run_one_experiment(rec_name, mode):
    """run single ORB-SLAM3 experiment: execute + evaluate"""
    rec_dir = os.path.join(DATA_DIR, rec_name)
    euroc_dir = rec_dir + "_euroc"
    rgbd_dir = rec_dir + "_rgbd"
    out_dir = os.path.join(RESULTS_DIR, rec_name, mode)
    os.makedirs(out_dir, exist_ok=True)

    # check if already done
    eval_json = os.path.join(out_dir, "eval_results.json")
    if os.path.exists(eval_json):
        print(f"  SKIP {rec_name}/{mode} (already done)")
        with open(eval_json) as f:
            return json.load(f)

    exe = EXECUTABLES[mode]
    config = CONFIGS[mode]
    log_path = os.path.join(out_dir, "orbslam3_log.txt")

    # output name for trajectory files
    output_name = f"rover_{rec_name}_{mode}"

    if mode in ("stereo", "stereo_inertial"):
        times_file = os.path.join(euroc_dir, "times.txt")
        gt_path = os.path.join(euroc_dir, "gt_tum.txt")
        if not os.path.exists(times_file):
            return {"error": f"EuRoC data not found for {rec_name}"}

        # count frames for tracking rate
        with open(times_file) as f:
            total_frames = sum(1 for _ in f)

        cmd = [exe, VOCAB, config, euroc_dir, times_file, output_name]
        print(f"  RUN {rec_name}/{mode} ({total_frames} frames)...")

    elif mode == "rgbd":
        assoc_file = os.path.join(rgbd_dir, "associations.txt")
        gt_path = os.path.join(rgbd_dir, "gt_tum.txt")
        if not os.path.exists(assoc_file):
            return {"error": f"RGB-D data not found for {rec_name}"}

        with open(assoc_file) as f:
            total_frames = sum(1 for _ in f)

        cmd = [exe, VOCAB, config, rgbd_dir, assoc_file]
        output_name = f"rover_{rec_name}_rgbd"
        print(f"  RUN {rec_name}/{mode} ({total_frames} frames)...")

    # run ORB-SLAM3
    t0 = time.time()
    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    try:
        ret = subprocess.run(
            ["xvfb-run", "-a", "--server-args=-screen 0 1024x768x24"] + cmd,
            capture_output=True, text=True, timeout=1200, env=env,
            cwd="/tmp")
        elapsed = time.time() - t0

        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\n\n")
            f.write("=== STDOUT ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else ret.stdout)
            f.write("\n=== STDERR ===\n")
            f.write(ret.stderr[-5000:] if len(ret.stderr) > 5000 else ret.stderr)

        if ret.returncode != 0:
            print(f"  WARN {rec_name}/{mode}: exit code {ret.returncode} ({elapsed:.0f}s)")
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT {rec_name}/{mode} (>1200s)")
        return {"error": "timeout", "recording": rec_name, "mode": mode}

    # find trajectory output
    traj_file = None
    # ORB-SLAM3 writes kf_ and f_ files to CWD
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
                    f"kf_{output_name}.txt", f"f_{output_name}.txt"]:
        if os.path.exists(pattern):
            traj_file = pattern
            break

    # also check CameraTrajectory.txt (RGB-D mode default)
    if traj_file is None:
        for candidate in ["/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
            if os.path.exists(candidate):
                traj_file = candidate
                break

    if traj_file is None:
        print(f"  NO TRAJECTORY {rec_name}/{mode}")
        return {"error": "no trajectory file", "recording": rec_name, "mode": mode}

    # copy trajectory to results
    import shutil
    traj_dst = os.path.join(out_dir, f"trajectory_{mode}.txt")
    shutil.copy2(traj_file, traj_dst)

    # evaluate
    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, mode, rec_name,
        total_frames=total_frames, max_diff=0.5)

    print(f"  DONE {rec_name}/{mode}: ATE={result.get('ate_sim3', {}).get('rmse', '?')}m "
          f"({elapsed:.0f}s)")

    return result


def make_summary(all_results, output_dir):
    """generate summary across all recordings and modes"""
    os.makedirs(output_dir, exist_ok=True)

    # collect results into table
    rows = []
    for r in all_results:
        if "error" in r:
            rows.append({
                "recording": r.get("recording", "?"),
                "mode": r.get("mode", "?"),
                "ate_rmse": None,
                "error": r["error"],
            })
        else:
            rows.append({
                "recording": r["recording"],
                "mode": r["mode"],
                "ate_rmse": r["ate_sim3"]["rmse"],
                "ate_mean": r["ate_sim3"]["mean"],
                "ate_median": r["ate_sim3"]["median"],
                "scale": r["sim3_scale"],
                "matched": r["num_matched"],
            })

    # save combined JSON
    with open(os.path.join(output_dir, "all_results.json"), 'w') as f:
        json.dump(all_results, f, indent=2)

    # text summary
    with open(os.path.join(output_dir, "summary.txt"), 'w') as f:
        f.write("ROVER ORB-SLAM3 Baseline Results\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"{'Recording':<45} {'Mode':<18} {'ATE RMSE':>10} {'Scale':>8} {'Matched':>8}\n")
        f.write("-" * 95 + "\n")
        for r in sorted(rows, key=lambda x: (x["recording"], x["mode"])):
            if r.get("ate_rmse") is not None:
                f.write(f"{r['recording']:<45} {r['mode']:<18} "
                        f"{r['ate_rmse']:>10.4f} {r['scale']:>8.4f} {r['matched']:>8}\n")
            else:
                f.write(f"{r['recording']:<45} {r['mode']:<18} "
                        f"{'FAILED':>10} {'':>8} {'':>8}  ({r['error']})\n")

        # averages per mode
        f.write("\n" + "=" * 80 + "\n")
        f.write("Average ATE RMSE per mode:\n")
        for mode in ["stereo", "stereo_inertial", "rgbd"]:
            vals = [r["ate_rmse"] for r in rows
                    if r["mode"] == mode and r.get("ate_rmse") is not None]
            if vals:
                f.write(f"  {mode:<20}: {np.mean(vals):.4f}m "
                        f"(median={np.median(vals):.4f}m, n={len(vals)})\n")

    # comparison bar chart
    modes = ["stereo", "stereo_inertial", "rgbd"]
    mode_labels = ["Stereo", "Stereo-Inertial", "RGB-D"]
    recordings = sorted(set(r["recording"] for r in rows))

    fig, ax = plt.subplots(figsize=(max(16, len(recordings)), 8))
    x = np.arange(len(recordings))
    width = 0.25

    for i, (mode, label) in enumerate(zip(modes, mode_labels)):
        vals = []
        for rec in recordings:
            r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
            vals.append(r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else 0)
        ax.bar(x + i * width, vals, width, label=label, alpha=0.8)

    ax.set_xlabel('Recording')
    ax.set_ylabel('ATE RMSE (m)')
    ax.set_title('ORB-SLAM3 on ROVER: ATE RMSE Comparison')
    ax.set_xticks(x + width)
    ax.set_xticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                        for r in recordings], rotation=45, ha='right', fontsize=8)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "comparison_bar.png"), dpi=150)
    plt.close()

    # heatmap
    fig, ax = plt.subplots(figsize=(8, max(8, len(recordings) * 0.5)))
    data = np.zeros((len(recordings), len(modes)))
    for i, rec in enumerate(recordings):
        for j, mode in enumerate(modes):
            r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
            data[i, j] = r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else np.nan

    im = ax.imshow(data, cmap='RdYlGn_r', aspect='auto', vmin=0,
                   vmax=np.nanpercentile(data, 90) if not np.all(np.isnan(data)) else 5)
    ax.set_xticks(range(len(modes)))
    ax.set_xticklabels(mode_labels)
    ax.set_yticks(range(len(recordings)))
    ax.set_yticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                        for r in recordings], fontsize=8)
    for i in range(len(recordings)):
        for j in range(len(modes)):
            v = data[i, j]
            if not np.isnan(v):
                ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=7,
                        color='white' if v > np.nanmedian(data) else 'black')
    plt.colorbar(im, label='ATE RMSE (m)')
    ax.set_title('ORB-SLAM3 ATE RMSE Heatmap')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "comparison_heatmap.png"), dpi=150)
    plt.close()

    print(f"\nSummary saved to {output_dir}/")


def main():
    parser = argparse.ArgumentParser(description="Run ORB-SLAM3 on all ROVER recordings")
    parser.add_argument("--recording", "-r", default=None,
                        help="Single recording name (default: all)")
    parser.add_argument("--mode", "-m", default=None,
                        choices=["stereo", "stereo_inertial", "rgbd"],
                        help="Single mode (default: all 3)")
    parser.add_argument("--parallel", "-p", type=int, default=1,
                        help="Number of parallel ORB-SLAM3 processes (default: 1)")
    parser.add_argument("--convert-only", action="store_true",
                        help="Only convert data, don't run SLAM")
    parser.add_argument("--eval-only", action="store_true",
                        help="Only generate summary from existing results")
    args = parser.parse_args()

    # determine recordings
    if args.recording:
        recordings = [args.recording]
    else:
        recordings = find_recordings()

    if not recordings:
        print("No recordings found!")
        sys.exit(1)

    print(f"Recordings: {len(recordings)}")
    for r in recordings:
        print(f"  {r}")

    modes = [args.mode] if args.mode else ["stereo", "stereo_inertial", "rgbd"]

    # skip to summary if eval-only
    if args.eval_only:
        all_results = []
        for rec in recordings:
            for mode in modes:
                eval_json = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
                if os.path.exists(eval_json):
                    with open(eval_json) as f:
                        all_results.append(json.load(f))
        make_summary(all_results, RESULTS_DIR)
        return

    # step 1: convert all recordings
    print(f"\n{'='*60}")
    print("STEP 1: Converting data...")
    print(f"{'='*60}")
    for rec in recordings:
        convert_recording(rec)

    if args.convert_only:
        print("\nConversion complete. Exiting (--convert-only).")
        return

    #step 2: run ORB-SLAM3
    print(f"\n{'='*60}")
    print(f"STEP 2: Running ORB-SLAM3 ({len(recordings)} recordings × {len(modes)} modes)")
    print(f"{'='*60}")

    # build task list
    tasks = [(rec, mode) for rec in recordings for mode in modes]
    print(f"Total tasks: {len(tasks)}, parallel: {args.parallel}")

    all_results = []
    if args.parallel > 1:
        with ProcessPoolExecutor(max_workers=args.parallel) as executor:
            futures = {executor.submit(run_one_experiment, rec, mode): (rec, mode)
                       for rec, mode in tasks}
            for future in as_completed(futures):
                rec, mode = futures[future]
                try:
                    result = future.result()
                    all_results.append(result)
                except Exception as e:
                    print(f"  EXCEPTION {rec}/{mode}: {e}")
                    all_results.append({"error": str(e), "recording": rec, "mode": mode})
    else:
        for rec, mode in tasks:
            result = run_one_experiment(rec, mode)
            all_results.append(result)

    # step 3: summary
    print(f"\n{'='*60}")
    print("STEP 3: Generating summary...")
    print(f"{'='*60}")
    make_summary(all_results, RESULTS_DIR)

    print("\nAll done!")


if __name__ == "__main__":
    main()
