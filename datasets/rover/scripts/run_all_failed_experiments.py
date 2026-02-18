#!/usr/bin/env python3
"""batch runner for all failed ROVER ORB-SLAM3 experiments

runs:
  1. Stereo PinHole on all 15 recordings (undistort fisheye -> pinhole first)
  2. Stereo-Inertial PinHole on all 15 recordings
  3. RGB-D on 4 missing recordings

skips already-completed, designed for unattended overnight on vast.ai

Usage:
    python3 run_all_failed_experiments.py 2>&1 | tee /workspace/datasets/rover/batch_run.log
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

# all 15 recordings
ALL_RECORDINGS = [
    "garden_large_autumn_2023-12-21",
    "garden_large_day_2024-05-29_1",
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night-light_2024-05-30_2",
    "garden_large_night_2024-05-30_1",
    "garden_large_spring_2024-04-11",
    "garden_large_summer_2023-08-18",
    "garden_large_winter_2024-01-13",
    "park_autumn_2023-11-07",
    "park_day_2024-05-08",
    "park_dusk_2024-05-13_1",
    "park_night-light_2024-05-24_2",
    "park_night_2024-05-13_2",
    "park_spring_2024-04-14",
    "park_summer_2023-07-31",
]

# RGB-D recordings missing results
RGBD_MISSING = [
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night_2024-05-30_1",
    "park_day_2024-05-08",
    "park_night-light_2024-05-24_2",
]


def log(msg):
    """print timestamped log msg"""
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def has_valid_result(rec_name, mode):
    """check if valid eval_results.json exists for this experiment"""
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


# === T265 Fisheye Undistortion ===

# T265 calibration from calib_t265.yaml
CAM_LEFT_K = np.array([
    [280.4362476646957, 0.0, 434.5911290024899],
    [0.0, 279.5757903173993, 395.3741210501516],
    [0.0, 0.0, 1.0]
])
CAM_LEFT_D = np.array([[-0.011532772136434897], [0.0501515488043061],
                        [-0.05041450901368907], [0.012741893876582578]])

CAM_RIGHT_K = np.array([
    [280.311263999059, 0.0, 431.35302371548494],
    [0.0, 279.5434630904508, 388.5071222043099],
    [0.0, 0.0, 1.0]
])
CAM_RIGHT_D = np.array([[-0.011950967309164085], [0.0530642563172375],
                         [-0.049469178559530994], [0.011573768486635416]])

ORIG_SIZE = (848, 800)
HFOV = 110
OUT_SIZE = (640, 480)


def extract_timestamp(filename):
    """extract timestamp from ROVER filename"""
    name = filename.replace('.png', '')
    if '_' in name:
        parts = name.split('_')
        for p in reversed(parts):
            try:
                val = float(p)
                if val > 1e9:
                    return p
            except ValueError:
                continue
    return name


def undistort_recording(rec_name):
    """undistort T265 fisheye to pinhole, output in EuRoC format"""
    import cv2

    rec_dir = Path(DATA_DIR) / rec_name
    output_dir = Path(DATA_DIR) / f"{rec_name}_pinhole_euroc"

    if (output_dir / "times.txt").exists():
        n = sum(1 for _ in open(output_dir / "times.txt"))
        log(f"  Pinhole EuRoC already exists: {rec_name} ({n} frames)")
        return True

    left_dir = rec_dir / "realsense_T265" / "cam_left"
    right_dir = rec_dir / "realsense_T265" / "cam_right"
    imu_file = rec_dir / "realsense_T265" / "imu" / "imu.txt"
    gt_file = rec_dir / "groundtruth.txt"

    if not left_dir.exists():
        log(f"  ERROR: {left_dir} not found")
        return False

    out_w, out_h = OUT_SIZE
    fx = (out_w / 2.0) / math.tan(math.radians(HFOV / 2.0))
    new_K = np.array([[fx, 0, out_w / 2.0], [0, fx, out_h / 2.0], [0, 0, 1]], dtype=np.float64)

    R_identity = np.eye(3)
    map_left_x, map_left_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_LEFT_K, CAM_LEFT_D, R_identity, new_K, OUT_SIZE, cv2.CV_32FC1)
    map_right_x, map_right_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_RIGHT_K, CAM_RIGHT_D, R_identity, new_K, OUT_SIZE, cv2.CV_32FC1)

    cam0_dir = output_dir / "mav0" / "cam0" / "data"
    cam1_dir = output_dir / "mav0" / "cam1" / "data"
    imu_out_dir = output_dir / "mav0" / "imu0"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    imu_out_dir.mkdir(parents=True, exist_ok=True)

    left_files = sorted([f for f in os.listdir(left_dir) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(right_dir) if f.endswith('.png')])

    left_ts_map = {extract_timestamp(f): f for f in left_files}
    right_ts_map = {extract_timestamp(f): f for f in right_files}
    common_ts = sorted(set(left_ts_map.keys()) & set(right_ts_map.keys()))
    log(f"  Undistorting {rec_name}: {len(common_ts)} stereo pairs...")

    times_list = []
    t0 = time.time()
    for i, ts in enumerate(common_ts):
        ts_ns = str(int(float(ts) * 1e9))

        img_l = cv2.imread(str(left_dir / left_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        img_r = cv2.imread(str(right_dir / right_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        if img_l is None or img_r is None:
            continue

        und_l = cv2.remap(img_l, map_left_x, map_left_y, cv2.INTER_LINEAR)
        und_r = cv2.remap(img_r, map_right_x, map_right_y, cv2.INTER_LINEAR)

        cv2.imwrite(str(cam0_dir / f"{ts_ns}.png"), und_l)
        cv2.imwrite(str(cam1_dir / f"{ts_ns}.png"), und_r)
        times_list.append(ts_ns)

        if (i + 1) % 3000 == 0:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed
            eta = (len(common_ts) - i - 1) / rate
            log(f"    {i + 1}/{len(common_ts)} ({rate:.1f} fps, ETA {eta:.0f}s)")

    elapsed = time.time() - t0
    log(f"  Undistorted {len(times_list)} pairs in {elapsed:.0f}s")

    with open(output_dir / "times.txt", 'w') as f:
        for ts_ns in times_list:
            f.write(f"{ts_ns}\n")

    if imu_file.exists():
        with open(imu_file, 'r') as fin, \
             open(imu_out_dir / "data.csv", 'w', newline='') as fout:
            writer = csv.writer(fout)
            writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]",
                             "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
                             "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                             "a_RS_S_z [m s^-2]"])
            count = 0
            for row in csv.reader(fin):
                if len(row) < 7:
                    continue
                try:
                    ts_s = float(row[0])
                except ValueError:
                    continue
                ts_ns = int(ts_s * 1e9)
                ax, ay, az = float(row[1]), float(row[2]), float(row[3])
                gx, gy, gz = float(row[4]), float(row[5]), float(row[6])
                writer.writerow([ts_ns, gx, gy, gz, ax, ay, az])
                count += 1
        log(f"  IMU: {count} samples")

    if gt_file.exists():
        gt_out = output_dir / "gt_tum.txt"
        with open(gt_file, 'r') as fin, open(gt_out, 'w') as fout:
            for line in fin:
                line = line.strip()
                if line and not line.startswith('#'):
                    fout.write(line + '\n')

    return True


# === Evaluation functions ===

def load_tum_trajectory(path):
    """load TUM-format trajectory"""
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
    """umeyama Sim3/SE3 alignment"""
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
    """evaluate one ORB-SLAM3 trajectory against GT"""
    os.makedirs(output_dir, exist_ok=True)

    ts_est, pos_est, quat_est = load_orbslam_trajectory(traj_path)
    ts_gt, pos_gt, quat_gt = load_tum_trajectory(gt_path)

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

    json_path = os.path.join(output_dir, "eval_results.json")
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2)

    # plot
    try:
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
    except Exception as e:
        log(f"  Plot error: {e}")

    return results


# === ORB-SLAM3 Runner ===

def run_orbslam3(rec_name, mode):
    """run single ORB-SLAM3 experiment"""
    rec_dir = os.path.join(DATA_DIR, rec_name)
    out_dir = os.path.join(RESULTS_DIR, rec_name, mode)
    os.makedirs(out_dir, exist_ok=True)

    exe = EXECUTABLES[mode]
    config = CONFIGS[mode]

    if mode in ("stereo_pinhole", "stereo_inertial_pinhole"):
        euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
        times_file = os.path.join(euroc_dir, "times.txt")
        gt_path = os.path.join(euroc_dir, "gt_tum.txt")

        if not os.path.exists(times_file):
            log(f"  ERROR: Pinhole EuRoC not found for {rec_name}")
            return {"error": "no pinhole data", "recording": rec_name, "mode": mode}

        with open(times_file) as f:
            total_frames = sum(1 for _ in f)

        output_name = f"rover_{rec_name}_{mode}"
        cmd = [exe, VOCAB, config, euroc_dir, times_file, output_name]

    elif mode == "rgbd":
        rgbd_dir = os.path.join(DATA_DIR, f"{rec_name}_rgbd")
        assoc_file = os.path.join(rgbd_dir, "associations.txt")
        gt_path = os.path.join(rgbd_dir, "gt_tum.txt")

        if not os.path.exists(assoc_file):
            log(f"  ERROR: RGB-D data not found for {rec_name}")
            return {"error": "no rgbd data", "recording": rec_name, "mode": mode}

        with open(assoc_file) as f:
            total_frames = sum(1 for _ in f)

        output_name = f"rover_{rec_name}_rgbd"
        cmd = [exe, VOCAB, config, rgbd_dir, assoc_file]

    log(f"  RUN {rec_name}/{mode} ({total_frames} frames)")

    t0 = time.time()
    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    log_path = os.path.join(out_dir, "orbslam3_log.txt")

    try:
        ret = subprocess.run(
            ["xvfb-run", "-a", "--server-args=-screen 0 1024x768x24"] + cmd,
            capture_output=True, text=True, timeout=1800, env=env,
            cwd="/tmp")
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
            log(f"  WARN {rec_name}/{mode}: exit code {ret.returncode} ({elapsed:.0f}s)")

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log(f"  TIMEOUT {rec_name}/{mode} (>{elapsed:.0f}s)")
        return {"error": "timeout", "recording": rec_name, "mode": mode}

    # find trajectory output
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

    # copy trajectory to results
    traj_dst = os.path.join(out_dir, f"trajectory_{mode}.txt")
    shutil.copy2(traj_file, traj_dst)

    # clean up /tmp to avoid stale files for next run
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
                    "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(pattern):
            os.remove(pattern)

    # evaluate
    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, mode, rec_name,
        total_frames=total_frames, max_diff=0.5)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  DONE {rec_name}/{mode}: ATE={ate}m, scale={scale} ({elapsed:.0f}s)")

    return result


# === Summary ===

def make_summary(output_dir):
    """generate summary from all eval_results.json"""
    all_results = []
    modes_to_check = ["stereo_pinhole", "stereo_inertial_pinhole", "rgbd"]

    for rec in ALL_RECORDINGS:
        for mode in modes_to_check:
            eval_json = os.path.join(output_dir, rec, mode, "eval_results.json")
            if os.path.exists(eval_json):
                with open(eval_json) as f:
                    all_results.append(json.load(f))

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

    with open(os.path.join(output_dir, "all_results_v2.json"), 'w') as f:
        json.dump(all_results, f, indent=2)

    # text summary
    with open(os.path.join(output_dir, "summary_v2.txt"), 'w') as f:
        f.write("ROVER ORB-SLAM3 Results (Exp 1.1c: PinHole Stereo + RGB-D)\n")
        f.write("=" * 100 + "\n\n")
        f.write(f"{'Recording':<45} {'Mode':<25} {'ATE RMSE':>10} {'Scale':>8} {'Matched':>8}\n")
        f.write("-" * 100 + "\n")
        for r in sorted(rows, key=lambda x: (x["recording"], x["mode"])):
            if r.get("ate_rmse") is not None:
                f.write(f"{r['recording']:<45} {r['mode']:<25} "
                        f"{r['ate_rmse']:>10.4f} {r['scale']:>8.4f} {r['matched']:>8}\n")
            else:
                f.write(f"{r['recording']:<45} {r['mode']:<25} "
                        f"{'FAILED':>10} {'':>8} {'':>8}  ({r['error']})\n")

        f.write("\n" + "=" * 100 + "\n")
        f.write("Summary per mode:\n")
        for mode in modes_to_check:
            vals = [r["ate_rmse"] for r in rows
                    if r["mode"] == mode and r.get("ate_rmse") is not None]
            n_fail = sum(1 for r in rows if r["mode"] == mode and r.get("ate_rmse") is None)
            if vals:
                f.write(f"  {mode:<25}: mean={np.mean(vals):.4f}m, "
                        f"median={np.median(vals):.4f}m, "
                        f"success={len(vals)}/{len(vals)+n_fail}\n")
            else:
                f.write(f"  {mode:<25}: no successful runs\n")

    log(f"Summary saved to {output_dir}/summary_v2.txt")

    # comparison plots
    try:
        modes = modes_to_check
        mode_labels = ["Stereo PH", "Stereo-Inertial PH", "RGB-D"]
        recordings = sorted(set(r["recording"] for r in rows))

        # bar chart
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
        ax.set_title('ROVER ORB-SLAM3: Stereo PinHole + SI PinHole + RGB-D')
        ax.set_xticks(x + width)
        ax.set_xticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                            for r in recordings], rotation=45, ha='right', fontsize=8)
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, "comparison_bar_v2.png"), dpi=150)
        plt.close()

        # heatmap
        fig, ax = plt.subplots(figsize=(8, max(8, len(recordings) * 0.5)))
        data = np.zeros((len(recordings), len(modes)))
        for i, rec in enumerate(recordings):
            for j, mode in enumerate(modes):
                r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
                data[i, j] = r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else np.nan

        vmax = np.nanpercentile(data, 90) if not np.all(np.isnan(data)) else 5
        im = ax.imshow(data, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=vmax)
        ax.set_xticks(range(len(modes)))
        ax.set_xticklabels(mode_labels)
        ax.set_yticks(range(len(recordings)))
        ax.set_yticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                            for r in recordings], fontsize=8)
        median_val = np.nanmedian(data)
        for i in range(len(recordings)):
            for j in range(len(modes)):
                v = data[i, j]
                if not np.isnan(v):
                    ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=7,
                            color='white' if v > median_val else 'black')
                else:
                    ax.text(j, i, 'X', ha='center', va='center', fontsize=7, color='gray')
        plt.colorbar(im, label='ATE RMSE (m)')
        ax.set_title('ORB-SLAM3 ATE RMSE Heatmap (v2)')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, "comparison_heatmap_v2.png"), dpi=150)
        plt.close()

        log("Plots saved")
    except Exception as e:
        log(f"Plot error: {e}")


# === Main ===

def main():
    log("=" * 70)
    log("ROVER Batch Experiment Runner")
    log(f"  Stereo PinHole: {len(ALL_RECORDINGS)} recordings")
    log(f"  Stereo-Inertial PinHole: {len(ALL_RECORDINGS)} recordings")
    log(f"  RGB-D (missing): {len(RGBD_MISSING)} recordings")
    log("=" * 70)

    total_start = time.time()
    results_log = []

    # ========================================
    # PHASE 1: Undistort all T265 recordings
    # ========================================
    log("")
    log("PHASE 1: Undistorting T265 fisheye -> pinhole")
    log("-" * 50)

    for i, rec in enumerate(ALL_RECORDINGS):
        log(f"[{i+1}/{len(ALL_RECORDINGS)}] {rec}")
        ok = undistort_recording(rec)
        if not ok:
            log(f"  FAILED undistortion for {rec}")

    # ========================================
    # PHASE 2: Run Stereo PinHole
    # ========================================
    log("")
    log("PHASE 2: ORB-SLAM3 Stereo PinHole")
    log("-" * 50)

    for i, rec in enumerate(ALL_RECORDINGS):
        mode = "stereo_pinhole"
        if has_valid_result(rec, mode):
            log(f"[{i+1}/{len(ALL_RECORDINGS)}] SKIP {rec}/{mode} (already done)")
            continue
        log(f"[{i+1}/{len(ALL_RECORDINGS)}] {rec}/{mode}")
        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # ========================================
    # PHASE 3: Run Stereo-Inertial PinHole
    # ========================================
    log("")
    log("PHASE 3: ORB-SLAM3 Stereo-Inertial PinHole")
    log("-" * 50)

    for i, rec in enumerate(ALL_RECORDINGS):
        mode = "stereo_inertial_pinhole"
        if has_valid_result(rec, mode):
            log(f"[{i+1}/{len(ALL_RECORDINGS)}] SKIP {rec}/{mode} (already done)")
            continue
        log(f"[{i+1}/{len(ALL_RECORDINGS)}] {rec}/{mode}")
        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # ========================================
    # PHASE 4: Run RGB-D on missing recordings
    # ========================================
    log("")
    log("PHASE 4: ORB-SLAM3 RGB-D (missing recordings)")
    log("-" * 50)

    for i, rec in enumerate(RGBD_MISSING):
        mode = "rgbd"
        # remove old failed eval to allow re-run
        old_eval = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
        if os.path.exists(old_eval):
            os.remove(old_eval)

        log(f"[{i+1}/{len(RGBD_MISSING)}] {rec}/{mode}")
        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # ========================================
    # PHASE 5: Generate summary
    # ========================================
    log("")
    log("PHASE 5: Generating summary")
    log("-" * 50)
    make_summary(RESULTS_DIR)

    total_elapsed = time.time() - total_start
    log("")
    log("=" * 70)
    log(f"ALL DONE! Total time: {total_elapsed/60:.1f} min ({total_elapsed/3600:.1f} hours)")
    log("=" * 70)

    # print quick summary
    n_ok = sum(1 for r in results_log
               if r.get("ate_sim3", {}).get("rmse") is not None)
    n_fail = sum(1 for r in results_log
                 if "error" in r or r.get("ate_sim3", {}).get("rmse") is None)
    log(f"Results: {n_ok} succeeded, {n_fail} failed out of {len(results_log)} new experiments")

    for r in results_log:
        ate = r.get("ate_sim3", {}).get("rmse")
        if ate is not None:
            log(f"  {r['recording']}/{r['mode']}: ATE={ate:.3f}m")
        else:
            log(f"  {r.get('recording', '?')}/{r.get('mode', '?')}: FAILED ({r.get('error', '?')})")


if __name__ == "__main__":
    main()
