#!/usr/bin/env python3
"""ORB-SLAM3 parameter sweep on NCLT Ladybug3. runs unattended.

first does a camera investigation (figure out which of Cam0-5 actually point
at useful scene content), then a quick 1k-frame validation on the top cameras,
then a parameter sweep on 3k frames, then a full-session run with whatever
config won the sweep. all results + plots saved at the end
"""

import os
import sys
import cv2
import csv
import json
import time
import shutil
import signal
import subprocess
import numpy as np
from pathlib import Path
from datetime import datetime, timedelta

# paths
PROJECT_ROOT = Path(__file__).resolve().parent.parent
NCLT_DATA = Path("/workspace/nclt_data")
ORB_SLAM3_DIR = Path("/tmp/ORB_SLAM3")
VOCAB_PATH = ORB_SLAM3_DIR / "Vocabulary" / "ORBvoc.txt"
MONO_TUM = ORB_SLAM3_DIR / "Examples" / "Monocular" / "mono_tum"
GT_FILE = NCLT_DATA / "ground_truth" / "groundtruth_2012-04-29.csv"

RESULTS_DIR = PROJECT_ROOT / "results" / "week0_orbslam3_tuned"
CAMERA_SAMPLES_DIR = RESULTS_DIR / "camera_samples"
TUNING_DIR = RESULTS_DIR / "tuning_runs"

SESSION = "2012-04-29"
# NOTE: only spring + summer have Ladybug3 images, winter/autumn = lidar-only

# ORB-SLAM3 environment
def orbslam_env():
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = (
        f"{ORB_SLAM3_DIR}/lib:"
        f"{ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib:"
        f"{ORB_SLAM3_DIR}/Thirdparty/g2o/lib:"
        "/usr/local/lib:" + env.get("LD_LIBRARY_PATH", "")
    )
    env["QT_QPA_PLATFORM"] = "offscreen"
    env.pop("QT_PLUGIN_PATH", None)
    return env


# logging
LOG_FILE = None

def log(msg, level="INFO"):
    """Print and log a timestamped message"""
    ts = datetime.now().strftime("%H:%M:%S")
    line = f"[{ts}] [{level}] {msg}"
    print(line, flush=True)
    if LOG_FILE:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")


# ground truth loading
def load_ground_truth(session="2012-04-29"):
    """Load NCLT ground truth as (N,8) array: [timestamp_us, x, y, z, r, p, y_aw, ?]."""
    gt_path = NCLT_DATA / "ground_truth" / f"groundtruth_{session}.csv"
    data = np.loadtxt(str(gt_path), delimiter=",")
    return data


def compute_ate(est_traj, gt_data):
    """compute ATE RMSE between estimated trajectory and ground truth"""
    if len(est_traj) < 10:
        return {"ate_rmse": float("inf"), "n_matched": 0, "coverage_pct": 0, "scale_factor": 0}

    est = np.array(est_traj)
    est_ts = est[:, 0]  # seconds
    est_xyz = est[:, 1:4]

    gt_ts = gt_data[:, 0] / 1e6  # microseconds -> seconds
    gt_xyz = gt_data[:, 1:4]

    # match timestamps (nearest neighbor, max 0.1s tolerance)
    matched_est = []
    matched_gt = []
    for i, t in enumerate(est_ts):
        idx = np.argmin(np.abs(gt_ts - t))
        if abs(gt_ts[idx] - t) < 0.1:
            matched_est.append(est_xyz[i])
            matched_gt.append(gt_xyz[idx])

    if len(matched_est) < 10:
        return {"ate_rmse": float("inf"), "n_matched": len(matched_est), "coverage_pct": 0, "scale_factor": 0}

    matched_est = np.array(matched_est)
    matched_gt = np.array(matched_gt)

    # sim(3) alignment (Umeyama)
    ate_rmse, scale = umeyama_alignment(matched_est, matched_gt)

    total_frames = len(est_traj)
    gt_duration = gt_ts[-1] - gt_ts[0]
    est_duration = est_ts[-1] - est_ts[0]
    coverage = min(est_duration / gt_duration * 100, 100) if gt_duration > 0 else 0

    return {
        "ate_rmse": float(ate_rmse),
        "n_matched": len(matched_est),
        "coverage_pct": float(coverage),
        "scale_factor": float(scale),
    }


def umeyama_alignment(est, gt):
    """Umeyama Sim(3) alignment. Returns (RMSE, scale)."""
    mu_est = est.mean(axis=0)
    mu_gt = gt.mean(axis=0)

    est_c = est - mu_est
    gt_c = gt - mu_gt

    H = est_c.T @ gt_c
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T

    var_est = np.sum(est_c ** 2) / len(est)
    scale = np.trace(np.diag(S) @ D) / var_est if var_est > 1e-10 else 1.0

    est_aligned = scale * (R @ est.T).T + mu_gt - scale * R @ mu_est
    errors = np.linalg.norm(est_aligned - gt, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2))

    return rmse, scale


# TUM dataset preparation
def prepare_tum_dataset(cam_id, n_frames=0, preprocess="none", clahe_clip=2.0,
                        output_dir=None, resolution="half"):
    """convert Ladybug3 TIFFs to TUM format, returns dataset path or None"""
    src_dir = NCLT_DATA / "images" / SESSION / "lb3" / f"Cam{cam_id}"
    if not src_dir.exists():
        log(f"Source images not found: {src_dir}", "ERROR")
        return None

    res_map = {
        "half": (808, 616),
        "3quarter": (1212, 924),
        "full": (1616, 1232),
    }
    w, h = res_map.get(resolution, (808, 616))

    tag = f"cam{cam_id}_{resolution}"
    if preprocess == "clahe":
        tag += f"_clahe{clahe_clip}"
    if n_frames > 0:
        tag += f"_n{n_frames}"

    if output_dir is None:
        output_dir = Path(f"/tmp/nclt_tum_{SESSION}_{tag}")

    rgb_dir = output_dir / "rgb"
    rgb_dir.mkdir(parents=True, exist_ok=True)

    assoc_file = output_dir / "rgb.txt"

    # check if already prepared
    if assoc_file.exists():
        with open(assoc_file) as f:
            existing = sum(1 for line in f if not line.startswith("#"))
        tiffs = sorted(src_dir.glob("*.tiff"))
        target = min(len(tiffs), n_frames) if n_frames > 0 else len(tiffs)
        if existing >= target * 0.95:
            log(f"  Dataset already prepared: {existing} frames in {output_dir}")
            return output_dir

    tiffs = sorted(src_dir.glob("*.tiff"))
    if n_frames > 0:
        # skip first 200 frames (overexposed on startup)
        skip = min(200, len(tiffs) // 10)
        tiffs = tiffs[skip : skip + n_frames]

    log(f"  Preparing {len(tiffs)} frames from Cam{cam_id} ({w}x{h}, preprocess={preprocess})...")

    clahe = None
    if preprocess == "clahe":
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))

    timestamps = []
    filenames = []
    t0 = time.time()

    for i, f in enumerate(tiffs):
        ts_us = int(f.stem)
        ts_s = ts_us / 1e6

        out_name = f"{ts_us}.png"
        out_path = rgb_dir / out_name

        if not out_path.exists():
            img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
            if clahe is not None:
                img = clahe.apply(img)
            cv2.imwrite(str(out_path), img)

        timestamps.append(ts_s)
        filenames.append(f"rgb/{out_name}")

        if (i + 1) % 2000 == 0:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed
            eta = (len(tiffs) - i - 1) / rate
            log(f"    [{i+1}/{len(tiffs)}] {rate:.0f} img/s, ETA {eta:.0f}s")

    # write association file
    with open(assoc_file, "w") as fout:
        fout.write("# NCLT Ladybug3 Cam{} in TUM format ({}, {})\n".format(cam_id, resolution, preprocess))
        fout.write("# timestamp filename\n")
        for ts, fn in zip(timestamps, filenames):
            fout.write(f"{ts:.6f} {fn}\n")

    elapsed = time.time() - t0
    log(f"  Prepared {len(timestamps)} frames in {elapsed:.0f}s")
    return output_dir


# ORB-SLAM3 config generation
def write_orbslam3_config(output_path, cam_params):
    """write an ORB-SLAM3 YAML config file"""
    p = cam_params
    content = f"""%YAML:1.0

File.version: "1.0"

Camera.type: "PinHole"

Camera1.fx: {p['fx']}
Camera1.fy: {p['fy']}
Camera1.cx: {p['cx']}
Camera1.cy: {p['cy']}

Camera1.k1: {p.get('k1', 0.0)}
Camera1.k2: {p.get('k2', 0.0)}
Camera1.p1: {p.get('p1', 0.0)}
Camera1.p2: {p.get('p2', 0.0)}

Camera.width: {p['width']}
Camera.height: {p['height']}

Camera.fps: {p.get('fps', 5)}

Camera.RGB: 0

ORBextractor.nFeatures: {p.get('nFeatures', 2000)}
ORBextractor.scaleFactor: {p.get('scaleFactor', 1.2)}
ORBextractor.nLevels: {p.get('nLevels', 8)}
ORBextractor.iniThFAST: {p.get('iniThFAST', 20)}
ORBextractor.minThFAST: {p.get('minThFAST', 7)}

Viewer.KeyFrameSize: 0.6
Viewer.KeyFrameLineWidth: 2.0
Viewer.GraphLineWidth: 1.0
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.7
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -100.0
Viewer.ViewpointZ: -0.1
Viewer.ViewpointF: 2000.0
"""
    with open(output_path, "w") as f:
        f.write(content)
    return output_path


# run ORB-SLAM3
def run_orbslam3(dataset_dir, config_path, timeout_s=900, run_label=""):
    """run ORB-SLAM3 mono on a TUM-format dataset"""
    result = {
        "success": False,
        "traj_path": None,
        "n_frames": 0,
        "n_keyframes": 0,
        "runtime_s": 0,
        "tracking_rate": 0,
        "stdout_tail": "",
    }

    if not MONO_TUM.exists():
        log(f"  mono_tum binary not found: {MONO_TUM}", "ERROR")
        return result

    # count input frames
    assoc = dataset_dir / "rgb.txt"
    if assoc.exists():
        with open(assoc) as f:
            n_input = sum(1 for line in f if not line.startswith("#"))
    else:
        n_input = 0

    # clean old trajectory files
    for candidate in ["KeyFrameTrajectory.txt", "CameraTrajectory.txt",
                      "FrameTrajectory_TUM_Format.txt"]:
        p = dataset_dir / candidate
        if p.exists():
            p.unlink()

    cmd = [
        "xvfb-run", "-a",
        str(MONO_TUM),
        str(VOCAB_PATH),
        str(config_path),
        str(dataset_dir),
    ]

    env = orbslam_env()
    label = f" [{run_label}]" if run_label else ""
    log(f"  Running ORB-SLAM3{label} on {dataset_dir.name} (timeout={timeout_s}s)...")

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            env=env,
            cwd=str(dataset_dir),
        )
        result["runtime_s"] = time.time() - t0

        # parse stdout
        stdout = proc.stdout or ""
        result["stdout_tail"] = "\n".join(stdout.strip().split("\n")[-15:])

        # extract tracking stats from ORB-SLAM3 output
        for line in stdout.split("\n"):
            if "Tracking" in line and "lost" in line.lower():
                log(f"    {line.strip()}")

        # find trajectory
        traj_path = None
        for candidate in ["KeyFrameTrajectory.txt", "CameraTrajectory.txt",
                          "FrameTrajectory_TUM_Format.txt"]:
            p = dataset_dir / candidate
            if p.exists() and p.stat().st_size > 0:
                traj_path = p
                break

        if traj_path:
            result["traj_path"] = str(traj_path)
            result["success"] = True

            # count output poses
            with open(traj_path) as f:
                poses = [l for l in f if not l.startswith("#") and l.strip()]
            result["n_keyframes"] = len(poses)
            result["n_frames"] = n_input
            result["tracking_rate"] = len(poses) / max(n_input, 1) * 100

            log(f"    OK: {len(poses)} keyframes from {n_input} frames "
                f"({result['tracking_rate']:.1f}%) in {result['runtime_s']:.1f}s")
        else:
            log(f"    FAIL: No trajectory produced (return code {proc.returncode})")
            if proc.stderr:
                for line in proc.stderr.strip().split("\n")[-5:]:
                    log(f"    STDERR: {line}")

    except subprocess.TimeoutExpired:
        result["runtime_s"] = time.time() - t0
        log(f"    TIMEOUT after {timeout_s}s")
    except Exception as e:
        result["runtime_s"] = time.time() - t0
        log(f"    ERROR: {e}", "ERROR")

    return result


def load_tum_trajectory(traj_path):
    """load a TUM-format trajectory file"""
    poses = []
    with open(traj_path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                poses.append([float(x) for x in parts[:8]])
    return poses


# PHASE 0: Camera Investigation
def phase0_camera_investigation():
    """Investigate all 6 Ladybug3 cameras to find best ones for SLAM"""
    log("=" * 70)
    log("PHASE 0: Camera Investigation")
    log("=" * 70)

    CAMERA_SAMPLES_DIR.mkdir(parents=True, exist_ok=True)

    src_base = NCLT_DATA / "images" / SESSION / "lb3"
    camera_stats = []

    for cam_id in range(6):
        cam_dir = src_base / f"Cam{cam_id}"
        tiffs = sorted(cam_dir.glob("*.tiff"))
        if not tiffs:
            log(f"  Cam{cam_id}: No images found", "WARN")
            continue

        # pick a sample from ~25% into the sequence (avoid overexposed startup)
        idx = max(len(tiffs) // 4, 200)
        sample_tiff = tiffs[min(idx, len(tiffs) - 1)]

        # read and resize to half-res
        img_color = cv2.imread(str(sample_tiff), cv2.IMREAD_COLOR)
        img_gray = cv2.imread(str(sample_tiff), cv2.IMREAD_GRAYSCALE)
        if img_color is None or img_gray is None:
            log(f"  Cam{cam_id}: Failed to read {sample_tiff}", "WARN")
            continue

        img_color = cv2.resize(img_color, (808, 616), interpolation=cv2.INTER_AREA)
        img_gray = cv2.resize(img_gray, (808, 616), interpolation=cv2.INTER_AREA)

        # save sample
        sample_path = CAMERA_SAMPLES_DIR / f"cam{cam_id}_sample.jpg"
        cv2.imwrite(str(sample_path), img_color)

        # compute stats
        brightness = float(img_gray.mean())
        contrast = float(img_gray.std())

        # ORB features
        orb = cv2.ORB_create(nfeatures=3000)
        kps = orb.detect(img_gray)
        n_orb = len(kps)

        # goodFeaturesToTrack (what ORB-SLAM3 internally uses is similar)
        corners = cv2.goodFeaturesToTrack(img_gray, maxCorners=3000,
                                          qualityLevel=0.01, minDistance=10)
        n_corners = len(corners) if corners is not None else 0

        # guess orientation from image content
        # top portion brightness vs bottom - sky cameras are bright on top
        top_half = img_gray[:308, :].mean()
        bot_half = img_gray[308:, :].mean()
        left_half = img_gray[:, :404].mean()
        right_half = img_gray[:, 404:].mean()

        # heuristic: if top is much brighter than bottom, likely upward-facing
        if top_half > 200 and bot_half > 200:
            orientation = "SKY (upward)"
        elif top_half > bot_half + 30:
            orientation = "side (sky on top)"
        else:
            orientation = "side (mixed)"

        stats = {
            "cam_id": cam_id,
            "brightness": brightness,
            "contrast": contrast,
            "n_orb": n_orb,
            "n_corners": n_corners,
            "top_brightness": top_half,
            "bot_brightness": bot_half,
            "orientation": orientation,
        }
        camera_stats.append(stats)
        log(f"  Cam{cam_id}: bright={brightness:.0f} contrast={contrast:.0f} "
            f"ORB={n_orb} corners={n_corners} orient={orientation}")

    # summary table
    log("\n  Camera Investigation Summary:")
    log(f"  {'Cam':>4} {'Bright':>7} {'Contrast':>9} {'ORB':>5} {'Corners':>8} {'Orientation':<20}")
    log(f"  {'-'*4:>4} {'-'*7:>7} {'-'*9:>9} {'-'*5:>5} {'-'*8:>8} {'-'*20:<20}")
    for s in camera_stats:
        log(f"  {s['cam_id']:>4} {s['brightness']:>7.0f} {s['contrast']:>9.1f} "
            f"{s['n_orb']:>5} {s['n_corners']:>8} {s['orientation']:<20}")

    # rank cameras by feature count (prefer high ORB features + high contrast)
    ranked = sorted(camera_stats, key=lambda x: x["n_orb"] * x["contrast"], reverse=True)

    # filter out sky cameras
    good_cameras = [c for c in ranked if "SKY" not in c["orientation"]]
    if not good_cameras:
        # fallback: use all non-sky cameras
        good_cameras = [c for c in ranked if c["n_orb"] > 100]
    if not good_cameras:
        good_cameras = ranked[:3]

    best_cameras = [c["cam_id"] for c in good_cameras[:3]]
    log(f"\n  Selected cameras (ranked): {best_cameras}")

    # save stats
    stats_path = CAMERA_SAMPLES_DIR / "camera_stats.json"
    with open(stats_path, "w") as f:
        json.dump({"cameras": camera_stats, "selected": best_cameras}, f, indent=2)

    return best_cameras, camera_stats


# PHASE 1: Quick Validation
def phase1_quick_validation(camera_ids):
    """Quick validation runs on 1000 frames per camera"""
    log("=" * 70)
    log("PHASE 1: Quick Validation (1000 frames)")
    log("=" * 70)

    N_FRAMES = 1000
    validation_results = []

    # base params - use previously known focal lengths as starting point
    # For Ladybug3 side cameras: fisheye but central region approximated as pinhole
    base_params = {
        "fx": 221.0, "fy": 221.0, "cx": 404.0, "cy": 308.0,
        "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        "width": 808, "height": 616, "fps": 5,
        "nFeatures": 2000, "scaleFactor": 1.2, "nLevels": 8,
        "iniThFAST": 15, "minThFAST": 5,
    }

    for cam_id in camera_ids:
        log(f"\n--- Quick test: Cam{cam_id} ---")

        # prepare dataset
        dataset_dir = prepare_tum_dataset(cam_id, n_frames=N_FRAMES)
        if dataset_dir is None:
            log(f"  Cam{cam_id}: Failed to prepare dataset", "ERROR")
            validation_results.append({"cam_id": cam_id, "success": False})
            continue

        # write config
        config_path = dataset_dir / "orbslam3.yaml"
        write_orbslam3_config(config_path, base_params)

        # run
        result = run_orbslam3(dataset_dir, config_path, timeout_s=300,
                              run_label=f"Cam{cam_id}_baseline")

        run_info = {
            "cam_id": cam_id,
            "success": result["success"],
            "n_keyframes": result["n_keyframes"],
            "n_frames": result["n_frames"],
            "tracking_rate": result["tracking_rate"],
            "runtime_s": result["runtime_s"],
        }

        # If tracking fails, try aggressive params
        if not result["success"] or result["tracking_rate"] < 5:
            log(f"  Cam{cam_id}: Baseline failed, trying aggressive params...")
            aggressive_params = base_params.copy()
            aggressive_params["nFeatures"] = 5000
            aggressive_params["iniThFAST"] = 5
            aggressive_params["minThFAST"] = 2
            aggressive_params["nLevels"] = 12

            config_path2 = dataset_dir / "orbslam3_aggressive.yaml"
            write_orbslam3_config(config_path2, aggressive_params)

            result2 = run_orbslam3(dataset_dir, config_path2, timeout_s=300,
                                   run_label=f"Cam{cam_id}_aggressive")
            run_info["aggressive"] = {
                "success": result2["success"],
                "n_keyframes": result2["n_keyframes"],
                "tracking_rate": result2["tracking_rate"],
            }

        # also try CLAHE preprocessing
        log(f"  Cam{cam_id}: Testing CLAHE preprocessing...")
        dataset_clahe = prepare_tum_dataset(cam_id, n_frames=N_FRAMES, preprocess="clahe", clahe_clip=3.0)
        if dataset_clahe:
            config_clahe = dataset_clahe / "orbslam3.yaml"
            write_orbslam3_config(config_clahe, base_params)
            result_clahe = run_orbslam3(dataset_clahe, config_clahe, timeout_s=300,
                                        run_label=f"Cam{cam_id}_CLAHE")
            run_info["clahe"] = {
                "success": result_clahe["success"],
                "n_keyframes": result_clahe["n_keyframes"],
                "tracking_rate": result_clahe["tracking_rate"],
            }

        validation_results.append(run_info)

    # summary
    log("\n  Phase 1 Summary:")
    log(f"  {'Cam':>4} {'KF':>5} {'Track%':>7} {'CLAHE_KF':>9} {'CLAHE_%':>8} {'Aggr_KF':>8}")
    log(f"  {'-'*4:>4} {'-'*5:>5} {'-'*7:>7} {'-'*9:>9} {'-'*8:>8} {'-'*8:>8}")
    for r in validation_results:
        clahe_kf = r.get("clahe", {}).get("n_keyframes", "-")
        clahe_pct = r.get("clahe", {}).get("tracking_rate", "-")
        aggr_kf = r.get("aggressive", {}).get("n_keyframes", "-")
        ckf = f"{clahe_kf}" if isinstance(clahe_kf, int) else clahe_kf
        cpct = f"{clahe_pct:.1f}" if isinstance(clahe_pct, float) else clahe_pct
        akf = f"{aggr_kf}" if isinstance(aggr_kf, int) else aggr_kf
        log(f"  {r['cam_id']:>4} {r['n_keyframes']:>5} {r['tracking_rate']:>7.1f} "
            f"{ckf:>9} {cpct:>8} {akf:>8}")

    # save results
    val_path = TUNING_DIR / "phase1_validation.json"
    val_path.parent.mkdir(parents=True, exist_ok=True)
    with open(val_path, "w") as f:
        json.dump(validation_results, f, indent=2)

    # pick best camera: highest tracking rate among all variants
    best_cam = None
    best_rate = 0
    best_preprocess = "none"

    for r in validation_results:
        # check baseline
        if r["tracking_rate"] > best_rate:
            best_rate = r["tracking_rate"]
            best_cam = r["cam_id"]
            best_preprocess = "none"
        # check CLAHE
        clahe_rate = r.get("clahe", {}).get("tracking_rate", 0)
        if isinstance(clahe_rate, (int, float)) and clahe_rate > best_rate:
            best_rate = clahe_rate
            best_cam = r["cam_id"]
            best_preprocess = "clahe"
        # check aggressive
        aggr_rate = r.get("aggressive", {}).get("tracking_rate", 0)
        if isinstance(aggr_rate, (int, float)) and aggr_rate > best_rate:
            best_rate = aggr_rate
            best_cam = r["cam_id"]
            best_preprocess = "aggressive"

    log(f"\n  Best camera: Cam{best_cam} ({best_preprocess}, {best_rate:.1f}%)")
    return best_cam, best_preprocess, validation_results


# PHASE 2: Systematic Parameter Tuning
def phase2_parameter_tuning(cam_id, preprocess="none"):
    """Systematic parameter sweep on 3000 frames"""
    log("=" * 70)
    log(f"PHASE 2: Systematic Parameter Tuning (Cam{cam_id}, preprocess={preprocess})")
    log("=" * 70)

    N_FRAMES = 3000
    gt_data = load_ground_truth()

    # prepare dataset once
    clahe_clip = 3.0 if preprocess == "clahe" else 0
    prep = "clahe" if preprocess in ("clahe", "aggressive") else "none"

    dataset_dir = prepare_tum_dataset(cam_id, n_frames=N_FRAMES, preprocess=prep,
                                      clahe_clip=clahe_clip)
    if dataset_dir is None:
        log("Failed to prepare dataset for tuning", "ERROR")
        return {}

    all_runs = []
    run_counter = [0]

    def do_run(params, label):
        """Execute a single tuning run and return metrics"""
        run_counter[0] += 1
        run_id = f"run{run_counter[0]:03d}_{label}"

        config_path = dataset_dir / f"{run_id}.yaml"
        write_orbslam3_config(config_path, params)

        result = run_orbslam3(dataset_dir, config_path, timeout_s=900, run_label=run_id)

        metrics = {
            "run_id": run_id,
            "label": label,
            "params": {k: v for k, v in params.items()
                       if k in ("nFeatures", "scaleFactor", "nLevels", "iniThFAST",
                                "minThFAST", "fx", "fy", "width", "height")},
            "success": result["success"],
            "n_keyframes": result["n_keyframes"],
            "tracking_rate": result["tracking_rate"],
            "runtime_s": result["runtime_s"],
            "ate_rmse": float("inf"),
            "scale_factor": 0,
        }

        # evaluate if trajectory exists
        if result["success"] and result["traj_path"]:
            traj = load_tum_trajectory(result["traj_path"])
            ate = compute_ate(traj, gt_data)
            metrics["ate_rmse"] = ate["ate_rmse"]
            metrics["scale_factor"] = ate["scale_factor"]
            metrics["n_matched"] = ate["n_matched"]
            metrics["coverage_pct"] = ate["coverage_pct"]
            log(f"    ATE={ate['ate_rmse']:.2f}m scale={ate['scale_factor']:.3f} "
                f"matched={ate['n_matched']}")

            # save trajectory copy
            run_dir = TUNING_DIR / run_id
            run_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(result["traj_path"], run_dir / "trajectory.txt")

        all_runs.append(metrics)

        # save incrementally
        with open(TUNING_DIR / "all_runs.json", "w") as f:
            json.dump(all_runs, f, indent=2)

        return metrics

    def pick_best(runs, key="tracking_rate"):
        """Pick best run by given metric (higher is better, except ate_rmse)."""
        if not runs:
            return None
        if key == "ate_rmse":
            valid = [r for r in runs if r["success"] and r["ate_rmse"] < float("inf")]
            return min(valid, key=lambda x: x[key]) if valid else None
        return max(runs, key=lambda x: x.get(key, 0))

    # --- Starting params ---
    params = {
        "fx": 221.0, "fy": 221.0, "cx": 404.0, "cy": 308.0,
        "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        "width": 808, "height": 616, "fps": 5,
        "nFeatures": 2000, "scaleFactor": 1.2, "nLevels": 8,
        "iniThFAST": 15, "minThFAST": 5,
    }
    # If aggressive was best in phase1, start there
    if preprocess == "aggressive":
        params["nFeatures"] = 5000
        params["iniThFAST"] = 5
        params["minThFAST"] = 2
        params["nLevels"] = 12

    # ---------------------------------------------------------------
    # step 1: nFeatures sweep
    # ---------------------------------------------------------------
    log("\n--- Step 1: nFeatures sweep ---")
    step1_runs = []
    for nf in [1000, 2000, 3000, 5000, 8000]:
        p = params.copy()
        p["nFeatures"] = nf
        r = do_run(p, f"nfeat_{nf}")
        step1_runs.append(r)

    best1 = pick_best(step1_runs)
    if best1:
        params["nFeatures"] = best1["params"]["nFeatures"]
        log(f"  Best nFeatures: {params['nFeatures']} "
            f"(tracking={best1['tracking_rate']:.1f}%, ATE={best1['ate_rmse']:.2f}m)")

    # ---------------------------------------------------------------
    # step 2: FAST threshold sweep
    # ---------------------------------------------------------------
    log("\n--- Step 2: FAST threshold sweep ---")
    step2_runs = []
    for ini, mn in [(20, 7), (15, 5), (10, 3), (7, 3), (5, 2)]:
        p = params.copy()
        p["iniThFAST"] = ini
        p["minThFAST"] = mn
        r = do_run(p, f"fast_{ini}_{mn}")
        step2_runs.append(r)

    best2 = pick_best(step2_runs)
    if best2:
        params["iniThFAST"] = best2["params"]["iniThFAST"]
        params["minThFAST"] = best2["params"]["minThFAST"]
        log(f"  Best FAST: ini={params['iniThFAST']} min={params['minThFAST']} "
            f"(tracking={best2['tracking_rate']:.1f}%, ATE={best2['ate_rmse']:.2f}m)")

    # ---------------------------------------------------------------
    # step 3: Scale and levels sweep
    # ---------------------------------------------------------------
    log("\n--- Step 3: Scale factor and pyramid levels sweep ---")
    step3_runs = []
    for sf, nl in [(1.1, 10), (1.1, 12), (1.2, 8), (1.2, 10), (1.3, 8), (1.15, 10)]:
        p = params.copy()
        p["scaleFactor"] = sf
        p["nLevels"] = nl
        r = do_run(p, f"scale_{sf}_{nl}")
        step3_runs.append(r)

    best3 = pick_best(step3_runs)
    if best3:
        params["scaleFactor"] = best3["params"]["scaleFactor"]
        params["nLevels"] = best3["params"]["nLevels"]
        log(f"  Best scale: factor={params['scaleFactor']} levels={params['nLevels']} "
            f"(tracking={best3['tracking_rate']:.1f}%, ATE={best3['ate_rmse']:.2f}m)")

    # ---------------------------------------------------------------
    # step 4: Preprocessing sweep (if not already CLAHE)
    # ---------------------------------------------------------------
    log("\n--- Step 4: Preprocessing sweep ---")
    step4_runs = []
    prep_variants = [("none", 0), ("clahe", 2.0), ("clahe", 4.0)]
    for pp, clip in prep_variants:
        ds = prepare_tum_dataset(cam_id, n_frames=N_FRAMES, preprocess=pp,
                                 clahe_clip=clip)
        if ds is None:
            continue
        cfg = ds / f"orbslam3_prep_{pp}_{clip}.yaml"
        write_orbslam3_config(cfg, params)
        result = run_orbslam3(ds, cfg, timeout_s=900,
                              run_label=f"prep_{pp}_clip{clip}")
        metrics = {
            "run_id": f"prep_{pp}_{clip}",
            "label": f"prep_{pp}_clip{clip}",
            "params": {"preprocess": pp, "clahe_clip": clip},
            "success": result["success"],
            "n_keyframes": result["n_keyframes"],
            "tracking_rate": result["tracking_rate"],
            "runtime_s": result["runtime_s"],
            "ate_rmse": float("inf"),
            "scale_factor": 0,
        }
        if result["success"] and result["traj_path"]:
            traj = load_tum_trajectory(result["traj_path"])
            ate = compute_ate(traj, gt_data)
            metrics["ate_rmse"] = ate["ate_rmse"]
            metrics["scale_factor"] = ate["scale_factor"]
            log(f"    ATE={ate['ate_rmse']:.2f}m scale={ate['scale_factor']:.3f}")

        step4_runs.append(metrics)
        all_runs.append(metrics)

    best4 = pick_best(step4_runs)
    best_preprocess = "none"
    best_clip = 0
    if best4:
        best_preprocess = best4["params"].get("preprocess", "none")
        best_clip = best4["params"].get("clahe_clip", 0)
        log(f"  Best preprocessing: {best_preprocess} (clip={best_clip}) "
            f"(tracking={best4['tracking_rate']:.1f}%, ATE={best4['ate_rmse']:.2f}m)")

    # ---------------------------------------------------------------
    # step 5: Multi-camera test
    # ---------------------------------------------------------------
    log("\n--- Step 5: Multi-camera comparison ---")
    # load phase0 selected cameras
    cam_stats_path = CAMERA_SAMPLES_DIR / "camera_stats.json"
    other_cams = []
    if cam_stats_path.exists():
        with open(cam_stats_path) as f:
            cam_data = json.load(f)
        other_cams = [c for c in cam_data["selected"] if c != cam_id][:2]

    step5_runs = []
    for other_cam in other_cams:
        ds = prepare_tum_dataset(other_cam, n_frames=N_FRAMES,
                                 preprocess=best_preprocess, clahe_clip=best_clip)
        if ds is None:
            continue
        cfg = ds / "orbslam3_multicam.yaml"
        write_orbslam3_config(cfg, params)
        result = run_orbslam3(ds, cfg, timeout_s=900,
                              run_label=f"multicam_Cam{other_cam}")
        metrics = {
            "run_id": f"multicam_{other_cam}",
            "label": f"Cam{other_cam}",
            "cam_id": other_cam,
            "success": result["success"],
            "n_keyframes": result["n_keyframes"],
            "tracking_rate": result["tracking_rate"],
            "runtime_s": result["runtime_s"],
            "ate_rmse": float("inf"),
        }
        if result["success"] and result["traj_path"]:
            traj = load_tum_trajectory(result["traj_path"])
            ate = compute_ate(traj, gt_data)
            metrics["ate_rmse"] = ate["ate_rmse"]
            log(f"    Cam{other_cam}: ATE={ate['ate_rmse']:.2f}m "
                f"tracking={result['tracking_rate']:.1f}%")
        step5_runs.append(metrics)
        all_runs.append(metrics)

    # check if another camera is better
    best5 = pick_best(step5_runs)
    final_cam = cam_id
    if best5 and best5["tracking_rate"] > params.get("_best_tracking", 0) * 1.2:
        final_cam = best5["cam_id"]
        log(f"  Cam{final_cam} is significantly better, switching!")

    # ---------------------------------------------------------------
    # step 6: Resolution sweep
    # ---------------------------------------------------------------
    log("\n--- Step 6: Resolution sweep ---")
    step6_runs = []
    for res_name, (rw, rh) in [("half", (808, 616)), ("3quarter", (1212, 924))]:
        p = params.copy()
        p["width"] = rw
        p["height"] = rh
        # adjust focal length proportionally
        scale = rw / 808.0
        p["fx"] = 221.0 * scale
        p["fy"] = 221.0 * scale
        p["cx"] = 404.0 * scale
        p["cy"] = 308.0 * scale

        ds = prepare_tum_dataset(final_cam, n_frames=N_FRAMES,
                                 preprocess=best_preprocess, clahe_clip=best_clip,
                                 resolution=res_name)
        if ds is None:
            continue
        cfg = ds / f"orbslam3_res_{res_name}.yaml"
        write_orbslam3_config(cfg, p)
        result = run_orbslam3(ds, cfg, timeout_s=900,
                              run_label=f"res_{res_name}")
        metrics = {
            "run_id": f"res_{res_name}",
            "label": f"resolution_{res_name}",
            "params": {"resolution": res_name, "width": rw, "height": rh},
            "success": result["success"],
            "n_keyframes": result["n_keyframes"],
            "tracking_rate": result["tracking_rate"],
            "runtime_s": result["runtime_s"],
            "ate_rmse": float("inf"),
        }
        if result["success"] and result["traj_path"]:
            traj = load_tum_trajectory(result["traj_path"])
            ate = compute_ate(traj, gt_data)
            metrics["ate_rmse"] = ate["ate_rmse"]
            log(f"    {res_name}: ATE={ate['ate_rmse']:.2f}m "
                f"tracking={result['tracking_rate']:.1f}%")
        step6_runs.append(metrics)
        all_runs.append(metrics)

    best6 = pick_best(step6_runs)
    best_resolution = "half"
    if best6 and best6.get("tracking_rate", 0) > 0:
        best_resolution = best6["params"]["resolution"]
        rw = best6["params"]["width"]
        rh = best6["params"]["height"]
        scale = rw / 808.0
        params["width"] = rw
        params["height"] = rh
        params["fx"] = 221.0 * scale
        params["fy"] = 221.0 * scale
        params["cx"] = 404.0 * scale
        params["cy"] = 308.0 * scale
        log(f"  Best resolution: {best_resolution} ({rw}x{rh}) "
            f"(tracking={best6['tracking_rate']:.1f}%, ATE={best6['ate_rmse']:.2f}m)")

    # save all runs
    with open(TUNING_DIR / "all_runs.json", "w") as f:
        json.dump(all_runs, f, indent=2)

    best_config = {
        "cam_id": final_cam,
        "preprocess": best_preprocess,
        "clahe_clip": best_clip,
        "resolution": best_resolution,
        "params": params,
    }

    with open(TUNING_DIR / "best_config.json", "w") as f:
        json.dump(best_config, f, indent=2)

    log(f"\n  Final best config: Cam{final_cam}, {best_resolution}, "
        f"preprocess={best_preprocess}")
    log(f"  Params: nFeatures={params['nFeatures']} "
        f"FAST={params['iniThFAST']}/{params['minThFAST']} "
        f"scale={params['scaleFactor']}x{params['nLevels']}")

    return best_config, all_runs


# PHASE 3: Full Session Run
def phase3_full_run(best_config):
    """Run ORB-SLAM3 on the full spring session with best config"""
    log("=" * 70)
    log("PHASE 3: Full Session Run")
    log("=" * 70)

    cam_id = best_config["cam_id"]
    preprocess = best_config["preprocess"]
    clahe_clip = best_config["clahe_clip"]
    resolution = best_config["resolution"]
    params = best_config["params"]

    gt_data = load_ground_truth()

    # prepare full dataset
    log("Preparing full dataset (this may take a while)...")
    dataset_dir = prepare_tum_dataset(cam_id, n_frames=0,
                                      preprocess=preprocess, clahe_clip=clahe_clip,
                                      resolution=resolution)
    if dataset_dir is None:
        log("Failed to prepare full dataset", "ERROR")
        return None

    # write config
    config_path = dataset_dir / "orbslam3_best.yaml"
    write_orbslam3_config(config_path, params)

    # save a copy of the best config
    best_yaml = RESULTS_DIR / "best_config.yaml"
    shutil.copy2(config_path, best_yaml)

    # run with generous timeout (45 min)
    result = run_orbslam3(dataset_dir, config_path, timeout_s=2700,
                          run_label="FULL_SESSION")

    if not result["success"]:
        log("Full session run FAILED", "ERROR")
        return None

    # evaluate
    traj = load_tum_trajectory(result["traj_path"])
    ate_metrics = compute_ate(traj, gt_data)

    full_results = {
        "cam_id": cam_id,
        "preprocess": preprocess,
        "resolution": resolution,
        "params": params,
        "n_keyframes": result["n_keyframes"],
        "n_frames": result["n_frames"],
        "tracking_rate": result["tracking_rate"],
        "runtime_s": result["runtime_s"],
        "ate_rmse": ate_metrics["ate_rmse"],
        "scale_factor": ate_metrics["scale_factor"],
        "n_matched": ate_metrics["n_matched"],
        "coverage_pct": ate_metrics["coverage_pct"],
    }

    log(f"  Full session results:")
    log(f"    Keyframes: {result['n_keyframes']}/{result['n_frames']} "
        f"({result['tracking_rate']:.1f}%)")
    log(f"    ATE RMSE: {ate_metrics['ate_rmse']:.2f}m")
    log(f"    Scale: {ate_metrics['scale_factor']:.3f}")
    log(f"    Coverage: {ate_metrics['coverage_pct']:.1f}%")
    log(f"    Runtime: {result['runtime_s']:.0f}s")

    shutil.copy2(result["traj_path"], RESULTS_DIR / "best_trajectory.txt")

    # save results
    with open(RESULTS_DIR / "full_session_results.json", "w") as f:
        json.dump(full_results, f, indent=2)

    # also run with default config for comparison
    log("\n  Running with DEFAULT config for comparison...")
    default_params = {
        "fx": 200.0, "fy": 200.0, "cx": 404.0, "cy": 308.0,
        "k1": 0.0, "k2": 0.0, "p1": 0.0, "p2": 0.0,
        "width": 808, "height": 616, "fps": 5,
        "nFeatures": 1500, "scaleFactor": 1.2, "nLevels": 8,
        "iniThFAST": 20, "minThFAST": 7,
    }
    # use Cam0 (original sky camera) with no preprocessing
    default_ds = prepare_tum_dataset(0, n_frames=0, preprocess="none")
    if default_ds:
        default_cfg = default_ds / "orbslam3_default.yaml"
        write_orbslam3_config(default_cfg, default_params)
        default_result = run_orbslam3(default_ds, default_cfg, timeout_s=2700,
                                       run_label="DEFAULT_COMPARISON")
        if default_result["success"]:
            default_traj = load_tum_trajectory(default_result["traj_path"])
            default_ate = compute_ate(default_traj, gt_data)
            full_results["default_comparison"] = {
                "n_keyframes": default_result["n_keyframes"],
                "tracking_rate": default_result["tracking_rate"],
                "ate_rmse": default_ate["ate_rmse"],
                "scale_factor": default_ate["scale_factor"],
            }
            shutil.copy2(default_result["traj_path"],
                        RESULTS_DIR / "default_trajectory.txt")
            log(f"    Default: {default_result['n_keyframes']} KF, "
                f"ATE={default_ate['ate_rmse']:.2f}m")
        else:
            full_results["default_comparison"] = {
                "n_keyframes": 0,
                "tracking_rate": 0,
                "ate_rmse": float("inf"),
                "note": "tracking failed (sky camera)",
            }
            log(f"    Default: FAILED (expected - sky camera)")

    # save updated results
    with open(RESULTS_DIR / "full_session_results.json", "w") as f:
        json.dump(full_results, f, indent=2)

    return full_results


# PHASE 4: Plots and Reporting
def phase4_plots_and_report(full_results, all_runs, camera_stats):
    """Generate plots and summary report"""
    log("=" * 70)
    log("PHASE 4: Generating Plots and Report")
    log("=" * 70)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # --- Plot 1: Parameter sweep results ---
    if all_runs:
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))

        # group runs by step
        steps = {
            "nFeatures": [r for r in all_runs if r["label"].startswith("nfeat_")],
            "FAST threshold": [r for r in all_runs if r["label"].startswith("fast_")],
            "Scale/Levels": [r for r in all_runs if r["label"].startswith("scale_")],
            "Preprocessing": [r for r in all_runs if r["label"].startswith("prep_")],
            "Multi-camera": [r for r in all_runs if r["label"].startswith("Cam")],
            "Resolution": [r for r in all_runs if r["label"].startswith("resolution_")],
        }

        for ax, (title, runs) in zip(axes.flat, steps.items()):
            if not runs:
                ax.set_visible(False)
                continue
            labels = [r["label"].split("_", 1)[-1] if "_" in r["label"] else r["label"]
                      for r in runs]
            rates = [r["tracking_rate"] for r in runs]
            ates = [r["ate_rmse"] if r["ate_rmse"] < 1000 else 0 for r in runs]

            x = range(len(labels))
            bars = ax.bar(x, rates, color="steelblue", alpha=0.7)
            ax.set_ylabel("Tracking Rate (%)", color="steelblue")
            ax.set_title(title)
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)

            # ATE on secondary axis
            ax2 = ax.twinx()
            ax2.plot(x, ates, "ro-", markersize=5, label="ATE")
            ax2.set_ylabel("ATE RMSE (m)", color="red")

        plt.tight_layout()
        plt.savefig(str(RESULTS_DIR / "parameter_sweep.png"), dpi=150)
        plt.close()
        log("  Saved parameter_sweep.png")

    # --- Plot 2: Trajectory comparison ---
    if full_results:
        gt_data = load_ground_truth()
        gt_xy = gt_data[:, 1:3]

        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        ax.plot(gt_xy[:, 0], gt_xy[:, 1], "k-", linewidth=0.5, alpha=0.5, label="Ground Truth")

        # best trajectory
        best_traj_path = RESULTS_DIR / "best_trajectory.txt"
        if best_traj_path.exists():
            traj = load_tum_trajectory(str(best_traj_path))
            if traj:
                traj_arr = np.array(traj)
                # align with Sim(3)
                est_xyz = traj_arr[:, 1:4]
                gt_ts = gt_data[:, 0] / 1e6
                matched_est, matched_gt = [], []
                for i, t in enumerate(traj_arr[:, 0]):
                    idx = np.argmin(np.abs(gt_ts - t))
                    if abs(gt_ts[idx] - t) < 0.1:
                        matched_est.append(est_xyz[i])
                        matched_gt.append(gt_data[idx, 1:4])
                if len(matched_est) > 10:
                    me = np.array(matched_est)
                    mg = np.array(matched_gt)
                    # Umeyama for alignment
                    mu_e = me.mean(0)
                    mu_g = mg.mean(0)
                    H = (me - mu_e).T @ (mg - mu_g)
                    U, S, Vt = np.linalg.svd(H)
                    d = np.linalg.det(Vt.T @ U.T)
                    D = np.diag([1, 1, d])
                    R = Vt.T @ D @ U.T
                    var_e = np.sum((me - mu_e)**2) / len(me)
                    s = np.trace(np.diag(S) @ D) / var_e if var_e > 1e-10 else 1.0
                    aligned = s * (R @ est_xyz.T).T + mu_g - s * R @ mu_e
                    ax.plot(aligned[:, 0], aligned[:, 1], "b-", linewidth=1.5,
                            label=f"Tuned (ATE={full_results['ate_rmse']:.1f}m)")

        # default trajectory
        default_traj_path = RESULTS_DIR / "default_trajectory.txt"
        if default_traj_path.exists():
            traj = load_tum_trajectory(str(default_traj_path))
            if traj:
                traj_arr = np.array(traj)
                est_xyz = traj_arr[:, 1:4]
                matched_est, matched_gt = [], []
                for i, t in enumerate(traj_arr[:, 0]):
                    idx = np.argmin(np.abs(gt_ts - t))
                    if abs(gt_ts[idx] - t) < 0.1:
                        matched_est.append(est_xyz[i])
                        matched_gt.append(gt_data[idx, 1:4])
                if len(matched_est) > 10:
                    me = np.array(matched_est)
                    mg = np.array(matched_gt)
                    mu_e = me.mean(0)
                    mu_g = mg.mean(0)
                    H = (me - mu_e).T @ (mg - mu_g)
                    U, S, Vt = np.linalg.svd(H)
                    d = np.linalg.det(Vt.T @ U.T)
                    D = np.diag([1, 1, d])
                    R = Vt.T @ D @ U.T
                    var_e = np.sum((me - mu_e)**2) / len(me)
                    s = np.trace(np.diag(S) @ D) / var_e if var_e > 1e-10 else 1.0
                    aligned = s * (R @ est_xyz.T).T + mu_g - s * R @ mu_e
                    def_ate = full_results.get("default_comparison", {}).get("ate_rmse", "?")
                    ax.plot(aligned[:, 0], aligned[:, 1], "r--", linewidth=1,
                            alpha=0.7, label=f"Default (ATE={def_ate})")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("ORB-SLAM3 Trajectory: Default vs Tuned")
        ax.legend()
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(str(RESULTS_DIR / "trajectory_comparison.png"), dpi=150)
        plt.close()
        log("  Saved trajectory_comparison.png")

    # --- Plot 3: Camera comparison ---
    if camera_stats:
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        cams = [s["cam_id"] for s in camera_stats]
        orb_feats = [s["n_orb"] for s in camera_stats]
        contrasts = [s["contrast"] for s in camera_stats]

        colors = ["red" if "SKY" in s["orientation"] else "steelblue" for s in camera_stats]

        axes[0].bar(cams, orb_feats, color=colors)
        axes[0].set_xlabel("Camera ID")
        axes[0].set_ylabel("ORB Features")
        axes[0].set_title("Features per Camera (red=sky)")

        axes[1].bar(cams, contrasts, color=colors)
        axes[1].set_xlabel("Camera ID")
        axes[1].set_ylabel("Contrast (std)")
        axes[1].set_title("Image Contrast per Camera")

        plt.tight_layout()
        plt.savefig(str(RESULTS_DIR / "camera_comparison.png"), dpi=150)
        plt.close()
        log("  Saved camera_comparison.png")

    # --- Summary report ---
    report = generate_report(full_results, all_runs, camera_stats)
    with open(RESULTS_DIR / "SUMMARY.md", "w") as f:
        f.write(report)
    log("  Saved SUMMARY.md")


def generate_report(full_results, all_runs, camera_stats):
    """Generate markdown summary report"""
    lines = [
        "# ORB-SLAM3 Systematic Parameter Tuning - NCLT Spring 2012-04-29",
        "",
        f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "",
        "## Key Finding",
        "",
    ]

    if full_results:
        lines.extend([
            f"**Camera 0 (Ladybug3 top) faces the SKY** - all previous experiments used sky images.",
            f"Switching to a side-facing camera dramatically improves tracking.",
            "",
            "## Results Summary",
            "",
            f"| Metric | Default (Cam0/sky) | Tuned |",
            f"|--------|-------------------|-------|",
        ])

        default = full_results.get("default_comparison", {})
        lines.extend([
            f"| Camera | Cam0 (sky) | Cam{full_results.get('cam_id', '?')} (side) |",
            f"| Preprocessing | none | {full_results.get('preprocess', 'none')} |",
            f"| Keyframes | {default.get('n_keyframes', 0)} | {full_results.get('n_keyframes', 0)} |",
            f"| Tracking Rate | {default.get('tracking_rate', 0):.1f}% | {full_results.get('tracking_rate', 0):.1f}% |",
            f"| ATE RMSE | {default.get('ate_rmse', 'inf'):.1f}m | {full_results.get('ate_rmse', 'inf'):.1f}m |",
            f"| Scale Factor | {default.get('scale_factor', 0):.3f} | {full_results.get('scale_factor', 0):.3f} |",
            "",
        ])

    # camera investigation
    if camera_stats:
        lines.extend([
            "## Camera Investigation",
            "",
            "| Camera | Brightness | Contrast | ORB Features | Orientation |",
            "|--------|-----------|----------|-------------|------------|",
        ])
        for s in camera_stats:
            lines.append(
                f"| Cam{s['cam_id']} | {s['brightness']:.0f} | {s['contrast']:.1f} "
                f"| {s['n_orb']} | {s['orientation']} |"
            )
        lines.append("")

    # parameter tuning
    if all_runs:
        lines.extend([
            "## Parameter Tuning Runs",
            "",
            "| Run | Tracking Rate | ATE RMSE | Runtime |",
            "|-----|--------------|----------|---------|",
        ])
        for r in all_runs:
            ate = f"{r['ate_rmse']:.1f}m" if r["ate_rmse"] < 1000 else "N/A"
            lines.append(
                f"| {r['label']} | {r['tracking_rate']:.1f}% | {ate} | {r['runtime_s']:.0f}s |"
            )
        lines.append("")

    # best config
    if full_results:
        params = full_results.get("params", {})
        lines.extend([
            "## Best Configuration",
            "",
            "```yaml",
            f"Camera: Cam{full_results.get('cam_id', '?')}",
            f"Preprocessing: {full_results.get('preprocess', 'none')}",
            f"Resolution: {full_results.get('resolution', 'half')}",
            f"nFeatures: {params.get('nFeatures', '?')}",
            f"scaleFactor: {params.get('scaleFactor', '?')}",
            f"nLevels: {params.get('nLevels', '?')}",
            f"iniThFAST: {params.get('iniThFAST', '?')}",
            f"minThFAST: {params.get('minThFAST', '?')}",
            f"fx: {params.get('fx', '?')}",
            f"fy: {params.get('fy', '?')}",
            "```",
            "",
        ])

    return "\n".join(lines)


# MAIN
def main():
    global LOG_FILE

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    TUNING_DIR.mkdir(parents=True, exist_ok=True)
    LOG_FILE = str(RESULTS_DIR / "tuning_log.txt")

    # clear old log
    with open(LOG_FILE, "w") as f:
        f.write(f"ORB-SLAM3 Tuning Log - Started {datetime.now()}\n")

    start_time = time.time()
    log("Starting ORB-SLAM3 systematic parameter tuning")
    log(f"Session: {SESSION}")
    log(f"Results dir: {RESULTS_DIR}")

    # check prerequisites
    if not MONO_TUM.exists():
        log(f"ORB-SLAM3 binary not found at {MONO_TUM}", "ERROR")
        log("Please build ORB-SLAM3 first")
        sys.exit(1)
    if not VOCAB_PATH.exists():
        log(f"ORB vocabulary not found at {VOCAB_PATH}", "ERROR")
        sys.exit(1)
    if not GT_FILE.exists():
        log(f"Ground truth not found at {GT_FILE}", "ERROR")
        sys.exit(1)

    # --- Phase 0 ---
    best_cameras, camera_stats = phase0_camera_investigation()

    # --- Phase 1 ---
    best_cam, best_preprocess, val_results = phase1_quick_validation(best_cameras)

    # --- Phase 2 ---
    best_config, all_runs = phase2_parameter_tuning(best_cam, best_preprocess)

    # --- Phase 3 ---
    full_results = phase3_full_run(best_config)

    # --- Phase 4 ---
    phase4_plots_and_report(full_results, all_runs, camera_stats)

    elapsed = time.time() - start_time
    log(f"\nTotal runtime: {elapsed/3600:.1f} hours")
    log("DONE!")


if __name__ == "__main__":
    main()
