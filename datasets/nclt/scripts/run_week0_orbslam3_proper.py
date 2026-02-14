#!/usr/bin/env python3
"""Experiment 0.6: proper ORB-SLAM3 run on NCLT after fixing earlier mistakes.

exp 0.2-0.5 all used Cam0 which turns out to be the top/sky-facing camera (!).
also forced a plain PinHole on a fisheye lens. this script redoes ORB-SLAM3
with: KannalaBrandt8 fisheye model, side cameras (Cam5 forward / Cam4 left /
Cam1 right), mono-inertial + stereo-inertial modes using MS25 + KVH hybrid IMU,
extrinsics computed from NCLT calibration + Ladybug3 geometry.

walks through: prep + calibration, mono-inertial sweep on 3k frames, then
stereo-inertial on 3k, then full spring session with the best config, post-hoc
wheel-odom fusion, summer session repeat, plots + report
"""

import os
import sys
import cv2
import json
import time
import shutil
import subprocess
import numpy as np
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d

# CONSTANTS
PROJECT_ROOT = Path(__file__).resolve().parent.parent
NCLT_DATA = Path("/workspace/nclt_data")
ORB_SLAM3_DIR = Path("/tmp/ORB_SLAM3")
VOCAB_PATH = ORB_SLAM3_DIR / 'Vocabulary' / "ORBvoc.txt"
MONO_INERTIAL_BIN = ORB_SLAM3_DIR / "Examples" / 'Monocular-Inertial' / "mono_inertial_euroc"
STEREO_INERTIAL_BIN = ORB_SLAM3_DIR / "Examples" / "Stereo-Inertial" / "stereo_inertial_euroc"

RESULTS_DIR = PROJECT_ROOT / "results" / "week0_orbslam3_proper"
CALIB_DIR = RESULTS_DIR / "calibration"
IMU_DIR = RESULTS_DIR / "imu_data"

SESSIONS = {"spring": "2012-04-29", "summer": "2012-08-04"}

# Ladybug3 camera yaw angles from forward (degrees).
# Cam0=top(sky, unusable), Cam1=right(+73), Cam2=back-right(+144),
# Cam3=back-left(-143), Cam4=left(-72), Cam5=forward(0).
# Only the three best side cameras (1, 4, 5) are used in the sweep.
CAM_ANGLES = {1: 73.0, 4: -72.0, 5: 0.0}

# Ladybug3 approximate geometry. pole-mounted on the Segway, hence
# center ~1.23 m above body origin in NED.
LB3_RADIUS = 0.08  # sphere radius in meters
LB3_CENTER_BODY = np.array([0.035, 0.002, -1.23])

# default fisheye calibration (half-res 808x616, equidistant Kannala-Brandt).
# For equidistant: f = r_max / theta_max = 404 / (60 deg in rad) ~= 386.
# assumes ~120 deg horizontal FOV for Ladybug3 side cameras.
DEFAULT_FISHEYE = {
    "fx": 386.0, "fy": 386.0, "cx": 404.0, "cy": 308.0,
    "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0,
}

# default IMU noise (Microstrain 3DM-GX3-45, relaxed for real-world conditions)
DEFAULT_IMU_NOISE = {
    "gyro_noise": 0.004,    # rad/s^0.5 (datasheet ~0.03 deg/s/sqrt(Hz) ≈ 5e-4, relaxed 8x)
    "accel_noise": 0.04,    # m/s^1.5 (datasheet ~0.03 m/s^2/sqrt(Hz), relaxed)
    "gyro_walk": 0.0002,    # rad/s^1.5
    "accel_walk": 0.004,    # m/s^2.5
}

# voxel_size = 0.3  # was 0.5 too coarse for poles
N_SWEEP_FRAMES = 3000
SWEEP_TIMEOUT = 900    # 15 min per sweep run
FULL_TIMEOUT = 5400    # 90 min for full session

# LOGGING
LOG_FILE = None
SCRIPT_START = None


def log(msg, level="INFO"):
    """Print and log a timestamped message"""
    ts = datetime.now().strftime("%H:%M:%S")
    elapsed = ""
    if SCRIPT_START:
        dt = (datetime.now() - SCRIPT_START).total_seconds()
        h, m = int(dt // 3600), int((dt % 3600) // 60)
        elapsed = f" [{h}h{m:02d}m]"
    line = f"[{ts}]{elapsed} [{level}] {msg}"
    print(line, flush=True)
    if LOG_FILE:
        with open(LOG_FILE, "a") as f:
            f.write(line + "\n")


def save_json(data, path):
    """Save dict to JSON file"""
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2, default=str)


# ORB-SLAM3 ENVIRONMENT
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


# GEOMETRY UTILITIES
def compute_Tbc(cam_angle_deg, cam_id=5):
    """compute body-to-camera transform T_b_c for ORB-SLAM3"""
    theta = np.radians(cam_angle_deg)
    ct, st = np.cos(theta), np.sin(theta)

    # camera at yaw angle theta on Ladybug3 sphere:
    # cam_z (optical axis) → body: [cos(θ), sin(θ), 0]
    # cam_y (down) → body: [0, 0, 1] (NED z-down = cam y-down)
    # cam_x (right) → body: cam_y×cam_z = [0,0,1]×[cθ,sθ,0] = [-sθ, cθ, 0]
    R_cam_to_body = np.array([
        [-st, 0, ct],
        [ct,  0, st],
        [0,   1, 0],
    ])

    # camera position on Ladybug3 sphere in body NED frame
    cam_pos = LB3_CENTER_BODY + np.array([
        LB3_RADIUS * ct,
        LB3_RADIUS * st,
        0.0,
    ])

    T = np.eye(4)
    T[:3, :3] = R_cam_to_body
    T[:3, 3] = cam_pos
    return T


def compute_Tc1c2(cam1_angle, cam1_id, cam2_angle, cam2_id):
    """Compute relative transform from cam1 to cam2 for stereo config.

    Stereo.T_c1_c2 = inv(T_b_c1) @ T_b_c2 = cam2 pose in cam1 frame.
    """
    T_b_c1 = compute_Tbc(cam1_angle, cam1_id)
    T_b_c2 = compute_Tbc(cam2_angle, cam2_id)
    return np.linalg.inv(T_b_c1) @ T_b_c2


def umeyama_alignment(est, gt, with_scale=True):
    mu_e = est.mean(axis=0)
    mu_g = gt.mean(axis=0)
    ec = est - mu_e
    gc = gt - mu_g

    H = ec.T @ gc
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T

    if with_scale:
        var_e = np.sum(ec ** 2) / len(est)
        s = np.trace(np.diag(S) @ D) / var_e if var_e > 1e-10 else 1.0
    else:
        s = 1.0

    aligned = s * (R @ est.T).T + mu_g - s * R @ mu_e
    errs = np.linalg.norm(aligned - gt, axis=1)
    rmse = np.sqrt(np.mean(errs ** 2))
    return rmse, s, aligned, errs


# DATA LOADING
def load_ground_truth(session):
    """Load NCLT ground truth as (N,7): utime, x, y, z, r/qx, p/qy, y/qz"""
    path = NCLT_DATA / "ground_truth" / f"groundtruth_{session}.csv"
    return np.loadtxt(str(path), delimiter=",")


def load_ms25_imu(session):
    """Load MS25 IMU as (N,10): utime, mag_xyz, accel_xyz, rot_xyz"""
    path = NCLT_DATA / "sensor_data" / session / "ms25.csv"
    return np.loadtxt(str(path), delimiter=",")


def load_kvh(session):
    """Load KVH FOG heading as (N,2): utime, heading_rad"""
    path = NCLT_DATA / "sensor_data" / session / "kvh.csv"
    return np.loadtxt(str(path), delimiter=",")


def load_odometry(session, hz=100):
    """Load wheel odometry as (N,7): utime, x, y, z, roll, pitch, yaw"""
    fname = "odometry_mu_100hz.csv" if hz == 100 else "odometry_mu.csv"
    path = NCLT_DATA / "sensor_data" / session / fname
    return np.loadtxt(str(path), delimiter=",")


def load_tum_trajectory(path):
    """Load TUM/EuRoC trajectory: timestamp_s x y z qx qy qz qw"""
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                poses.append([float(x) for x in parts[:8]])
    return np.array(poses) if poses else np.zeros((0, 8))


def get_image_utimes(session, cam_id, n_frames=0, skip_start=200):
    """get sorted list of image timestamps (microseconds) for a camera"""
    src_dir = NCLT_DATA / "images" / session / "lb3" / f"Cam{cam_id}"
    tiffs = sorted(src_dir.glob("*.tiff"))
    tiffs = tiffs[skip_start:]
    if n_frames > 0:
        tiffs = tiffs[:n_frames]
    return [int(f.stem) for f in tiffs]


# EVALUATION
def evaluate_trajectory(est_path, gt_data, with_scale=True):
    """evaluate estimated trajectory against ground truth"""
    est = load_tum_trajectory(est_path)
    if len(est) < 10:
        return {"status": "too_few_poses", "n_poses": len(est)}

    est_ts = est[:, 0]
    # ORB-SLAM3 EuRoC output timestamps are in nanoseconds; convert to seconds
    if est_ts[0] > 1e15:  # nanoseconds
        est_ts = est_ts / 1e9
    elif est_ts[0] > 1e9:  # microseconds
        est_ts = est_ts / 1e6
    est_xyz = est[:, 1:4]

    gt_ts = gt_data[:, 0] / 1e6  # microseconds → seconds
    gt_xyz = gt_data[:, 1:4]

    # match timestamps (nearest neighbor, 0.15s tolerance for 5Hz camera)
    matched_est, matched_gt = [], []
    for i, t in enumerate(est_ts):
        idx = np.argmin(np.abs(gt_ts - t))
        if abs(gt_ts[idx] - t) < 0.15:
            matched_est.append(est_xyz[i])
            matched_gt.append(gt_xyz[idx])

    if len(matched_est) < 10:
        return {"status": "too_few_matches", "n_poses": len(est),
                "n_matched": len(matched_est)}

    m_est = np.array(matched_est)
    m_gt = np.array(matched_gt)

    rmse, scale, aligned, errs = umeyama_alignment(m_est, m_gt, with_scale)

    # time coverage
    est_span = est_ts[-1] - est_ts[0]
    gt_span = gt_ts[-1] - gt_ts[0]
    coverage = min(est_span / gt_span * 100, 100) if gt_span > 0 else 0

    return {
        "status": "ok",
        "n_poses": len(est),
        "n_matched": len(matched_est),
        "ate_rmse": float(rmse),
        "ate_mean": float(errs.mean()),
        "ate_median": float(np.median(errs)),
        "ate_max": float(errs.max()),
        "scale": float(scale),
        "coverage_pct": float(coverage),
        "trajectory_length_est": float(np.sum(np.linalg.norm(np.diff(m_est, axis=0), axis=1))),
        "trajectory_length_gt": float(np.sum(np.linalg.norm(np.diff(m_gt, axis=0), axis=1))),
    }


# PHASE A: CALIBRATION AND DATA PREPARATION

def patch_orbslam3_remove_usleep():
    """Skip usleep patching, original real-time delay is required.

    ORB-SLAM3 needs the full usleep delay for thread synchronization.
    Removing or reducing usleep causes SIGSEGV crashes due to the backend
    (local mapping, loop closing) threads being overwhelmed by rapid frame
    submission. At 5fps the processing runs at real-time speed anyway.
    """
    log("A0: Skipping usleep patch (original delay required for stability)")
    return True


def run_colmap_calibration(cam_id, session, n_images=120):
    """Run COLMAP SfM with OPENCV_FISHEYE model to get camera intrinsics.

    Returns dict with fx, fy, cx, cy, k1-k4 or None on failure.
    """
    log(f"  COLMAP calibration for Cam{cam_id} ({n_images} images)...")
    work_dir = Path(f"/tmp/colmap_calib_cam{cam_id}")
    if work_dir.exists():
        shutil.rmtree(work_dir)
    img_dir = work_dir / "images"
    img_dir.mkdir(parents=True)
    db_path = work_dir / "database.db"
    sparse_dir = work_dir / "sparse"
    sparse_dir.mkdir()

    # prepare images (grayscale, half-res PNG)
    src = NCLT_DATA / "images" / session / "lb3" / f"Cam{cam_id}"
    tiffs = sorted(src.glob("*.tiff"))[200:]  # skip overexposed startup
    step = max(1, len(tiffs) // n_images)
    selected = tiffs[::step][:n_images]

    for tiff in selected:
        img = cv2.imread(str(tiff), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        img = cv2.resize(img, (808, 616), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(img_dir / f"{tiff.stem}.png"), img)

    n_prepared = len(list(img_dir.glob("*.png")))
    log(f"    Prepared {n_prepared} images")
    if n_prepared < 30:
        log("    Too few images for COLMAP", "WARN")
        return None

    try:
        # feature extraction
        t0 = time.time()
        subprocess.run([
            "colmap", "feature_extractor",
            "--database_path", str(db_path),
            "--image_path", str(img_dir),
            "--ImageReader.camera_model", "OPENCV_FISHEYE",
            "--ImageReader.single_camera", "1",
            "--SiftExtraction.max_num_features", "5000",
        ], check=True, capture_output=True, timeout=600)
        log(f"    Feature extraction: {time.time()-t0:.0f}s")

        # sequential matching
        t0 = time.time()
        subprocess.run([
            "colmap", "sequential_matcher",
            "--database_path", str(db_path),
            "--SequentialMatching.overlap", "5",
        ], check=True, capture_output=True, timeout=600)
        log(f"    Matching: {time.time()-t0:.0f}s")

        # mapper
        t0 = time.time()
        subprocess.run([
            "colmap", "mapper",
            "--database_path", str(db_path),
            "--image_path", str(img_dir),
            "--output_path", str(sparse_dir),
            "--Mapper.ba_refine_focal_length", "1",
            "--Mapper.ba_refine_extra_params", "1",
        ], check=True, capture_output=True, timeout=1800)
        log(f"    Mapper: {time.time()-t0:.0f}s")

        # read camera parameters from model
        model_dir = sparse_dir / "0"
        if not model_dir.exists():
            log("    No reconstruction produced", "WARN")
            return None

        # export to text format
        subprocess.run([
            "colmap", "model_converter",
            "--input_path", str(model_dir),
            "--output_path", str(model_dir),
            "--output_type", "TXT",
        ], check=True, capture_output=True, timeout=60)

        cameras_txt = model_dir / "cameras.txt"
        if not cameras_txt.exists():
            log("    cameras.txt not found", "WARN")
            return None

        # parse cameras.txt
        with open(cameras_txt) as f:
            for line in f:
                line = line.strip()
                if line.startswith("#") or not line:
                    continue
                parts = line.split()
                if len(parts) >= 12 and parts[1] == "OPENCV_FISHEYE":
                    # CAMERA_ID MODEL WIDTH HEIGHT fx fy cx cy k1 k2 k3 k4
                    calib = {
                        "fx": float(parts[4]), "fy": float(parts[5]),
                        "cx": float(parts[6]), "cy": float(parts[7]),
                        "k1": float(parts[8]), "k2": float(parts[9]),
                        "k3": float(parts[10]), "k4": float(parts[11]),
                        "source": "colmap",
                    }
                    log(f"    COLMAP result: fx={calib['fx']:.1f} fy={calib['fy']:.1f} "
                        f"cx={calib['cx']:.1f} cy={calib['cy']:.1f} "
                        f"k1={calib['k1']:.6f} k2={calib['k2']:.6f}")
                    return calib

        log("    No OPENCV_FISHEYE camera found in output", "WARN")
        return None

    except subprocess.TimeoutExpired:
        log("    COLMAP timed out", "WARN")
        return None
    except subprocess.CalledProcessError as e:
        log(f"    COLMAP failed: {e}", "WARN")
        return None
    except Exception as e:
        log(f"    COLMAP error: {e}", "WARN")
        return None


def characterize_imu_noise(session):
    """Estimate IMU noise parameters from static segments.

    Returns dict with gyro_noise, accel_noise, gyro_walk, accel_walk.
    """
    log("  Characterizing IMU noise from static segments...")
    imu = load_ms25_imu(session)
    odom = load_odometry(session, hz=100)

    # find static segments: where wheel odometry velocity is near zero
    # velocity ≈ diff(position) / diff(time)
    dt = np.diff(odom[:, 0]) / 1e6  # seconds
    dx = np.linalg.norm(np.diff(odom[:, 1:4], axis=0), axis=1)
    vel = dx / np.maximum(dt, 1e-6)

    # static: velocity < 0.05 m/s for at least 2 seconds
    static_mask = vel < 0.05
    static_utimes = odom[:-1, 0][static_mask]

    if len(static_utimes) < 100:
        log("    Not enough static data, using defaults", "WARN")
        return DEFAULT_IMU_NOISE

    # find IMU samples during static periods
    imu_ts = imu[:, 0]
    # use first static segment (at start of session)
    t_start = static_utimes[0]
    # find end of first static segment
    breaks = np.where(np.diff(static_utimes) > 1e6)[0]  # >1s gap
    t_end = static_utimes[breaks[0]] if len(breaks) > 0 else static_utimes[min(500, len(static_utimes)-1)]

    mask = (imu_ts >= t_start) & (imu_ts <= t_end)
    static_imu = imu[mask]
    log(f"    Found {len(static_imu)} static IMU samples ({(t_end-t_start)/1e6:.1f}s)")

    if len(static_imu) < 50:
        log("    Static segment too short, using defaults", "WARN")
        return DEFAULT_IMU_NOISE

    # compute noise (standard deviation of static readings)
    accel = static_imu[:, 4:7]  # accel_x, accel_y, accel_z
    gyro = static_imu[:, 7:10]  # rot_x, rot_y, rot_z

    dt_imu = np.mean(np.diff(static_imu[:, 0])) / 1e6  # mean IMU period in seconds
    rate = 1.0 / dt_imu if dt_imu > 0 else 47.0

    # noise density = std / sqrt(rate)
    gyro_std = np.std(gyro, axis=0)
    accel_std = np.std(accel, axis=0)

    gyro_noise = float(np.mean(gyro_std) / np.sqrt(rate))
    accel_noise = float(np.mean(accel_std) / np.sqrt(rate))

    # Random walk: use conservative estimates (hard to measure from short static segment)
    gyro_walk = gyro_noise * 0.1
    accel_walk = accel_noise * 0.1

    # clamp to reasonable ranges
    gyro_noise = max(min(gyro_noise, 0.05), 1e-5)
    accel_noise = max(min(accel_noise, 0.5), 1e-4)
    gyro_walk = max(min(gyro_walk, 0.005), 1e-6)
    accel_walk = max(min(accel_walk, 0.05), 1e-5)

    result = {
        "gyro_noise": gyro_noise,
        "accel_noise": accel_noise,
        "gyro_walk": gyro_walk,
        "accel_walk": accel_walk,
        "gyro_std_xyz": gyro_std.tolist(),
        "accel_std_xyz": accel_std.tolist(),
        "imu_rate_hz": float(rate),
        "static_duration_s": float((t_end - t_start) / 1e6),
    }
    log(f"    IMU rate: {rate:.1f} Hz")
    log(f"    Gyro noise: {gyro_noise:.6f} rad/s/√Hz, Accel noise: {accel_noise:.6f} m/s²/√Hz")
    log(f"    Gyro walk: {gyro_walk:.6f}, Accel walk: {accel_walk:.6f}")
    return result


def prepare_imu_csv(session, version, image_utimes, output_path):
    """Prepare IMU data in EuRoC CSV format.

    Versions:
        v1: MS25 raw at ~47 Hz
        v2: MS25 interpolated to 100 Hz
        v3: MS25 interpolated to 200 Hz
        v4: Hybrid (MS25 accel + KVH yaw rate, MS25 roll/pitch gyro) at 100 Hz
        v5: Hybrid interpolated to 200 Hz

    Format: timestamp_ns, gx, gy, gz, ax, ay, az

    Returns number of IMU samples written.
    """
    imu = load_ms25_imu(session)
    imu_ts = imu[:, 0]  # microseconds
    accel = imu[:, 4:7]  # accel_x, accel_y, accel_z
    gyro = imu[:, 7:10]  # rot_x, rot_y, rot_z

    # trim to cover image range with 2s margin
    t_min = min(image_utimes) - 2e6
    t_max = max(image_utimes) + 2e6
    mask = (imu_ts >= t_min) & (imu_ts <= t_max)
    imu_ts = imu_ts[mask]
    accel = accel[mask]
    gyro = gyro[mask]

    if version in ("v4", "v5"):
        # load KVH FOG and compute yaw rate
        kvh = load_kvh(session)
        kvh_ts = kvh[:, 0]
        kvh_heading = kvh[:, 1]

        # KVH yaw rate = d(heading)/dt
        kvh_dt = np.diff(kvh_ts) / 1e6  # seconds
        kvh_yaw_rate = np.diff(kvh_heading) / kvh_dt
        # unwrap large jumps (heading wraps at ±π)
        kvh_yaw_rate = np.clip(kvh_yaw_rate, -10, 10)
        kvh_ts_mid = (kvh_ts[:-1] + kvh_ts[1:]) / 2

        # trim to range
        kvh_mask = (kvh_ts_mid >= t_min) & (kvh_ts_mid <= t_max)
        kvh_ts_mid = kvh_ts_mid[kvh_mask]
        kvh_yaw_rate = kvh_yaw_rate[kvh_mask]

    if version == "v1":
        # raw MS25 at ~47 Hz
        out_ts = imu_ts
        out_gyro = gyro
        out_accel = accel
    elif version in ("v2", "v3"):
        target_rate = 100 if version == "v2" else 200
        # interpolate to target rate
        t_new = np.arange(imu_ts[0], imu_ts[-1], 1e6 / target_rate)
        interp_g = interp1d(imu_ts, gyro, axis=0, kind='linear', fill_value='extrapolate')
        interp_a = interp1d(imu_ts, accel, axis=0, kind='linear', fill_value='extrapolate')
        out_ts = t_new
        out_gyro = interp_g(t_new)
        out_accel = interp_a(t_new)
    elif version in ("v4", "v5"):
        target_rate = 100 if version == "v4" else 200
        t_new = np.arange(imu_ts[0], imu_ts[-1], 1e6 / target_rate)

        interp_g = interp1d(imu_ts, gyro, axis=0, kind='linear', fill_value='extrapolate')
        interp_a = interp1d(imu_ts, accel, axis=0, kind='linear', fill_value='extrapolate')
        out_gyro = interp_g(t_new)
        out_accel = interp_a(t_new)

        # replace yaw (z) gyro component with KVH yaw rate
        if len(kvh_ts_mid) > 10:
            interp_kvh = interp1d(kvh_ts_mid, kvh_yaw_rate, kind='linear',
                                  fill_value='extrapolate')
            out_gyro[:, 2] = interp_kvh(t_new)

        out_ts = t_new
    else:
        raise ValueError(f"Unknown IMU version: {version}")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                "a_RS_S_z [m s^-2]\n")
        for i in range(len(out_ts)):
            ts_ns = int(out_ts[i] * 1000)  # us → ns
            f.write(f"{ts_ns},{out_gyro[i,0]:.12f},{out_gyro[i,1]:.12f},"
                    f"{out_gyro[i,2]:.12f},{out_accel[i,0]:.12f},"
                    f"{out_accel[i,1]:.12f},{out_accel[i,2]:.12f}\n")

    return len(out_ts)


def prepare_euroc_dir(session, cam_ids, n_frames, imu_csv_path,
                      preprocess="none", clahe_clip=2.0, output_base=None):
    """prepare EuRoC-format dir for ORB-SLAM3, returns (euroc_dir, timestamps_path, utimes) or None"""
    cam_tag = "_".join(f"cam{c}" for c in cam_ids)
    pre_tag = f"_clahe{clahe_clip}" if preprocess == "clahe" else ""
    n_tag = f"_n{n_frames}" if n_frames > 0 else ""
    tag = f"{session}_{cam_tag}{pre_tag}{n_tag}"

    if output_base is None:
        output_base = Path(f"/tmp/nclt_euroc_{tag}")
    output_base = Path(output_base)

    ts_path = output_base / "timestamps.txt"

    # always update IMU symlink (different runs may use different IMU versions)
    imu_link = output_base / "mav0" / "imu0" / "data.csv"
    imu_link.parent.mkdir(parents=True, exist_ok=True)
    if imu_link.exists() or imu_link.is_symlink():
        imu_link.unlink()
    imu_link.symlink_to(Path(imu_csv_path).resolve())

    # check if images already prepared
    if ts_path.exists():
        with open(ts_path) as f:
            existing = sum(1 for line in f if line.strip())
        if existing > 0:
            # print(f"DEBUG config={cfg}")
            log(f"  EuRoC dir already prepared: {existing} frames, IMU→{Path(imu_csv_path).name}")
            utimes = []
            with open(ts_path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        utimes.append(int(line) // 1000)  # ns → us
            return output_base, ts_path, utimes

    clahe = None
    if preprocess == "clahe":
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))

    image_utimes = []
    for ci, cam_id in enumerate(cam_ids):
        cam_dir_name = f"cam{ci}"  # cam0, cam1
        out_img_dir = output_base / "mav0" / cam_dir_name / "data"
        out_img_dir.mkdir(parents=True, exist_ok=True)

        src_dir = NCLT_DATA / "images" / session / "lb3" / f"Cam{cam_id}"
        tiffs = sorted(src_dir.glob("*.tiff"))
        skip = min(200, len(tiffs) // 10)
        tiffs = tiffs[skip:]
        if n_frames > 0:
            tiffs = tiffs[:n_frames]

        log(f"  Preparing {len(tiffs)} frames from Cam{cam_id} as {cam_dir_name}...")
        t0 = time.time()

        cam_utimes = []
        for i, tiff in enumerate(tiffs):
            utime_us = int(tiff.stem)
            ts_ns = utime_us * 1000

            out_path = out_img_dir / f"{ts_ns}.png"
            if not out_path.exists():
                img = cv2.imread(str(tiff), cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                img = cv2.resize(img, (808, 616), interpolation=cv2.INTER_AREA)
                if clahe is not None:
                    img = clahe.apply(img)
                cv2.imwrite(str(out_path), img)

            cam_utimes.append(utime_us)

            if (i + 1) % 3000 == 0:
                elapsed = time.time() - t0
                log(f"    [{i+1}/{len(tiffs)}] {(i+1)/elapsed:.0f} img/s")

        elapsed = time.time() - t0
        log(f"    Prepared {len(cam_utimes)} frames in {elapsed:.0f}s")

        if ci == 0:
            image_utimes = cam_utimes

    # write timestamps file
    ts_path.parent.mkdir(parents=True, exist_ok=True)
    with open(ts_path, "w") as f:
        for ut in image_utimes:
            f.write(f"{ut * 1000}\n")  # nanoseconds

    # symlink IMU data
    imu_link.parent.mkdir(parents=True, exist_ok=True)
    if imu_link.exists() or imu_link.is_symlink():
        imu_link.unlink()
    imu_link.symlink_to(Path(imu_csv_path).resolve())

    return output_base, ts_path, image_utimes


# CONFIG GENERATION

def write_mono_inertial_config(output_path, cam_calib, imu_noise, Tbc,
                                imu_freq=47.0, cam_fps=5,
                                n_features=3000, scale_factor=1.2,
                                n_levels=10, ini_th_fast=20, min_th_fast=7):
    """write ORB-SLAM3 mono-inertial config YAML (KannalaBrandt8 fisheye)"""
    c = cam_calib
    n = imu_noise
    # format Tbc as comma-separated row-major
    tbc_data = ", ".join(f"{v:.10f}" for v in Tbc.flatten())

    content = f"""%YAML:1.0

File.version: "1.0"

Camera.type: "KannalaBrandt8"

Camera1.fx: {c['fx']:.6f}
Camera1.fy: {c['fy']:.6f}
Camera1.cx: {c['cx']:.6f}
Camera1.cy: {c['cy']:.6f}

Camera1.k1: {c['k1']:.10f}
Camera1.k2: {c['k2']:.10f}
Camera1.k3: {c['k3']:.10f}
Camera1.k4: {c['k4']:.10f}

Camera.width: 808
Camera.height: 616

Camera.fps: {cam_fps}

Camera.RGB: 0

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{tbc_data}]

IMU.NoiseGyro: {n['gyro_noise']:.8f}
IMU.NoiseAcc: {n['accel_noise']:.8f}
IMU.GyroWalk: {n['gyro_walk']:.8f}
IMU.AccWalk: {n['accel_walk']:.8f}
IMU.Frequency: {imu_freq:.1f}

ORBextractor.nFeatures: {n_features}
ORBextractor.scaleFactor: {scale_factor}
ORBextractor.nLevels: {n_levels}
ORBextractor.iniThFAST: {ini_th_fast}
ORBextractor.minThFAST: {min_th_fast}

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
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(content)
    return output_path


def write_stereo_inertial_config(output_path, cam_calib_left, cam_calib_right,
                                  imu_noise, Tbc, Tc1c2,
                                  imu_freq=47.0, cam_fps=5,
                                  n_features=3000, th_depth=200.0):
    """Write ORB-SLAM3 stereo-inertial config YAML (KannalaBrandt8 fisheye)."""
    cl = cam_calib_left
    cr = cam_calib_right
    n = imu_noise
    tbc_data = ", ".join(f"{v:.10f}" for v in Tbc.flatten())
    tc1c2_data = ", ".join(f"{v:.10f}" for v in Tc1c2.flatten())

    content = f"""%YAML:1.0

File.version: "1.0"

Camera.type: "KannalaBrandt8"

Camera1.fx: {cl['fx']:.6f}
Camera1.fy: {cl['fy']:.6f}
Camera1.cx: {cl['cx']:.6f}
Camera1.cy: {cl['cy']:.6f}

Camera1.k1: {cl['k1']:.10f}
Camera1.k2: {cl['k2']:.10f}
Camera1.k3: {cl['k3']:.10f}
Camera1.k4: {cl['k4']:.10f}

Camera2.fx: {cr['fx']:.6f}
Camera2.fy: {cr['fy']:.6f}
Camera2.cx: {cr['cx']:.6f}
Camera2.cy: {cr['cy']:.6f}

Camera2.k1: {cr['k1']:.10f}
Camera2.k2: {cr['k2']:.10f}
Camera2.k3: {cr['k3']:.10f}
Camera2.k4: {cr['k4']:.10f}

Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [{tc1c2_data}]

Camera1.overlappingBegin: 0
Camera1.overlappingEnd: 807

Camera2.overlappingBegin: 0
Camera2.overlappingEnd: 807

Camera.width: 808
Camera.height: 616

Camera.fps: {cam_fps}

Camera.RGB: 0

Stereo.ThDepth: {th_depth:.1f}

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{tbc_data}]

IMU.NoiseGyro: {n['gyro_noise']:.8f}
IMU.NoiseAcc: {n['accel_noise']:.8f}
IMU.GyroWalk: {n['gyro_walk']:.8f}
IMU.AccWalk: {n['accel_walk']:.8f}
IMU.Frequency: {imu_freq:.1f}

ORBextractor.nFeatures: {n_features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 10
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

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
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(content)
    return output_path


def write_mono_inertial_pinhole_config(output_path, imu_noise, Tbc,
                                        imu_freq=47.0):
    """Write mono-inertial config with PinHole model (f=221) for comparison"""
    n = imu_noise
    tbc_data = ", ".join(f"{v:.10f}" for v in Tbc.flatten())

    content = f"""%YAML:1.0

File.version: "1.0"

Camera.type: "PinHole"

Camera1.fx: 221.0
Camera1.fy: 221.0
Camera1.cx: 404.0
Camera1.cy: 308.0

Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0

Camera.width: 808
Camera.height: 616

Camera.fps: 5

Camera.RGB: 0

IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [{tbc_data}]

IMU.NoiseGyro: {n['gyro_noise']:.8f}
IMU.NoiseAcc: {n['accel_noise']:.8f}
IMU.GyroWalk: {n['gyro_walk']:.8f}
IMU.AccWalk: {n['accel_walk']:.8f}
IMU.Frequency: {imu_freq:.1f}

ORBextractor.nFeatures: 3000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 10
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

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
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        f.write(content)
    return output_path


# RUN ORB-SLAM3

def run_orbslam3(mode, config_path, euroc_dir, timestamps_path,
                 timeout_s=SWEEP_TIMEOUT, output_dir=None, label=""):
    """run ORB-SLAM3 (mono_inertial or stereo_inertial), returns result dict"""
    binary = MONO_INERTIAL_BIN if mode == "mono_inertial" else STEREO_INERTIAL_BIN
    result = {
        "success": False, "traj_path": None, "n_keyframes": 0,
        "n_frames": 0, "tracking_rate": 0, "runtime_s": 0,
        "mode": mode, "label": label, "stdout_tail": "",
    }

    if not binary.exists():
        log(f"  Binary not found: {binary}", "ERROR")
        return result

    if output_dir is None:
        output_dir = Path(euroc_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # count input frames
    n_input = 0
    with open(timestamps_path) as f:
        n_input = sum(1 for line in f if line.strip())

    # clean old trajectory files
    for candidate in ["CameraTrajectory.txt", "KeyFrameTrajectory.txt",
                      "f_nclt.txt", "kf_nclt.txt"]:
        p = output_dir / candidate
        if p.exists():
            p.unlink()

    cmd = [
        "xvfb-run", "-a",
        str(binary),
        str(VOCAB_PATH),
        str(config_path),
        str(euroc_dir),
        str(timestamps_path),
        "nclt",  # file_name suffix for trajectory output
    ]

    env = orbslam_env()
    tag = f" [{label}]" if label else ""
    log(f"  Running {mode}{tag} ({n_input} frames, timeout={timeout_s}s)...")

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout_s, env=env, cwd=str(output_dir),
        )
        result["runtime_s"] = time.time() - t0
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""
        result["stdout_tail"] = "\n".join(stdout.strip().split("\n")[-20:])

        # check for IMU initialization
        imu_init = "IMU" in stdout and ("initialized" in stdout.lower() or "init" in stdout.lower())
        result["imu_initialized"] = imu_init

        # find trajectory file
        traj_path = None
        for candidate in ["f_nclt.txt", "CameraTrajectory.txt",
                          "kf_nclt.txt", "KeyFrameTrajectory.txt"]:
            p = output_dir / candidate
            if p.exists() and p.stat().st_size > 100:
                traj_path = p
                break

        if traj_path:
            result["traj_path"] = str(traj_path)
            result["success"] = True

            poses = load_tum_trajectory(traj_path)
            result["n_keyframes"] = len(poses)
            result["n_frames"] = n_input
            result["tracking_rate"] = len(poses) / max(n_input, 1) * 100

            log(f"    OK: {len(poses)} poses / {n_input} frames "
                f"({result['tracking_rate']:.1f}%) in {result['runtime_s']:.0f}s"
                f" IMU_init={'Y' if imu_init else 'N'}")
        else:
            log(f"    FAIL: No trajectory (rc={proc.returncode})")
            if stderr:
                for line in stderr.strip().split("\n")[-5:]:
                    log(f"    STDERR: {line}")

    except subprocess.TimeoutExpired:
        result["runtime_s"] = time.time() - t0
        log(f"    TIMEOUT after {timeout_s}s")
    except Exception as e:
        result["runtime_s"] = time.time() - t0
        log(f"    ERROR: {e}", "ERROR")

    return result


# PHASE A: CALIBRATION AND DATA PREP

def phase_a(session="2012-04-29"):
    """Phase A: Calibration and data preparation.

    Returns dict with calibrations, IMU paths, etc.
    """
    log("PHASE A: CALIBRATION AND DATA PREPARATION")

    CALIB_DIR.mkdir(parents=True, exist_ok=True)
    IMU_DIR.mkdir(parents=True, exist_ok=True)

    calib = {"cam_calib": {}, "imu_noise": {}, "imu_paths": {}, "Tbc": {}}

    # a0: Patch ORB-SLAM3 (remove usleep)
    patch_orbslam3_remove_usleep()

    # a1: COLMAP fisheye calibration for Cam1, Cam4, Cam5
    log("\nA1: COLMAP fisheye calibration")
    for cam_id in [5, 4, 1]:
        log(f"\n--- Cam{cam_id} ---")
        colmap_result = run_colmap_calibration(cam_id, session, n_images=120)

        if colmap_result is not None:
            calib["cam_calib"][cam_id] = colmap_result
            log(f"  Cam{cam_id}: COLMAP success")
        else:
            calib["cam_calib"][cam_id] = DEFAULT_FISHEYE.copy()
            calib["cam_calib"][cam_id]["source"] = "default"
            log(f"  Cam{cam_id}: Using default calibration (COLMAP failed)")

        save_json(calib["cam_calib"][cam_id],
                  CALIB_DIR / f"cam{cam_id}_fisheye.json")

    # a2: Compute Tbc matrices for each camera
    log("\nA2: Computing body-to-camera transforms (Tbc)")
    for cam_id, angle in CAM_ANGLES.items():
        Tbc = compute_Tbc(angle, cam_id)
        calib["Tbc"][cam_id] = Tbc.tolist()
        log(f"  Cam{cam_id} (θ={angle:+.0f}°): t=[{Tbc[0,3]:.3f}, {Tbc[1,3]:.3f}, {Tbc[2,3]:.3f}]")
        np.savetxt(str(CALIB_DIR / f"Tbc_cam{cam_id}.txt"), Tbc, fmt="%.10f")

    # a3: IMU noise characterization
    log("\nA3: IMU noise characterization")
    imu_noise = characterize_imu_noise(session)
    calib["imu_noise"] = imu_noise
    save_json(imu_noise, CALIB_DIR / "imu_noise.json")

    # a4: Prepare IMU data (5 versions)
    log("\nA4: Preparing IMU data (5 versions)")
    image_utimes = get_image_utimes(session, 5, n_frames=0, skip_start=200)
    # print("DEBUG: entering main loop")
    log(f"  Image time range: {min(image_utimes)/1e6:.1f}s - {max(image_utimes)/1e6:.1f}s")

    imu_versions = {
        "v1": "MS25 raw ~47Hz",
        "v2": "MS25 interpolated 100Hz",
        "v3": "MS25 interpolated 200Hz",
        "v4": "Hybrid (MS25+KVH) 100Hz",
        "v5": "Hybrid (MS25+KVH) 200Hz",
    }
    for ver, desc in imu_versions.items():
        path = IMU_DIR / f"imu_{ver}.csv"
        if path.exists():
            log(f"  {ver} ({desc}): already exists")
        else:
            n_samples = prepare_imu_csv(session, ver, image_utimes, path)
            log(f"  {ver} ({desc}): {n_samples} samples")
        calib["imu_paths"][ver] = str(path)

    # save calibration summary
    save_json(calib, RESULTS_DIR / "calibration_summary.json")
    log("\nPhase A complete")
    return calib


# PHASE B: MONO-INERTIAL SWEEP

def phase_b(session, calib):
    # NOTE: not thread-safe but we run single threaded anyway
    """Phase B: Mono-inertial sweep on 3000 frames.

    Tests IMU rates, camera models, cameras, and parameters.
    Returns list of all run results.
    """
    log("\n" + "=" * 70)
    log("PHASE B: MONO-INERTIAL SWEEP (3000 frames)")

    phase_dir = RESULTS_DIR / "phase_b"
    phase_dir.mkdir(parents=True, exist_ok=True)

    gt_data = load_ground_truth(session)
    all_results = []
    run_num = 0

    imu_freq_map = {"v1": 47.0, "v2": 100.0, "v3": 200.0, "v4": 100.0, "v5": 200.0}

    # helper to run one config
    def do_run(label, cam_id, imu_ver, cam_model="fisheye", preprocess="none",
               clahe_clip=2.0, n_features=3000, repeat=1):
        nonlocal run_num
        results = []
        for rep in range(repeat):
            run_num += 1
            run_label = f"B{run_num:03d}_{label}" + (f"_r{rep+1}" if repeat > 1 else "")
            run_dir = phase_dir / run_label
            run_dir.mkdir(parents=True, exist_ok=True)

            log(f"\n--- Run {run_num}: {run_label} ---")

            # prepare EuRoC directory
            imu_path = calib["imu_paths"][imu_ver]
            euroc_dir, ts_path, img_utimes = prepare_euroc_dir(
                session, [cam_id], N_SWEEP_FRAMES, imu_path,
                preprocess=preprocess, clahe_clip=clahe_clip)

            if euroc_dir is None:
                log("  Failed to prepare data", "ERROR")
                continue

            # write config
            config_path = run_dir / "config.yaml"
            cam_calib = calib["cam_calib"][cam_id]
            imu_noise = calib["imu_noise"]
            Tbc = np.array(calib["Tbc"][cam_id])

            if cam_model == "fisheye":
                write_mono_inertial_config(
                    config_path, cam_calib, imu_noise, Tbc,
                    imu_freq=imu_freq_map[imu_ver],
                    n_features=n_features)
            else:
                write_mono_inertial_pinhole_config(
                    config_path, imu_noise, Tbc,
                    imu_freq=imu_freq_map[imu_ver])

            # run
            result = run_orbslam3(
                "mono_inertial", config_path, euroc_dir, ts_path,
                timeout_s=SWEEP_TIMEOUT, output_dir=run_dir, label=run_label)

            result["config"] = {
                "cam_id": cam_id, "imu_ver": imu_ver, "cam_model": cam_model,
                "preprocess": preprocess, "n_features": n_features,
            }

            # evaluate
            if result["success"] and result["traj_path"]:
                eval_result = evaluate_trajectory(result["traj_path"], gt_data, with_scale=True)
                result["eval"] = eval_result
                if eval_result.get("status") == "ok":
                    log(f"    ATE: {eval_result['ate_rmse']:.1f}m (scale={eval_result['scale']:.3f})")
            else:
                result["eval"] = {"status": "no_trajectory"}

            save_json(result, run_dir / "result.json")
            results.append(result)
            all_results.append(result)

        return results

    # ---- B1: IMU rate sweep (Cam5 fisheye) ----
    log("\n=== B1: IMU Rate Sweep (Cam5 fisheye) ===")
    best_imu = "v1"
    best_tracking = 0

    for imu_ver in ["v1", "v2", "v3", "v4", "v5"]:
        res = do_run(f"imu_{imu_ver}", cam_id=5, imu_ver=imu_ver)
        if res and res[0].get("tracking_rate", 0) > best_tracking:
            best_tracking = res[0]["tracking_rate"]
            best_imu = imu_ver

    log(f"\n  Best IMU: {best_imu} ({best_tracking:.1f}% tracking)")

    # ---- B2: Camera model comparison (fisheye vs pinhole) ----
    log("\n=== B2: Camera Model Comparison ===")
    do_run("cam5_pinhole", cam_id=5, imu_ver=best_imu, cam_model="pinhole")

    # ---- B3: Camera comparison (Cam5, Cam4, Cam1) ----
    log("\n=== B3: Camera Comparison ===")
    for cam_id in [4, 1]:
        do_run(f"cam{cam_id}_fisheye", cam_id=cam_id, imu_ver=best_imu)

    # ---- B4: CLAHE + nFeatures sweep ----
    log("\n=== B4: CLAHE and nFeatures Sweep ===")
    do_run("cam5_clahe", cam_id=5, imu_ver=best_imu, preprocess="clahe")
    do_run("cam5_nfeat5000", cam_id=5, imu_ver=best_imu, n_features=5000)

    # summary table
    log("\n=== Phase B Summary ===")
    log(f"{'Run':<35} {'Track%':>7} {'KF':>5} {'ATE':>8} {'Scale':>7} {'IMU':>4} {'Time':>6}")
    for r in all_results:
        label = r.get("label", "?")[:35]
        tr = f"{r.get('tracking_rate', 0):.1f}%"
        kf = str(r.get("n_keyframes", 0))
        ate = f"{r['eval']['ate_rmse']:.1f}m" if r.get("eval", {}).get("status") == "ok" else "N/A"
        sc = f"{r['eval']['scale']:.3f}" if r.get("eval", {}).get("status") == "ok" else "N/A"
        imu_init = "Y" if r.get("imu_initialized") else "N"
        rt = f"{r.get('runtime_s', 0):.0f}s"
        log(f"{label:<35} {tr:>7} {kf:>5} {ate:>8} {sc:>7} {imu_init:>4} {rt:>6}")

    # find best config
    best_result = max(all_results, key=lambda r: r.get("tracking_rate", 0))
    log(f"\nBest: {best_result.get('label', '?')} "
        f"({best_result.get('tracking_rate', 0):.1f}% tracking)")

    save_json({"runs": all_results, "best_imu": best_imu,
               "best_label": best_result.get("label")},
              phase_dir / "summary.json")

    return all_results, best_imu


# PHASE C: STEREO-INERTIAL

def phase_c(session, calib, best_imu):
    """Phase C: Stereo-inertial on 3000 frames"""
    log("\n" + "=" * 70)
    log("PHASE C: STEREO-INERTIAL (3000 frames)")

    phase_dir = RESULTS_DIR / "phase_c"
    phase_dir.mkdir(parents=True, exist_ok=True)
    gt_data = load_ground_truth(session)
    all_results = []

    imu_freq_map = {"v1": 47.0, "v2": 100.0, "v3": 200.0, "v4": 100.0, "v5": 200.0}

    # stereo pairs to test
    stereo_pairs = [
        (4, 5, "cam4_cam5"),   # Left + Forward
        (5, 1, "cam5_cam1"),   # Forward + Right
    ]

    for left_id, right_id, pair_label in stereo_pairs:
        run_label = f"C_{pair_label}"
        run_dir = phase_dir / run_label
        run_dir.mkdir(parents=True, exist_ok=True)
        log(f"\n--- Stereo: Cam{left_id} + Cam{right_id} ---")

        # prepare EuRoC directory with both cameras
        imu_path = calib["imu_paths"][best_imu]
        euroc_dir, ts_path, img_utimes = prepare_euroc_dir(
            session, [left_id, right_id], N_SWEEP_FRAMES, imu_path)

        if euroc_dir is None:
            log("  Failed to prepare stereo data", "ERROR")
            continue

        # compute stereo geometry
        left_angle = CAM_ANGLES[left_id]
        right_angle = CAM_ANGLES[right_id]
        Tbc_left = compute_Tbc(left_angle, left_id)
        Tc1c2 = compute_Tc1c2(left_angle, left_id, right_angle, right_id)

        baseline = np.linalg.norm(Tc1c2[:3, 3])
        log(f"  Baseline: {baseline*100:.1f} cm")

        # write config
        config_path = run_dir / "config.yaml"
        write_stereo_inertial_config(
            config_path,
            calib["cam_calib"][left_id],
            calib["cam_calib"][right_id],
            calib["imu_noise"], Tbc_left, Tc1c2,
            imu_freq=imu_freq_map[best_imu])

        # run
        result = run_orbslam3(
            "stereo_inertial", config_path, euroc_dir, ts_path,
            timeout_s=SWEEP_TIMEOUT, output_dir=run_dir, label=run_label)

        result["config"] = {"left": left_id, "right": right_id,
                            "imu_ver": best_imu, "baseline_m": float(baseline)}

        # evaluate with SE(3) alignment because stereo already has metric scale
        # (no 7-DoF Sim(3) scale recovery needed, just 6-DoF rigid).
        if result["success"] and result["traj_path"]:
            eval_result = evaluate_trajectory(result["traj_path"], gt_data, with_scale=False)
            result["eval"] = eval_result
            if eval_result.get("status") == "ok":
                log(f"    ATE (SE3): {eval_result['ate_rmse']:.1f}m")

            # also try Sim(3) for comparison
            eval_sim3 = evaluate_trajectory(result["traj_path"], gt_data, with_scale=True)
            result["eval_sim3"] = eval_sim3
        else:
            result["eval"] = {"status": "no_trajectory"}

        save_json(result, run_dir / "result.json")
        all_results.append(result)

    # summary
    log("\n=== Phase C Summary ===")
    for r in all_results:
        label = r.get("label", "?")
        tr = r.get("tracking_rate", 0)
        ate = r.get("eval", {}).get("ate_rmse", float("inf"))
        log(f"  {label}: {tr:.1f}% tracking, ATE={ate:.1f}m")

    save_json(all_results, phase_dir / "summary.json")
    return all_results


# PHASE D: FULL SPRING SESSION

def phase_d(session, calib, best_config):
    """Phase D: Run best config on full spring session"""
    log("\n" + "=" * 70)
    log("PHASE D: FULL SPRING SESSION")

    phase_dir = RESULTS_DIR / "phase_d"
    phase_dir.mkdir(parents=True, exist_ok=True)
    gt_data = load_ground_truth(session)
    all_results = []

    imu_freq_map = {"v1": 47.0, "v2": 100.0, "v3": 200.0, "v4": 100.0, "v5": 200.0}

    # run top 2 configs from Phase B on full session
    configs_to_try = best_config if isinstance(best_config, list) else [best_config]

    for ci, cfg in enumerate(configs_to_try[:2]):
        cam_id = cfg.get("cam_id", 5)
        imu_ver = cfg.get("imu_ver", "v1")
        cam_model = cfg.get("cam_model", "fisheye")
        preprocess = cfg.get("preprocess", "none")
        n_features = cfg.get("n_features", 3000)

        run_label = f"D{ci+1}_full_{cam_model}_cam{cam_id}_{imu_ver}"
        run_dir = phase_dir / run_label
        run_dir.mkdir(parents=True, exist_ok=True)
        log(f"\n--- {run_label} ---")

        # prepare full session data
        imu_path = calib["imu_paths"][imu_ver]
        euroc_dir, ts_path, img_utimes = prepare_euroc_dir(
            session, [cam_id], 0, imu_path,  # 0 = all frames
            preprocess=preprocess)

        if euroc_dir is None:
            log("  Failed to prepare full session data", "ERROR")
            continue

        # write config
        config_path = run_dir / "config.yaml"
        cam_calib = calib["cam_calib"][cam_id]
        imu_noise = calib["imu_noise"]
        Tbc = np.array(calib["Tbc"][cam_id])

        if cam_model == "fisheye":
            write_mono_inertial_config(
                config_path, cam_calib, imu_noise, Tbc,
                imu_freq=imu_freq_map[imu_ver], n_features=n_features)
        else:
            write_mono_inertial_pinhole_config(
                config_path, imu_noise, Tbc, imu_freq=imu_freq_map[imu_ver])

        # run with extended timeout
        result = run_orbslam3(
            "mono_inertial", config_path, euroc_dir, ts_path,
            timeout_s=FULL_TIMEOUT, output_dir=run_dir, label=run_label)

        result["config"] = cfg

        # evaluate
        if result["success"] and result["traj_path"]:
            eval_result = evaluate_trajectory(result["traj_path"], gt_data, with_scale=True)
            result["eval"] = eval_result
            if eval_result.get("status") == "ok":
                log(f"    ATE: {eval_result['ate_rmse']:.1f}m "
                    f"(scale={eval_result['scale']:.3f}, coverage={eval_result['coverage_pct']:.0f}%)")
        else:
            result["eval"] = {"status": "no_trajectory"}

        save_json(result, run_dir / "result.json")
        all_results.append(result)

    save_json(all_results, phase_dir / "summary.json")
    return all_results


# PHASE E: WHEEL ODOMETRY FUSION

def phase_e(session, phase_d_results):
    """Phase E: Fuse best ORB-SLAM3 trajectory with wheel odometry"""
    log("\n" + "=" * 70)
    log("PHASE E: WHEEL ODOMETRY FUSION")

    phase_dir = RESULTS_DIR / "phase_e"
    phase_dir.mkdir(parents=True, exist_ok=True)
    gt_data = load_ground_truth(session)

    # find best trajectory from Phase D
    best = None
    for r in phase_d_results:
        if r.get("success") and r.get("traj_path"):
            if best is None or r.get("tracking_rate", 0) > best.get("tracking_rate", 0):
                best = r

    if best is None:
        log("  No trajectory from Phase D to fuse", "WARN")
        return None

    log(f"  Using trajectory: {best.get('label', '?')}")
    est = load_tum_trajectory(best["traj_path"])
    if len(est) < 20:
        log("  Trajectory too short for fusion", "WARN")
        return None

    # load wheel odometry
    odom = load_odometry(session, hz=100)
    odom_ts = odom[:, 0] / 1e6  # to seconds
    odom_xyz = odom[:, 1:4]
    odom_rpy = odom[:, 4:7]

    # build VIO pose graph edges from consecutive estimated poses
    est_ts = est[:, 0]
    est_xyz = est[:, 1:4]

    # For each consecutive pair of estimated poses, also get wheel odom relative transform
    n_est = len(est)
    vio_edges = []  # (i, j, dx, dy, dtheta) in frame i's local frame
    odom_edges = []

    for i in range(n_est - 1):
        # VIO relative (just 2D: x, y, yaw)
        dx_vio = est_xyz[i+1] - est_xyz[i]
        # approximate yaw from quaternion
        q = est[i, 4:8]  # qx, qy, qz, qw
        try:
            r = Rotation.from_quat(q)
            yaw_i = r.as_euler('xyz')[2]
        except Exception:
            yaw_i = 0
        c, s = np.cos(-yaw_i), np.sin(-yaw_i)
        dx_local = c * dx_vio[0] + s * dx_vio[1]
        dy_local = -s * dx_vio[0] + c * dx_vio[1]

        q2 = est[i+1, 4:8]
        try:
            yaw_j = Rotation.from_quat(q2).as_euler('xyz')[2]
        except Exception:
            yaw_j = 0
        dyaw = (yaw_j - yaw_i + np.pi) % (2 * np.pi) - np.pi

        vio_edges.append((i, i+1, dx_local, dy_local, dyaw))

        # wheel odometry relative transform at same timestamps
        # interpolate odom at est timestamps
        idx_i = np.searchsorted(odom_ts, est_ts[i])
        idx_j = np.searchsorted(odom_ts, est_ts[i+1])
        idx_i = min(max(idx_i, 0), len(odom_ts) - 1)
        idx_j = min(max(idx_j, 0), len(odom_ts) - 1)

        if abs(odom_ts[idx_i] - est_ts[i]) < 0.5 and abs(odom_ts[idx_j] - est_ts[i+1]) < 0.5:
            odom_di = odom_xyz[idx_j] - odom_xyz[idx_i]
            odom_yaw_i = odom_rpy[idx_i, 2]
            odom_yaw_j = odom_rpy[idx_j, 2]
            c2, s2 = np.cos(-odom_yaw_i), np.sin(-odom_yaw_i)
            odx = c2 * odom_di[0] + s2 * odom_di[1]
            ody = -s2 * odom_di[0] + c2 * odom_di[1]
            odyaw = (odom_yaw_j - odom_yaw_i + np.pi) % (2 * np.pi) - np.pi
            odom_edges.append((i, i+1, odx, ody, odyaw))

    log(f"  VIO edges: {len(vio_edges)}, Odom edges: {len(odom_edges)}")

    if len(odom_edges) < 10:
        log("  Too few odom edges for fusion", "WARN")
        return None

    # pose graph optimization
    sys.path.insert(0, str(PROJECT_ROOT / "src"))
    from slam.loop_closure import PoseGraphOptimizer2D

    # initial poses from VIO (2D: x, y, yaw)
    poses_2d = np.zeros((n_est, 3))
    poses_2d[:, 0] = est_xyz[:, 0]
    poses_2d[:, 1] = est_xyz[:, 1]
    for i in range(n_est):
        try:
            poses_2d[i, 2] = Rotation.from_quat(est[i, 4:8]).as_euler('xyz')[2]
        except Exception:
            poses_2d[i, 2] = 0

    optimizer = PoseGraphOptimizer2D(odom_w=1.0, lc_w=2.0)
    optimized = optimizer.optimize(poses_2d, vio_edges, odom_edges, max_iter=50)

    # convert back to TUM format (3D, using original z and quaternion)
    fused_tum = est.copy()
    fused_tum[:, 1] = optimized[:, 0]
    fused_tum[:, 2] = optimized[:, 1]

    # save fused trajectory
    fused_path = phase_dir / "fused_trajectory.txt"
    np.savetxt(str(fused_path), fused_tum,
               fmt="%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f")

    # evaluate
    eval_fused = evaluate_trajectory(str(fused_path), gt_data, with_scale=True)
    eval_original = evaluate_trajectory(best["traj_path"], gt_data, with_scale=True)

    log(f"\n  Original ATE: {eval_original.get('ate_rmse', float('inf')):.1f}m")
    log(f"  Fused ATE:    {eval_fused.get('ate_rmse', float('inf')):.1f}m")

    result = {
        "original_eval": eval_original,
        "fused_eval": eval_fused,
        "n_vio_edges": len(vio_edges),
        "n_odom_edges": len(odom_edges),
        "fused_path": str(fused_path),
    }
    save_json(result, phase_dir / "summary.json")
    return result


# PHASE F: SUMMER SESSION

def phase_f(calib, best_config):
    """Phase F: Run best config on summer session"""
    log("\n" + "=" * 70)
    log("PHASE F: SUMMER SESSION (2012-08-04)")

    session = "2012-08-04"
    phase_dir = RESULTS_DIR / "phase_f"
    phase_dir.mkdir(parents=True, exist_ok=True)
    gt_data = load_ground_truth(session)

    imu_freq_map = {"v1": 47.0, "v2": 100.0, "v3": 200.0, "v4": 100.0, "v5": 200.0}

    cfg = best_config
    cam_id = cfg.get("cam_id", 5)
    imu_ver = cfg.get("imu_ver", "v1")
    n_features = cfg.get("n_features", 3000)

    # prepare IMU data for summer session
    log("  Preparing summer IMU data...")
    summer_imu_path = IMU_DIR / f"imu_{imu_ver}_summer.csv"
    if not summer_imu_path.exists():
        image_utimes_summer = get_image_utimes(session, cam_id, n_frames=0, skip_start=200)
        if len(image_utimes_summer) == 0:
            log("  No summer images available", "WARN")
            return None
        prepare_imu_csv(session, imu_ver, image_utimes_summer, summer_imu_path)

    # run on 5000 frames first (to save time)
    for n_frames, suffix in [(5000, "5k"), (0, "full")]:
        run_label = f"F_summer_cam{cam_id}_{suffix}"
        run_dir = phase_dir / run_label
        run_dir.mkdir(parents=True, exist_ok=True)
        log(f"\n--- {run_label} ---")

        euroc_dir, ts_path, img_utimes = prepare_euroc_dir(
            session, [cam_id], n_frames, str(summer_imu_path))

        if euroc_dir is None:
            log("  Failed to prepare summer data", "ERROR")
            continue

        # write config with summer Tbc
        config_path = run_dir / "config.yaml"
        cam_calib = calib["cam_calib"][cam_id]
        imu_noise = calib["imu_noise"]
        Tbc = np.array(calib["Tbc"][cam_id])

        write_mono_inertial_config(
            config_path, cam_calib, imu_noise, Tbc,
            imu_freq=imu_freq_map[imu_ver], n_features=n_features)

        timeout = SWEEP_TIMEOUT if n_frames > 0 else FULL_TIMEOUT
        result = run_orbslam3(
            "mono_inertial", config_path, euroc_dir, ts_path,
            timeout_s=timeout, output_dir=run_dir, label=run_label)

        result["config"] = cfg

        if result["success"] and result["traj_path"]:
            eval_result = evaluate_trajectory(result["traj_path"], gt_data, with_scale=True)
            result["eval"] = eval_result
            if eval_result.get("status") == "ok":
                log(f"    ATE: {eval_result['ate_rmse']:.1f}m "
                    f"(scale={eval_result['scale']:.3f})")
        else:
            result["eval"] = {"status": "no_trajectory"}

        save_json(result, run_dir / "result.json")

        # If 5k run had 0% tracking, skip full
        if n_frames > 0 and result.get("tracking_rate", 0) < 1:
            log("  5k run failed, skipping full session")
            break

    return result


# PHASE G: RESULTS, PLOTS, AND REPORT

def phase_g():
    """Phase G: Generate summary plots and report"""
    log("\n" + "=" * 70)
    log("PHASE G: RESULTS AND REPORT")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plots_dir = RESULTS_DIR / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)

    # collect all results
    all_runs = []
    for phase in ["phase_b", "phase_c", "phase_d", "phase_e", "phase_f"]:
        summary = RESULTS_DIR / phase / "summary.json"
        if summary.exists():
            with open(summary) as f:
                data = json.load(f)
            if isinstance(data, list):
                all_runs.extend(data)
            elif isinstance(data, dict):
                if "runs" in data:
                    all_runs.extend(data["runs"])

    # ---- Plot 1: Phase B tracking rates ----
    phase_b_dir = RESULTS_DIR / "phase_b"
    if phase_b_dir.exists():
        try:
            with open(phase_b_dir / "summary.json") as f:
                pb_data = json.load(f)
            runs = pb_data.get("runs", [])
            if runs:
                labels = [r.get("label", "?")[-20:] for r in runs]
                tracking = [r.get("tracking_rate", 0) for r in runs]

                fig, ax = plt.subplots(figsize=(14, 6))
                colors = ['green' if t > 50 else 'orange' if t > 20 else 'red' for t in tracking]
                ax.barh(range(len(labels)), tracking, color=colors)
                ax.set_yticks(range(len(labels)))
                ax.set_yticklabels(labels, fontsize=8)
                ax.set_xlabel("Tracking Rate (%)")
                ax.set_title("Phase B: Mono-Inertial Sweep (3000 frames)")
                ax.axvline(x=80, color='gray', linestyle='--', alpha=0.5, label="Viable threshold")
                ax.legend()
                plt.tight_layout()
                plt.savefig(str(plots_dir / "phase_b_tracking.png"), dpi=150)
                plt.close()
                log("  Saved phase_b_tracking.png")
        except Exception as e:
            log(f"  Plot 1 error: {e}", "WARN")

    # ---- Plot 2: Trajectory vs GT ----
    gt_spring = load_ground_truth("2012-04-29")
    for phase in ["phase_b", "phase_d"]:
        phase_path = RESULTS_DIR / phase
        if not phase_path.exists():
            continue
        try:
            # find best trajectory
            best_traj = None
            best_rate = 0
            for run_dir in phase_path.iterdir():
                if not run_dir.is_dir():
                    continue
                result_file = run_dir / "result.json"
                if result_file.exists():
                    with open(result_file) as f:
                        r = json.load(f)
                    if r.get("tracking_rate", 0) > best_rate:
                        # check for trajectory file
                        for tname in ["f_nclt.txt", "CameraTrajectory.txt", "kf_nclt.txt"]:
                            tp = run_dir / tname
                            if tp.exists():
                                best_traj = tp
                                best_rate = r["tracking_rate"]
                                break

            if best_traj:
                est = load_tum_trajectory(best_traj)
                if len(est) > 10:
                    fig, ax = plt.subplots(figsize=(10, 10))
                    ax.plot(gt_spring[:, 1], gt_spring[:, 2], 'k-', alpha=0.3,
                            linewidth=0.5, label="Ground Truth")
                    ax.plot(est[:, 1], est[:, 2], 'r-', linewidth=1,
                            label=f"ORB-SLAM3 ({phase})")
                    ax.set_xlabel("X (m)")
                    ax.set_ylabel("Y (m)")
                    ax.set_title(f"Trajectory: {phase} (best run)")
                    ax.legend()
                    ax.set_aspect("equal")
                    plt.tight_layout()
                    plt.savefig(str(plots_dir / f"{phase}_trajectory.png"), dpi=150)
                    plt.close()
                    log(f"  Saved {phase}_trajectory.png")
        except Exception as e:
            log(f"  Trajectory plot error ({phase}): {e}", "WARN")

    # ---- Plot 3: ATE comparison bar chart ----
    try:
        methods = []
        ates = []

        # LiDAR ICP reference
        methods.append("LiDAR ICP\n(Exp 0.1)")
        ates.append(174.0)

        # best Phase B
        if (RESULTS_DIR / "phase_b" / "summary.json").exists():
            with open(RESULTS_DIR / "phase_b" / "summary.json") as f:
                pb = json.load(f)
            for r in pb.get("runs", []):
                if r.get("eval", {}).get("status") == "ok":
                    methods.append(f"Mono-I\n({r.get('label', '?')[-15:]})")
                    ates.append(r["eval"]["ate_rmse"])
                    break

        # best Phase D
        if (RESULTS_DIR / "phase_d" / "summary.json").exists():
            with open(RESULTS_DIR / "phase_d" / "summary.json") as f:
                pd_data = json.load(f)
            for r in pd_data if isinstance(pd_data, list) else []:
                if r.get("eval", {}).get("status") == "ok":
                    methods.append(f"Full Session\n(Phase D)")
                    ates.append(r["eval"]["ate_rmse"])
                    break

        if len(methods) > 1:
            fig, ax = plt.subplots(figsize=(10, 6))
            colors = ['#2196F3'] + ['#FF5722'] * (len(methods) - 1)
            ax.bar(range(len(methods)), ates, color=colors)
            ax.set_xticks(range(len(methods)))
            ax.set_xticklabels(methods, fontsize=8)
            ax.set_ylabel("ATE RMSE (m)")
            ax.set_title("ATE Comparison: ORB-SLAM3 Mono-Inertial vs LiDAR ICP")
            plt.tight_layout()
            plt.savefig(str(plots_dir / "ate_comparison.png"), dpi=150)
            plt.close()
            log("  Saved ate_comparison.png")
    except Exception as e:
        log(f"  ATE comparison plot error: {e}", "WARN")

    # ---- Generate REPORT.md ----
    log("  Generating REPORT.md...")
    report_lines = [
        "# Experiment 0.6: Definitive ORB-SLAM3 Evaluation on NCLT",
        f"\nGenerated: {datetime.now().strftime('%Y-%m-%d %H:%M')}",
        "\n## Summary",
        "\nThis experiment fixes ALL known issues from previous ORB-SLAM3 attempts (0.2-0.5):",
        "1. **KannalaBrandt8 fisheye** camera model (instead of PinHole f=221)",
        "2. **Mono-Inertial** and **Stereo-Inertial** modes with IMU fusion",
        "3. **Side cameras** (Cam5 forward, Cam4 left) instead of Cam0 (sky)",
        "4. **IMU at 47-200Hz** with 5 versions including KVH FOG hybrid",
        "5. **Proper Tbc extrinsics** from NCLT calibration + Ladybug3 geometry",
        "\n## Calibration",
    ]

    calib_json = RESULTS_DIR / "calibration_summary.json"
    if calib_json.exists():
        with open(calib_json) as f:
            cal = json.load(f)
        for cam_id in [5, 4, 1]:
            cc = cal.get("cam_calib", {}).get(str(cam_id), {})
            report_lines.append(f"\n### Cam{cam_id}")
            report_lines.append(f"- Source: {cc.get('source', 'unknown')}")
            report_lines.append(f"- fx={cc.get('fx', 'N/A'):.1f}, fy={cc.get('fy', 'N/A'):.1f}")
            report_lines.append(f"- cx={cc.get('cx', 'N/A'):.1f}, cy={cc.get('cy', 'N/A'):.1f}")

    report_lines.append("\n## Phase B: Mono-Inertial Sweep")
    report_lines.append("\n| Run | Camera | IMU | Tracking | ATE RMSE | Scale | IMU Init |")
    report_lines.append("|-----|--------|-----|----------|----------|-------|----------|")

    pb_summary = RESULTS_DIR / "phase_b" / "summary.json"
    if pb_summary.exists():
        with open(pb_summary) as f:
            pb = json.load(f)
        for r in pb.get("runs", []):
            cfg = r.get("config", {})
            ev = r.get("eval", {})
            ate_str = f"{ev['ate_rmse']:.1f}m" if ev.get("status") == "ok" else "N/A"
            scale_str = f"{ev['scale']:.3f}" if ev.get("status") == "ok" else "N/A"
            report_lines.append(
                f"| {r.get('label', '?')[:25]} "
                f"| Cam{cfg.get('cam_id', '?')} "
                f"| {cfg.get('imu_ver', '?')} "
                f"| {r.get('tracking_rate', 0):.1f}% "
                f"| {ate_str} "
                f"| {scale_str} "
                f"| {'Y' if r.get('imu_initialized') else 'N'} |"
            )

    report_lines.extend([
        "\n## Key Questions Answered",
        "\n### Does KannalaBrandt8 fisheye improve tracking?",
        "See Phase B: fisheye vs pinhole comparison.",
        "\n### Does IMU interpolation to 200Hz enable initialization?",
        "See Phase B: v1 (47Hz) vs v2 (100Hz) vs v3 (200Hz) comparison.",
        "\n### Does KVH hybrid IMU help?",
        "See Phase B: v4/v5 (hybrid) vs v2/v3 (MS25 only) comparison.",
        "\n### Is stereo-inertial better than mono-inertial?",
        "See Phase C results.",
        "\n### Does wheel odometry improve results?",
        "See Phase E results.",
        "\n### FINAL VERDICT",
        "\n*(To be filled after reviewing all results)*",
        "\n## Files",
        "\n| Directory | Contents |",
        "|-----------|----------|",
        "| calibration/ | COLMAP results, Tbc matrices, IMU noise |",
        "| imu_data/ | All 5 IMU versions |",
        "| phase_b/ | Mono-inertial sweep runs |",
        "| phase_c/ | Stereo-inertial runs |",
        "| phase_d/ | Full session trajectories |",
        "| phase_e/ | Wheel odometry fusion |",
        "| phase_f/ | Summer session |",
        "| plots/ | Visualization plots |",
    ])

    report_path = RESULTS_DIR / "REPORT.md"
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines))
    log(f"  Saved REPORT.md")

    log("\nPhase G complete")


# MAIN

def main():
    global LOG_FILE, SCRIPT_START
    SCRIPT_START = datetime.now()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    LOG_FILE = str(RESULTS_DIR / "experiment_log.txt")

    log("EXPERIMENT 0.6: DEFINITIVE ORB-SLAM3 EVALUATION ON NCLT")
    log(f"Started: {SCRIPT_START.strftime('%Y-%m-%d %H:%M:%S')}")

    session = SESSIONS["spring"]

    try:
        # phase A: Calibration and data prep
        calib = phase_a(session)

        # phase B: Mono-inertial sweep
        phase_b_results, best_imu = phase_b(session, calib)

        # find best config from Phase B
        best_run = max(phase_b_results, key=lambda r: r.get("tracking_rate", 0))
        best_config = best_run.get("config", {"cam_id": 5, "imu_ver": best_imu})

        # also keep second-best for Phase D comparison
        sorted_runs = sorted(phase_b_results, key=lambda r: r.get("tracking_rate", 0), reverse=True)
        top_configs = [r.get("config", best_config) for r in sorted_runs[:2]
                       if r.get("tracking_rate", 0) > 0]
        if not top_configs:
            top_configs = [best_config]

        # phase C: Stereo-inertial
        try:
            phase_c_results = phase_c(session, calib, best_imu)
        except Exception as e:
            log(f"Phase C failed: {e}", "ERROR")

        # phase D: Full session
        try:
            phase_d_results = phase_d(session, calib, top_configs)
        except Exception as e:
            log(f"Phase D failed: {e}", "ERROR")
            phase_d_results = []

        # phase E: Wheel odometry fusion
        try:
            phase_e(session, phase_d_results)
        except Exception as e:
            log(f"Phase E failed: {e}", "ERROR")

        # phase F: Summer session
        try:
            phase_f(calib, best_config)
        except Exception as e:
            log(f"Phase F failed: {e}", "ERROR")

        # phase G: Results and report
        phase_g()

    except Exception as e:
        log(f"FATAL ERROR: {e}", "ERROR")
        import traceback
        log(traceback.format_exc(), "ERROR")

    elapsed = (datetime.now() - SCRIPT_START).total_seconds()
    log(f"\nTotal runtime: {elapsed/3600:.1f} hours ({elapsed:.0f}s)")
    log("EXPERIMENT 0.6 COMPLETE")


if __name__ == "__main__":
    main()
