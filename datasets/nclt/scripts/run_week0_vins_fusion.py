#!/usr/bin/env python3
"""Experiment 0.5: VINS-Fusion mono-inertial + wheel odom + GPS on NCLT.

runs VINS-Fusion (no-ROS build) in mono-inertial mode on Ladybug3 Cam0 for
spring and summer, then progressively fuses wheel odometry and GPS constraints
via pose graph optimisation.

three configs compared:
  A = pure VIO (mono + IMU), Sim(3) aligned to GT
  B = VIO + wheel odom in a pose graph, SE(3) aligned (metric scale recovered)
  C = VIO + wheel odom + GPS in a pose graph, SE(3) aligned

sessions: 2012-04-29 (spring, 21510 images), 2012-08-04 (summer, 24138 images)

Prerequisites:
  - VINS-Fusion binary built at /tmp/vins_fusion_noros/vins_estimator/build/vins_estimator
  - Camera models library at /tmp/vins_fusion_noros/camera_models/build/libcamera_models.so
  - NCLT images prepared at /workspace/nclt_data/images_prepared/{session}/
  - NCLT sensor data at /workspace/nclt_data/sensor_data/{session}/

Usage:
    python run_week0_vins_fusion.py [--sessions 2012-04-29 2012-08-04]
                                    [--skip-vins] [--max-images 0]
                                    [--timeout 5400]
"""

import sys
import os
import json
import time
import subprocess
import shutil
import math
import traceback
import argparse
import logging
from pathlib import Path
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from scipy import sparse

# project paths
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / 'src'))

from data_loaders.sensor_loader import SensorLoader
from data_loaders.ground_truth_loader import GroundTruthLoader
from evaluation.metrics import compute_ate, compute_rpe, sync_trajectories

VINS_BINARY = Path("/tmp/vins_fusion_noros/vins_estimator/build/vins_estimator")
CAMERA_MODELS_LIB = Path("/tmp/vins_fusion_noros/camera_models/build")
SESSIONS = ["2012-04-29", "2012-08-04"]
RESULTS_DIR = PROJECT_ROOT / "results" / "week0_vins_fusion"
DATA_ROOT = Path("/workspace/nclt_data")
SENSOR_ROOT = DATA_ROOT / "sensor_data"
IMAGE_ROOT = DATA_ROOT / "images_prepared"
GT_ROOT = DATA_ROOT / "ground_truth"
GT_TUM_DIR = PROJECT_ROOT / "results" / "week0_visual_slam" / "ground_truth"

DEFAULT_TIMEOUT = 5400  # 90 minutes per session
START_TIME = time.time()

# WGS84 constants for lat/lon -> ENU conversionWGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014

# logging
_file_logger = None


def setup_logging(log_path):
    """set up dual logging to console and file"""
    global _file_logger
    log_path = Path(log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    _file_logger = open(log_path, 'w')


def log(msg):
    """log a timestamped message to console and file"""
    elapsed = time.time() - START_TIME
    line = f"[{elapsed:8.1f}s] {msg}"
    print(line, flush=True)
    if _file_logger:
        _file_logger.write(line + "\n")
        _file_logger.flush()


# Umeyama Sim(3) alignment
def umeyama_alignment(source, target):
    """compute Sim(3) alignment (scale, rotation, translation) via Umeyama"""
    n, dim = source.shape
    assert target.shape == source.shape

    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    src_c = source - mu_s
    tgt_c = target - mu_t

    var_s = np.sum(src_c ** 2) / n
    if var_s < 1e-12:
        # degenerate case: all source points are the same
        return 1.0, np.eye(3), mu_t - mu_s, np.tile(mu_t, (n, 1))

    H = (tgt_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(H)

    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[dim - 1, dim - 1] = -1

    R = U @ S @ Vt
    scale = np.trace(np.diag(D) @ S) / var_s
    t = mu_t - scale * R @ mu_s
    aligned = scale * (source @ R.T) + t

    return scale, R, t, aligned


# sE(3) alignment (no scale)
def se3_alignment(source, target):
    """compute rigid SE(3) alignment (rotation + translation, no scale)"""
    n = source.shape[0]
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    src_c = source - mu_s
    tgt_c = target - mu_t

    H = tgt_c.T @ src_c
    U, _, Vt = np.linalg.svd(H)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    t = mu_t - R @ mu_s
    aligned = (source @ R.T) + t

    return R, t, aligned


# GPS coordinate conversion
def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """convert geodetic (lat, lon, alt) to local ENU coordinates"""
    sin_lat0 = np.sin(lat0)
    cos_lat0 = np.cos(lat0)
    sin_lon0 = np.sin(lon0)
    cos_lon0 = np.cos(lon0)

    # ECEF for reference
    N0 = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat0 ** 2)
    x0 = (N0 + alt0) * cos_lat0 * cos_lon0
    y0 = (N0 + alt0) * cos_lat0 * sin_lon0
    z0 = (N0 * (1 - WGS84_E2) + alt0) * sin_lat0

    # ECEF for points
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)
    N = WGS84_A / np.sqrt(1 - WGS84_E2 * sin_lat ** 2)
    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1 - WGS84_E2) + alt) * sin_lat

    # difference in ECEF
    dx = x - x0
    dy = y - y0
    dz = z - z0

    # rotation to ENU
    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz

    return east, north, up


# data preparation
def prepare_vins_data(session, work_dir, max_images=0):
    """prepare images, IMU data, and config files for VINS-Fusion binary"""
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    # ---- Images ----
    src_image_dir = IMAGE_ROOT / session
    if not src_image_dir.exists():
        raise FileNotFoundError(f"Image directory not found: {src_image_dir}")

    # list and sort image files
    image_files = sorted(src_image_dir.glob("*.png"))
    if max_images > 0:
        image_files = image_files[:max_images]
    n_images = len(image_files)
    log(f"  Images: {n_images} files from {src_image_dir}")

    # VINS binary expects: image_folder/{timestamp_ns}.png
    # NCLT images are named {utime_us}.png (microseconds)
    # We need to create symlinks named {utime_ns}.png (nanoseconds)
    vins_image_dir = work_dir / "images"
    if vins_image_dir.exists():
        shutil.rmtree(vins_image_dir)
    vins_image_dir.mkdir()

    timestamps_ns = []
    for img_path in image_files:
        utime_us = int(img_path.stem)
        utime_ns = utime_us * 1000  # microseconds -> nanoseconds
        timestamps_ns.append(utime_ns)
        link_path = vins_image_dir / f"{utime_ns}.png"
        link_path.symlink_to(img_path)

    # ---- Timestamps file ----
    timestamps_path = work_dir / "timestamps.txt"
    with open(timestamps_path, 'w') as f:
        for ts in timestamps_ns:
            f.write(f"{ts}\n")
    log(f"  Timestamps: {len(timestamps_ns)} entries written")

    # ---- IMU data ----
    sensor_loader = SensorLoader()
    imu_df = sensor_loader.load_ms25_imu(session)
    n_imu = len(imu_df)
    log(f"  IMU: {n_imu} samples loaded")

    # VINS-Fusion expects: timestamp_ns, gx, gy, gz, ax, ay, az
    imu_csv_path = work_dir / "imu_data.csv"
    with open(imu_csv_path, 'w') as f:
        f.write("#timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n")
        for _, row in imu_df.iterrows():
            ts_ns = int(row['utime']) * 1000  # us -> ns
            f.write(f"{ts_ns},{row['rot_x']},{row['rot_y']},{row['rot_z']},"
                    f"{row['accel_x']},{row['accel_y']},{row['accel_z']}\n")
    log(f"  IMU CSV: {n_imu} rows written to {imu_csv_path.name}")

    # ---- Camera calibration YAML ----
    cam_calib_path = work_dir / "nclt_cam0_fisheye.yaml"
    cam_calib_content = """%YAML:1.0
model_type: KANNALA_BRANDT
camera_name: nclt_cam0
image_width: 808
image_height: 616
projection_parameters:
   k2: 0.01
   k3: -0.005
   k4: 0.001
   k5: 0.0
   mu: 579.0
   mv: 579.0
   u0: 404.0
   v0: 308.0
"""
    with open(cam_calib_path, 'w') as f:
        f.write(cam_calib_content)

    # ---- VINS-Fusion config YAML ----
    output_path = work_dir / "output"
    output_path.mkdir(exist_ok=True)

    config_yaml_path = work_dir / "vins_config.yaml"
    config_content = f"""%YAML:1.0

imu: 1
num_of_cam: 1

cam0_calib: "{cam_calib_path}"
image_width: 808
image_height: 616

estimate_extrinsic: 2
estimate_td: 1
td: 0.0

body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [0,-1,0,0, 0,0,-1,0, 1,0,0,0, 0,0,0,1]

acc_n: 0.2
gyr_n: 0.02
acc_w: 0.002
gyr_w: 0.0002
g_norm: 9.81007

max_cnt: 200
min_dist: 25
fisheye: 1
flow_back: 1
F_threshold: 1.0

max_solver_time: 0.08
max_num_iterations: 10
keyframe_parallax: 10.0

use_gpu: 0
use_gpu_acc_flow: 0
show_track: 0
show_TMI: 0
multiple_thread: 1

output_path: "{output_path}/"
"""
    with open(config_yaml_path, 'w') as f:
        f.write(config_content)
    log(f"  Config written: {config_yaml_path.name}")

    return (config_yaml_path, vins_image_dir, timestamps_path,
            imu_csv_path, n_images, n_imu)


# run VINS-Fusion binary
def run_vins_fusion(session, work_dir, timeout=DEFAULT_TIMEOUT):
    """run the VINS-Fusion binary on prepared data and return the VIO trajectory"""
    work_dir = Path(work_dir)
    config_path = work_dir / "vins_config.yaml"
    image_dir = work_dir / "images"
    timestamps_path = work_dir / "timestamps.txt"
    imu_csv_path = work_dir / "imu_data.csv"
    output_csv = work_dir / "output" / "vio.csv"

    if not VINS_BINARY.exists():
        log(f"  ERROR: VINS binary not found: {VINS_BINARY}")
        return None

    cmd = [
        str(VINS_BINARY),
        str(config_path),
        str(image_dir) + "/",
        str(timestamps_path),
        str(imu_csv_path),
    ]

    env = os.environ.copy()
    existing_ld = env.get('LD_LIBRARY_PATH', '')
    env['LD_LIBRARY_PATH'] = f"{CAMERA_MODELS_LIB}:{existing_ld}"

    log(f"  Running VINS-Fusion binary...")
    log(f"  Command: {' '.join(cmd[-4:])}")
    log(f"  Timeout: {timeout}s ({timeout/60:.0f} min)")

    t_start = time.time()
    try:
        result = subprocess.run(
            cmd, env=env, capture_output=True, text=True,
            timeout=timeout, cwd=str(work_dir)
        )
        t_elapsed = time.time() - t_start
        log(f"  VINS-Fusion completed in {t_elapsed:.1f}s "
            f"(exit code {result.returncode})")

        # save stdout/stderr logs
        log_file = work_dir / "vins_stdout.log"
        with open(log_file, 'w') as f:
            f.write(result.stdout)
        stderr_file = work_dir / "vins_stderr.log"
        with open(stderr_file, 'w') as f:
            f.write(result.stderr)

        # count warnings/errors in output
        n_warn = result.stdout.count("WARNING") + result.stderr.count("WARNING")
        n_err = result.stdout.count("ERROR") + result.stderr.count("ERROR")
        log(f"  Log: {n_warn} warnings, {n_err} errors")

        # check for tracking failures in output
        n_fail = result.stdout.count("fail") + result.stdout.count("FAIL")
        if n_fail > 0:
            log(f"  Note: found {n_fail} 'fail' mentions in output")

    except subprocess.TimeoutExpired:
        t_elapsed = time.time() - t_start
        log(f"  VINS-Fusion TIMED OUT after {t_elapsed:.1f}s")
        return None
    except Exception as e:
        log(f"  VINS-Fusion ERROR: {e}")
        return None

    # read output trajectory
    if not output_csv.exists():
        log(f"  ERROR: Output file not found: {output_csv}")
        # check if there are any files in the output directory
        output_files = list((work_dir / "output").glob("*"))
        log(f"  Output dir contents: {[f.name for f in output_files]}")
        return None

    try:
        # format: timestamp_sec p_x p_y p_z q_w q_x q_y q_z v_x v_y v_z
        raw = np.loadtxt(str(output_csv))
        if raw.ndim == 1:
            raw = raw.reshape(1, -1)
        n_poses = len(raw)
        log(f"  Loaded {n_poses} VIO poses from {output_csv.name}")

        if n_poses < 10:
            log(f"  WARNING: Very few poses ({n_poses}), VIO likely failed")
            return None

        # convert to TUM format: timestamp_s x y z qx qy qz qw
        # input: ts px py pz qw qx qy qz vx vy vz
        traj = np.zeros((n_poses, 8))
        traj[:, 0] = raw[:, 0]          # timestamp_s
        traj[:, 1:4] = raw[:, 1:4]      # position
        traj[:, 4] = raw[:, 5]           # qx
        traj[:, 5] = raw[:, 6]           # qy
        traj[:, 6] = raw[:, 7]           # qz
        traj[:, 7] = raw[:, 4]           # qw

        # log trajectory statistics
        pos = traj[:, 1:4]
        traj_len = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))
        duration = traj[-1, 0] - traj[0, 0]
        log(f"  VIO trajectory: length={traj_len:.1f}m, "
            f"duration={duration:.1f}s, "
            f"t=[{traj[0, 0]:.3f}, {traj[-1, 0]:.3f}]")

        return traj

    except Exception as e:
        log(f"  ERROR reading VIO output: {e}")
        traceback.print_exc()
        return None


# load ground truth in TUM format
def load_ground_truth_tum(session):
    """load ground truth trajectory in TUM format (Nx8)"""
    tum_path = GT_TUM_DIR / f"gt_{session}.tum"
    if tum_path.exists():
        gt = np.loadtxt(str(tum_path))
        log(f"  Ground truth: {len(gt)} poses from {tum_path.name}")
        return gt

    # fallback: load from CSV
    log(f"  TUM file not found, loading from CSV...")
    gt_loader = GroundTruthLoader()
    gt_df = gt_loader.load_ground_truth(session)
    gt = np.column_stack([
        gt_df['utime'].values / 1e6,  # us -> seconds
        gt_df['x'].values, gt_df['y'].values, gt_df['z'].values,
        gt_df['qx'].values, gt_df['qy'].values,
        gt_df['qz'].values, gt_df['qw'].values
    ])
    log(f"  Ground truth: {len(gt)} poses from CSV")
    return gt


# wheel odometry fusion (Config B)
def interpolate_odometry_at_times(odom_df, query_times_us):
    """interpolate wheel odometry poses at specified timestamps"""
    odom_utimes = odom_df['utime'].values.astype(np.float64)
    results = []

    for qt in query_times_us:
        idx = np.searchsorted(odom_utimes, qt)
        if idx <= 0 or idx >= len(odom_utimes):
            continue
        # linear interpolation
        t0, t1 = odom_utimes[idx - 1], odom_utimes[idx]
        alpha = (qt - t0) / (t1 - t0) if (t1 - t0) > 0 else 0.0
        row0 = odom_df.iloc[idx - 1]
        row1 = odom_df.iloc[idx]
        x = row0['x'] + alpha * (row1['x'] - row0['x'])
        y = row0['y'] + alpha * (row1['y'] - row0['y'])
        z = row0['z'] + alpha * (row1['z'] - row0['z'])
        roll = row0['roll'] + alpha * (row1['roll'] - row0['roll'])
        pitch = row0['pitch'] + alpha * (row1['pitch'] - row0['pitch'])
        # handle yaw wrapping
        dyaw = (row1['yaw'] - row0['yaw'] + np.pi) % (2 * np.pi) - np.pi
        yaw = row0['yaw'] + alpha * dyaw
        results.append([qt, x, y, z, roll, pitch, yaw])

    return np.array(results) if results else None


def fuse_wheel_odometry(vio_traj, session):
    """fuse VIO trajectory with wheel odometry via 2D pose graph optimization"""
    log(f"  Loading wheel odometry for fusion...")
    try:
        sensor_loader = SensorLoader()
        odom_df = sensor_loader.load_odometry_mu(session, hz=100)
        log(f"  Wheel odometry: {len(odom_df)} samples (100Hz)")
    except FileNotFoundError:
        try:
            odom_df = sensor_loader.load_odometry_mu(session, hz=10)
            log(f"  Wheel odometry: {len(odom_df)} samples (10Hz)")
        except FileNotFoundError:
            log(f"  WARNING: No wheel odometry available")
            return None

    n = len(vio_traj)
    if n < 10:
        log(f"  WARNING: Too few VIO poses ({n}) for fusion")
        return None

    # get VIO timestamps in microseconds
    vio_ts_us = vio_traj[:, 0] * 1e6

    # interpolate wheel odometry at VIO timestamps
    odom_interp = interpolate_odometry_at_times(odom_df, vio_ts_us)
    if odom_interp is None or len(odom_interp) < 10:
        log(f"  WARNING: Could not interpolate enough odometry poses")
        return None
    log(f"  Interpolated {len(odom_interp)} odom poses at VIO timestamps")

    # match VIO and odom by timestamp
    odom_ts_set = set(odom_interp[:, 0].astype(np.int64))
    matched_vio_idx = []
    matched_odom_idx = []
    odom_ts_array = odom_interp[:, 0].astype(np.int64)

    for i in range(n):
        ts_us = int(vio_ts_us[i])
        # find closest odom timestamp
        diffs = np.abs(odom_ts_array - ts_us)
        best_j = np.argmin(diffs)
        if diffs[best_j] < 50000:  # within 50ms
            matched_vio_idx.append(i)
            matched_odom_idx.append(best_j)

    matched_vio_idx = np.array(matched_vio_idx)
    matched_odom_idx = np.array(matched_odom_idx)
    n_matched = len(matched_vio_idx)
    log(f"  Matched {n_matched}/{n} VIO poses with odometry")

    if n_matched < 10:
        log(f"  WARNING: Too few matches for fusion")
        return None

    # extract 2D poses from VIO
    vio_2d = np.zeros((n_matched, 3))  # x, y, yaw
    for k, vi in enumerate(matched_vio_idx):
        vio_2d[k, 0] = vio_traj[vi, 1]  # x
        vio_2d[k, 1] = vio_traj[vi, 2]  # y
        q = vio_traj[vi, 4:8]  # qx, qy, qz, qw
        R = Rotation.from_quat(q).as_matrix()
        vio_2d[k, 2] = np.arctan2(R[1, 0], R[0, 0])  # yaw

    # extract 2D poses from wheel odometry
    odom_2d = np.zeros((n_matched, 3))
    for k, oi in enumerate(matched_odom_idx):
        odom_2d[k, 0] = odom_interp[oi, 1]  # x
        odom_2d[k, 1] = odom_interp[oi, 2]  # y
        odom_2d[k, 2] = odom_interp[oi, 6]  # yaw

    # build pose graph edges
    # VIO relative poses
    vio_edges = []
    for k in range(n_matched - 1):
        xi, yi, ti = vio_2d[k]
        xj, yj, tj = vio_2d[k + 1]
        ct, st = np.cos(ti), np.sin(ti)
        dx = xj - xi
        dy = yj - yi
        dxl = ct * dx + st * dy
        dyl = -st * dx + ct * dy
        dtl = (tj - ti + np.pi) % (2 * np.pi) - np.pi
        vio_edges.append((k, k + 1, dxl, dyl, dtl))

    # wheel odometry relative poses
    odom_edges = []
    for k in range(n_matched - 1):
        xi, yi, ti = odom_2d[k]
        xj, yj, tj = odom_2d[k + 1]
        ct, st = np.cos(ti), np.sin(ti)
        dx = xj - xi
        dy = yj - yi
        dxl = ct * dx + st * dy
        dyl = -st * dx + ct * dy
        dtl = (tj - ti + np.pi) % (2 * np.pi) - np.pi
        odom_edges.append((k, k + 1, dxl, dyl, dtl))

    log(f"  Pose graph: {n_matched} nodes, {len(vio_edges)} VIO edges, "
        f"{len(odom_edges)} odom edges")

    # optimize using simple Gauss-Newton on 2D poses
    VIO_WEIGHT = 1.0
    ODOM_WEIGHT = 5.0  # Trust odometry more for metric scale

    opt_poses = _optimize_2d_pose_graph(
        vio_2d.copy(), vio_edges, odom_edges,
        VIO_WEIGHT, ODOM_WEIGHT, max_iter=50
    )

    # rebuild full trajectory: use optimized 2D + original z and pitch/roll
    fused_traj = np.zeros((n_matched, 8))
    for k, vi in enumerate(matched_vio_idx):
        fused_traj[k, 0] = vio_traj[vi, 0]  # timestamp
        fused_traj[k, 1] = opt_poses[k, 0]   # optimized x
        fused_traj[k, 2] = opt_poses[k, 1]   # optimized y
        fused_traj[k, 3] = vio_traj[vi, 3]   # original z
        # reconstruct quaternion from optimized yaw + original pitch/roll
        yaw = opt_poses[k, 2]
        orig_q = vio_traj[vi, 4:8]
        orig_euler = Rotation.from_quat(orig_q).as_euler('ZYX')
        new_euler = np.array([yaw, orig_euler[1], orig_euler[2]])
        new_q = Rotation.from_euler('ZYX', new_euler).as_quat()
        fused_traj[k, 4:8] = new_q  # qx, qy, qz, qw

    pos = fused_traj[:, 1:4]
    traj_len = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))
    log(f"  Fused trajectory: {n_matched} poses, length={traj_len:.1f}m")

    return fused_traj


# GPS fusion (Config C)
def fuse_gps(fused_traj, session):
    """fuse trajectory with GPS constraints via 2D pose graph optimization"""
    log(f"  Loading GPS RTK for fusion...")
    try:
        sensor_loader = SensorLoader()
        gps_df = sensor_loader.load_gps_rtk(session)
        log(f"  GPS RTK: {len(gps_df)} raw samples")
    except FileNotFoundError:
        log(f"  WARNING: No GPS RTK data available")
        return None

    if len(gps_df) < 10:
        log(f"  WARNING: Too few GPS samples")
        return None

    # GPS RTK format: utime, mode, n_sats, lat(rad), lon(rad), alt(m), heading, speed
    gps_utimes = gps_df.iloc[:, 0].values.astype(np.float64)
    gps_lat = gps_df.iloc[:, 3].values.astype(np.float64)
    gps_lon = gps_df.iloc[:, 4].values.astype(np.float64)
    gps_alt = gps_df.iloc[:, 5].values.astype(np.float64)

    # filter out invalid GPS readings (NaN, mode 0)
    gps_mode = gps_df.iloc[:, 1].values
    valid_mask = (~np.isnan(gps_lat)) & (~np.isnan(gps_lon)) & (~np.isnan(gps_alt))
    valid_mask &= (gps_mode >= 1)
    valid_mask &= (np.abs(gps_lat) > 0.01) & (np.abs(gps_lon) > 0.01)

    gps_utimes = gps_utimes[valid_mask]
    gps_lat = gps_lat[valid_mask]
    gps_lon = gps_lon[valid_mask]
    gps_alt = gps_alt[valid_mask]
    log(f"  Valid GPS: {len(gps_utimes)} samples after filtering")

    if len(gps_utimes) < 10:
        log(f"  WARNING: Too few valid GPS samples")
        return None

    # convert to local ENU using first GPS reading as reference
    lat0, lon0, alt0 = gps_lat[0], gps_lon[0], gps_alt[0]
    gps_east, gps_north, gps_up = geodetic_to_enu(
        gps_lat, gps_lon, gps_alt, lat0, lon0, alt0)

    n = len(fused_traj)
    traj_ts_us = fused_traj[:, 0] * 1e6

    # match GPS to trajectory timestamps (nearest neighbor, 1s tolerance)
    gps_constraints = []  # (traj_idx, gps_east, gps_north)
    for g in range(len(gps_utimes)):
        gt_us = gps_utimes[g]
        diffs = np.abs(traj_ts_us - gt_us)
        best_i = np.argmin(diffs)
        if diffs[best_i] < 1e6:  # 1 second tolerance
            gps_constraints.append((best_i, gps_east[g], gps_north[g]))

    log(f"  GPS constraints: {len(gps_constraints)} matched to trajectory")

    if len(gps_constraints) < 5:
        log(f"  WARNING: Too few GPS constraints")
        return None

    # extract 2D poses
    poses_2d = np.zeros((n, 3))
    for i in range(n):
        poses_2d[i, 0] = fused_traj[i, 1]
        poses_2d[i, 1] = fused_traj[i, 2]
        q = fused_traj[i, 4:8]
        R = Rotation.from_quat(q).as_matrix()
        poses_2d[i, 2] = np.arctan2(R[1, 0], R[0, 0])

    # build odometry edges from current trajectory
    odom_edges = []
    for i in range(n - 1):
        xi, yi, ti = poses_2d[i]
        xj, yj, tj = poses_2d[i + 1]
        ct, st = np.cos(ti), np.sin(ti)
        dx = xj - xi
        dy = yj - yi
        dxl = ct * dx + st * dy
        dyl = -st * dx + ct * dy
        dtl = (tj - ti + np.pi) % (2 * np.pi) - np.pi
        odom_edges.append((i, i + 1, dxl, dyl, dtl))

    # We need to align GPS ENU frame to trajectory frame first
    # use the GPS-matched trajectory positions for initial alignment
    gps_pos = np.array([[gc[1], gc[2]] for gc in gps_constraints])
    traj_pos = np.array([[poses_2d[gc[0], 0], poses_2d[gc[0], 1]]
                         for gc in gps_constraints])

    # estimate 2D rigid transform: traj = R * gps + t
    # (or vice versa; we want GPS in traj frame)
    gps_mu = gps_pos.mean(axis=0)
    traj_mu = traj_pos.mean(axis=0)
    gps_c = gps_pos - gps_mu
    traj_c = traj_pos - traj_mu

    H = traj_c.T @ gps_c
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[1, 1] = -1
    R_gps = U @ S @ Vt
    t_gps = traj_mu - R_gps @ gps_mu

    # transform GPS constraints to trajectory frame
    gps_aligned = []
    for traj_i, ge, gn in gps_constraints:
        gps_in_traj = R_gps @ np.array([ge, gn]) + t_gps
        gps_aligned.append((traj_i, gps_in_traj[0], gps_in_traj[1]))

    # optimize with GPS constraints
    ODOM_WEIGHT = 1.0
    GPS_WEIGHT = 0.5  # GPS is noisy, don't over-constrain

    opt_poses = _optimize_2d_pose_graph_with_gps(
        poses_2d.copy(), odom_edges, gps_aligned,
        ODOM_WEIGHT, GPS_WEIGHT, max_iter=50
    )

    # rebuild trajectory
    gps_fused_traj = fused_traj.copy()
    for i in range(n):
        gps_fused_traj[i, 1] = opt_poses[i, 0]
        gps_fused_traj[i, 2] = opt_poses[i, 1]
        # update yaw in quaternion
        yaw = opt_poses[i, 2]
        orig_q = fused_traj[i, 4:8]
        orig_euler = Rotation.from_quat(orig_q).as_euler('ZYX')
        new_euler = np.array([yaw, orig_euler[1], orig_euler[2]])
        new_q = Rotation.from_euler('ZYX', new_euler).as_quat()
        gps_fused_traj[i, 4:8] = new_q

    pos = gps_fused_traj[:, 1:4]
    traj_len = np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1))
    log(f"  GPS-fused trajectory: {n} poses, length={traj_len:.1f}m")

    return gps_fused_traj


# 2D Pose Graph Optimizers
def _angle_diff(a, b):
    """normalize angle difference to [-pi, pi]"""
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def _optimize_2d_pose_graph(init_poses, edges_a, edges_b,
                            weight_a, weight_b, max_iter=50,
                            damping=1e-3):
    """optimize a 2D pose graph with two sets of relative pose edges"""
    n = len(init_poses)
    p = init_poses.copy()
    all_edges = [(e, weight_a) for e in edges_a] + [(e, weight_b) for e in edges_b]
    ne = len(all_edges)

    lam = damping
    prev_cost = float('inf')

    for it in range(max_iter):
        rows, cols, vals, res = [], [], [], []
        ri = 0

        for (i, j, dxm, dym, dtm), w in all_edges:
            xi, yi, ti = p[i]
            xj, yj, tj = p[j]
            ct, st = np.cos(ti), np.sin(ti)
            dx, dy = xj - xi, yj - yi
            dxp = ct * dx + st * dy
            dyp = -st * dx + ct * dy
            dtp = _angle_diff(tj, ti)

            res.extend([w * (dxp - dxm), w * (dyp - dym),
                        w * _angle_diff(dtp, dtm)])

            ii, jj = 3 * i, 3 * j
            # row ri: d(dxp)/d(poses)
            rows.extend([ri] * 5)
            cols.extend([ii, ii + 1, ii + 2, jj, jj + 1])
            vals.extend([w * (-ct), w * (-st), w * (-st * dx + ct * dy),
                         w * ct, w * st])
            # row ri+1: d(dyp)/d(poses)
            rows.extend([ri + 1] * 5)
            cols.extend([ii, ii + 1, ii + 2, jj, jj + 1])
            vals.extend([w * st, w * (-ct), w * (-ct * dx - st * dy),
                         w * (-st), w * ct])
            # row ri+2: d(dtp)/d(poses)
            rows.extend([ri + 2] * 2)
            cols.extend([ii + 2, jj + 2])
            vals.extend([w * (-1.0), w * 1.0])
            ri += 3

        J = sparse.csr_matrix((vals, (rows, cols)), shape=(3 * ne, 3 * n))
        e = np.array(res)
        cost = np.sum(e ** 2)
        H = J.T @ J
        b = -J.T @ e

        # lM damping
        diag_H = H.diagonal().copy()
        diag_H[diag_H < 1e-6] = 1e-6
        H_lm = H + sparse.diags(lam * diag_H)

        # fix first pose
        for k in range(3):
            H_lm[k, :] = 0
            H_lm[:, k] = 0
            H_lm[k, k] = 1e6
            b[k] = 0

        try:
            dx_vec = sparse.linalg.spsolve(H_lm.tocsc(), b)
        except Exception:
            lam *= 10
            continue
        if np.any(np.isnan(dx_vec)):
            lam *= 10
            continue

        p[:, 0] += dx_vec[0::3]
        p[:, 1] += dx_vec[1::3]
        p[:, 2] += dx_vec[2::3]
        p[:, 2] = (p[:, 2] + np.pi) % (2 * np.pi) - np.pi

        unorm = np.linalg.norm(dx_vec)
        lam = max(lam / 2, 1e-7) if cost < prev_cost else min(lam * 5, 1e3)
        prev_cost = cost

        if it % 10 == 0:
            log(f"    PG iter {it}: cost={cost:.1f}, |dx|={unorm:.6f}, lam={lam:.1e}")
        if unorm < 1e-4:
            log(f"    PG converged at iter {it}")
            break

    log(f"    PG final cost: {cost:.1f}")
    return p


def _optimize_2d_pose_graph_with_gps(init_poses, odom_edges, gps_constraints,
                                     odom_weight, gps_weight, max_iter=50,
                                     damping=1e-3):
    """optimize a 2D pose graph with odometry edges and GPS absolute constraints"""
    n = len(init_poses)
    p = init_poses.copy()
    n_odom = len(odom_edges)
    n_gps = len(gps_constraints)
    ne_rows = 3 * n_odom + 2 * n_gps  # 3 residuals per odom, 2 per GPS

    lam = damping
    prev_cost = float('inf')

    for it in range(max_iter):
        rows, cols, vals, res = [], [], [], []
        ri = 0

        # odometry edges
        for (i, j, dxm, dym, dtm) in odom_edges:
            w = odom_weight
            xi, yi, ti = p[i]
            xj, yj, tj = p[j]
            ct, st = np.cos(ti), np.sin(ti)
            dx, dy = xj - xi, yj - yi
            dxp = ct * dx + st * dy
            dyp = -st * dx + ct * dy
            dtp = _angle_diff(tj, ti)

            res.extend([w * (dxp - dxm), w * (dyp - dym),
                        w * _angle_diff(dtp, dtm)])

            ii, jj = 3 * i, 3 * j
            rows.extend([ri] * 5)
            cols.extend([ii, ii + 1, ii + 2, jj, jj + 1])
            vals.extend([w * (-ct), w * (-st), w * (-st * dx + ct * dy),
                         w * ct, w * st])
            rows.extend([ri + 1] * 5)
            cols.extend([ii, ii + 1, ii + 2, jj, jj + 1])
            vals.extend([w * st, w * (-ct), w * (-ct * dx - st * dy),
                         w * (-st), w * ct])
            rows.extend([ri + 2] * 2)
            cols.extend([ii + 2, jj + 2])
            vals.extend([w * (-1.0), w * 1.0])
            ri += 3

        # GPS absolute constraints
        for (node_i, gps_x, gps_y) in gps_constraints:
            w = gps_weight
            xi, yi = p[node_i, 0], p[node_i, 1]
            res.extend([w * (xi - gps_x), w * (yi - gps_y)])
            ii = 3 * node_i
            rows.extend([ri, ri + 1])
            cols.extend([ii, ii + 1])
            vals.extend([w, w])
            ri += 2

        J = sparse.csr_matrix((vals, (rows, cols)), shape=(ri, 3 * n))
        e = np.array(res)
        cost = np.sum(e ** 2)
        H = J.T @ J
        b = -J.T @ e

        diag_H = H.diagonal().copy()
        diag_H[diag_H < 1e-6] = 1e-6
        H_lm = H + sparse.diags(lam * diag_H)

        # with GPS constraints, we don't need to fix the first pose rigidly,
        # but we add a mild anchor for numerical stability
        for k in range(3):
            H_lm[k, k] += 1e3

        try:
            dx_vec = sparse.linalg.spsolve(H_lm.tocsc(), b)
        except Exception:
            lam *= 10
            continue
        if np.any(np.isnan(dx_vec)):
            lam *= 10
            continue

        p[:, 0] += dx_vec[0::3]
        p[:, 1] += dx_vec[1::3]
        p[:, 2] += dx_vec[2::3]
        p[:, 2] = (p[:, 2] + np.pi) % (2 * np.pi) - np.pi

        unorm = np.linalg.norm(dx_vec)
        lam = max(lam / 2, 1e-7) if cost < prev_cost else min(lam * 5, 1e3)
        prev_cost = cost

        if it % 10 == 0:
            log(f"    GPS-PG iter {it}: cost={cost:.1f}, |dx|={unorm:.6f}, lam={lam:.1e}")
        if unorm < 1e-4:
            log(f"    GPS-PG converged at iter {it}")
            break

    log(f"    GPS-PG final cost: {cost:.1f}")
    return p


# evaluation
def evaluate_trajectory(traj, gt_all, method_name, session, output_dir,
                        use_sim3=True):
    """evaluate a trajectory against ground truth and save results"""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    log(f"  Evaluating {method_name} on {session}...")

    # synchronize trajectories
    synced_est, synced_gt = sync_trajectories(traj, gt_all, tolerance=0.5)
    n_synced = len(synced_est)
    log(f"  Synchronized: {n_synced} / {len(traj)} poses "
        f"(of {len(gt_all)} GT)")

    if n_synced < 10:
        log(f"  WARNING: Too few synchronized poses ({n_synced})")
        return None

    est_pos = synced_est[:, 1:4]
    gt_pos = synced_gt[:, 1:4]

    # alignment
    if use_sim3:
        scale, R, t, aligned_pos = umeyama_alignment(est_pos, gt_pos)
        log(f"  Sim(3) alignment: scale={scale:.4f}")
    else:
        R, t, aligned_pos = se3_alignment(est_pos, gt_pos)
        scale = 1.0
        log(f"  SE(3) alignment (no scale)")

    # build aligned trajectory array for ATE/RPE computation
    aligned_traj = synced_est.copy()
    aligned_traj[:, 1:4] = aligned_pos

    # compute ATE
    ate = compute_ate(aligned_traj, synced_gt)
    log(f"  ATE RMSE: {ate['rmse']:.2f}m, Mean: {ate['mean']:.2f}m, "
        f"Median: {ate['median']:.2f}m, Max: {ate['max']:.2f}m")

    # compute RPE
    rpe = compute_rpe(aligned_traj, synced_gt, delta=1)
    log(f"  RPE Trans RMSE: {rpe['trans_rmse']:.4f}m, "
        f"Rot RMSE: {rpe['rot_rmse']:.2f}deg")

    # trajectory lengths
    traj_len = np.sum(np.linalg.norm(np.diff(aligned_pos, axis=0), axis=1))
    gt_len = np.sum(np.linalg.norm(np.diff(gt_pos, axis=0), axis=1))
    duration = synced_est[-1, 0] - synced_est[0, 0]

    results = {
        'method': method_name,
        'session': session,
        'n_poses': n_synced,
        'n_total_poses': len(traj),
        'alignment': 'Sim(3)' if use_sim3 else 'SE(3)',
        'scale': float(scale),
        'ate_rmse': float(ate['rmse']),
        'ate_mean': float(ate['mean']),
        'ate_median': float(ate['median']),
        'ate_max': float(ate['max']),
        'ate_std': float(ate['std']),
        'rpe_trans_rmse': float(rpe['trans_rmse']),
        'rpe_trans_mean': float(rpe['trans_mean']),
        'rpe_rot_rmse': float(rpe['rot_rmse']),
        'rpe_rot_mean': float(rpe['rot_mean']),
        'traj_length': float(traj_len),
        'gt_length': float(gt_len),
        'length_error_pct': float(abs(traj_len - gt_len) / gt_len * 100)
                            if gt_len > 0 else 0.0,
        'duration_s': float(duration),
    }

    np.savetxt(output_dir / "trajectory.txt", aligned_traj, fmt='%.6f',
               header='timestamp x y z qx qy qz qw')
    np.savetxt(output_dir / "trajectory_raw.txt", traj, fmt='%.6f',
               header='timestamp x y z qx qy qz qw')

    # save evaluation results
    with open(output_dir / "eval_results.json", 'w') as f:
        json.dump(results, f, indent=2)

    # save stats
    stats = {
        'method': method_name,
        'session': session,
        'n_poses': n_synced,
        'runtime_note': 'see parent stats.json',
    }
    with open(output_dir / "stats.json", 'w') as f:
        json.dump(stats, f, indent=2)

    # store error arrays for plotting (not saved to JSON)
    results['_ate_errors'] = ate['errors']
    results['_aligned_pos'] = aligned_pos
    results['_gt_pos'] = gt_pos
    results['_timestamps'] = synced_est[:, 0]

    return results


# plotting
def plot_trajectory_comparison(all_results, session, output_dir):
    """plot trajectory comparison for all configs on one session"""
    output_dir = Path(output_dir)

    fig, axes = plt.subplots(1, 2, figsize=(20, 9))

    colors = {
        'Config A (VIO)': '#e74c3c',
        'Config B (+Wheel)': '#3498db',
        'Config C (+GPS)': '#2ecc71',
    }

    # left: XY trajectory
    ax = axes[0]
    gt_plotted = False
    for name, res in all_results.items():
        if res is None:
            continue
        if not gt_plotted:
            ax.plot(res['_gt_pos'][:, 0], res['_gt_pos'][:, 1],
                    'k-', lw=2, alpha=0.8, label='Ground Truth')
            ax.plot(res['_gt_pos'][0, 0], res['_gt_pos'][0, 1],
                    'go', ms=10, mec='k', mew=1.5, zorder=5, label='Start')
            ax.plot(res['_gt_pos'][-1, 0], res['_gt_pos'][-1, 1],
                    'rs', ms=10, mec='k', mew=1.5, zorder=5, label='End')
            gt_plotted = True
        color = colors.get(name, '#95a5a6')
        label = f"{name} (ATE={res['ate_rmse']:.1f}m)"
        ax.plot(res['_aligned_pos'][:, 0], res['_aligned_pos'][:, 1],
                '-', color=color, lw=1.2, alpha=0.8, label=label)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(f'Trajectory Comparison - {session}', fontsize=14,
                 fontweight='bold')
    ax.legend(fontsize=10, loc='best')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # right: ATE over time
    ax = axes[1]
    for name, res in all_results.items():
        if res is None:
            continue
        color = colors.get(name, '#95a5a6')
        t = res['_timestamps'] - res['_timestamps'][0]
        ax.plot(t, res['_ate_errors'], '-', color=color, lw=0.8, alpha=0.7,
                label=f"{name} (RMSE={res['ate_rmse']:.1f}m)")

    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Position Error (m)', fontsize=12)
    ax.set_title('ATE Over Time', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = output_dir / f"trajectory_comparison_{session}.png"
    plt.savefig(str(plot_path), dpi=150, bbox_inches='tight')
    plt.close()
    log(f"  Plot saved: {plot_path.name}")


def plot_ate_comparison(all_session_results, output_dir):
    """plot ATE bar chart comparison across sessions and configs"""
    output_dir = Path(output_dir)

    sessions = list(all_session_results.keys())
    config_names = ['Config A (VIO)', 'Config B (+Wheel)', 'Config C (+GPS)']
    colors = ['#e74c3c', '#3498db', '#2ecc71']

    fig, ax = plt.subplots(figsize=(12, 6))

    x = np.arange(len(sessions))
    width = 0.25

    for ci, (cname, color) in enumerate(zip(config_names, colors)):
        values = []
        for s in sessions:
            res = all_session_results[s].get(cname)
            values.append(res['ate_rmse'] if res else 0)
        bars = ax.bar(x + ci * width - width, values, width,
                      label=cname, color=color, alpha=0.85, edgecolor='black')
        for bar, val in zip(bars, values):
            if val > 0:
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                        f'{val:.1f}', ha='center', fontsize=9, fontweight='bold')

    ax.set_xlabel('Session', fontsize=12)
    ax.set_ylabel('ATE RMSE (m)', fontsize=12)
    ax.set_title('VINS-Fusion: ATE Comparison Across Configurations',
                 fontsize=14, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels(sessions, fontsize=11)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout()
    plot_path = output_dir / "ate_comparison.png"
    plt.savefig(str(plot_path), dpi=150, bbox_inches='tight')
    plt.close()
    log(f"  ATE comparison plot saved: {plot_path.name}")


# summary generation
def generate_summary(all_session_results, timing_info, output_dir):
    """generate text and JSON summaries of all results"""
    output_dir = Path(output_dir)
    lines = []
    lines.append("=" * 80)
    lines.append("VINS-Fusion Mono-Inertial Experiment Results")
    lines.append(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 80)
    lines.append("")

    config_names = ['Config A (VIO)', 'Config B (+Wheel)', 'Config C (+GPS)']

    # per-session tables
    for session in all_session_results:
        lines.append(f"Session: {session}")
        lines.append("-" * 70)
        header = (f"  {'Config':<22} {'ATE RMSE':>10} {'ATE Mean':>10} "
                  f"{'RPE Trans':>10} {'Scale':>8} {'Poses':>7}")
        lines.append(header)
        lines.append("  " + "-" * 68)

        for cname in config_names:
            res = all_session_results[session].get(cname)
            if res is None:
                lines.append(f"  {cname:<22} {'FAILED':>10}")
                continue
            line = (f"  {cname:<22} {res['ate_rmse']:>10.2f} "
                    f"{res['ate_mean']:>10.2f} "
                    f"{res['rpe_trans_rmse']:>10.4f} "
                    f"{res['scale']:>8.4f} "
                    f"{res['n_poses']:>7}")
            lines.append(line)

        lines.append("")

    # timing
    lines.append("Timing:")
    lines.append("-" * 50)
    total_time = 0
    for session in timing_info:
        for cname, t in timing_info[session].items():
            lines.append(f"  {session} / {cname}: {t:.1f}s")
            total_time += t
    lines.append(f"  Total: {total_time:.1f}s ({total_time/60:.1f} min)")
    lines.append("")

    # best results
    lines.append("Best Results:")
    lines.append("-" * 50)
    for cname in config_names:
        best_ate = float('inf')
        best_session = None
        for session in all_session_results:
            res = all_session_results[session].get(cname)
            if res and res['ate_rmse'] < best_ate:
                best_ate = res['ate_rmse']
                best_session = session
        if best_session:
            lines.append(f"  {cname}: ATE RMSE = {best_ate:.2f}m ({best_session})")
    lines.append("")

    summary_text = "\n".join(lines)
    log(f"\n{summary_text}")

    # save text summary
    with open(output_dir / "summary.txt", 'w') as f:
        f.write(summary_text + "\n")

    # save JSON summary
    json_results = {}
    for session in all_session_results:
        json_results[session] = {}
        for cname in config_names:
            res = all_session_results[session].get(cname)
            if res:
                # remove non-serializable numpy arrays
                clean_res = {k: v for k, v in res.items()
                             if not k.startswith('_')}
                json_results[session][cname] = clean_res

    json_summary = {
        'experiment': 'week0_vins_fusion',
        'date': datetime.now().isoformat(),
        'sessions': list(all_session_results.keys()),
        'configs': config_names,
        'results': json_results,
        'timing': {s: {c: float(t) for c, t in ti.items()}
                   for s, ti in timing_info.items()},
    }
    with open(output_dir / "summary.json", 'w') as f:
        json.dump(json_summary, f, indent=2)

    log(f"  Summary saved to {output_dir / 'summary.txt'} and summary.json")


# copy ground truth to results
def copy_ground_truth(sessions, output_dir):
    """copy ground truth TUM files to the results directory"""
    gt_out_dir = Path(output_dir) / "ground_truth"
    gt_out_dir.mkdir(parents=True, exist_ok=True)

    for session in sessions:
        src = GT_TUM_DIR / f"gt_{session}.tum"
        dst = gt_out_dir / f"gt_{session}.tum"
        if src.exists():
            shutil.copy2(src, dst)
            log(f"  Copied GT: {dst.name}")
        else:
            # generate from CSV
            log(f"  GT TUM not found, generating from CSV...")
            gt = load_ground_truth_tum(session)
            np.savetxt(str(dst), gt, fmt='%.6f',
                       header='timestamp x y z qx qy qz qw')
            log(f"  Generated GT: {dst.name} ({len(gt)} poses)")


def main():
    """main entry point for the VINS-Fusion experiment pipeline"""
    global START_TIME
    START_TIME = time.time()

    parser = argparse.ArgumentParser(
        description='Experiment 0.5: VINS-Fusion Mono-Inertial on NCLT')
    parser.add_argument('--sessions', nargs='+', default=SESSIONS,
                        help='NCLT sessions to process')
    parser.add_argument('--skip-vins', action='store_true',
                        help='Skip VINS-Fusion binary run, load existing output')
    parser.add_argument('--max-images', type=int, default=0,
                        help='Maximum images per session (0=all)')
    parser.add_argument('--timeout', type=int, default=DEFAULT_TIMEOUT,
                        help=f'VINS-Fusion binary timeout in seconds '
                             f'(default: {DEFAULT_TIMEOUT})')
    parser.add_argument('--skip-config-b', action='store_true',
                        help='Skip Config B (wheel odometry fusion)')
    parser.add_argument('--skip-config-c', action='store_true',
                        help='Skip Config C (GPS fusion)')
    args = parser.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    setup_logging(RESULTS_DIR / "run_log.txt")

    log("=" * 70)
    log("VINS-Fusion Mono-Inertial Experiment on NCLT")
    log(f"Sessions: {args.sessions}")
    log(f"Max images: {args.max_images if args.max_images > 0 else 'all'}")
    log(f"Timeout: {args.timeout}s ({args.timeout/60:.0f} min)")
    log(f"Skip VINS: {args.skip_vins}")
    log(f"Results: {RESULTS_DIR}")
    log("=" * 70)

    # copy ground truth
    copy_ground_truth(args.sessions, RESULTS_DIR)

    all_session_results = {}
    timing_info = {}

    for session in args.sessions:
        log(f"\n{'#' * 70}")
        log(f"# SESSION: {session}")
        log(f"{'#' * 70}")

        all_session_results[session] = {}
        timing_info[session] = {}

        work_dir = Path(f"/tmp/vins_nclt_{session}")        # config A: Pure VIO
        log(f"\n--- Config A: Mono + IMU (VINS-Fusion VIO) ---")
        config_a_dir = RESULTS_DIR / "config_a_vio" / session
        config_a_dir.mkdir(parents=True, exist_ok=True)
        vio_traj = None

        t0 = time.time()
        try:
            if not args.skip_vins:
                # prepare data
                log(f"  Preparing data for VINS-Fusion...")
                prep_result = prepare_vins_data(
                    session, work_dir, max_images=args.max_images)
                (config_yaml, image_dir, ts_path, imu_path,
                 n_images, n_imu) = prep_result
                log(f"  Prepared: {n_images} images, {n_imu} IMU samples")

                # save preparation stats
                with open(config_a_dir / "stats.json", 'w') as f:
                    json.dump({
                        'session': session,
                        'n_images': n_images,
                        'n_imu_samples': n_imu,
                        'work_dir': str(work_dir),
                    }, f, indent=2)

                # run VINS-Fusion
                vio_traj = run_vins_fusion(session, work_dir,
                                           timeout=args.timeout)
            else:
                # try to load existing VIO output
                vio_output = work_dir / "output" / "vio.csv"
                if vio_output.exists():
                    raw = np.loadtxt(str(vio_output))
                    if raw.ndim == 1:
                        raw = raw.reshape(1, -1)
                    vio_traj = np.zeros((len(raw), 8))
                    vio_traj[:, 0] = raw[:, 0]
                    vio_traj[:, 1:4] = raw[:, 1:4]
                    vio_traj[:, 4] = raw[:, 5]  # qx
                    vio_traj[:, 5] = raw[:, 6]  # qy
                    vio_traj[:, 6] = raw[:, 7]  # qz
                    vio_traj[:, 7] = raw[:, 4]  # qw
                    log(f"  Loaded existing VIO: {len(vio_traj)} poses")
                else:
                    # also check the config_a output dir
                    raw_path = config_a_dir / "trajectory_raw.txt"
                    if raw_path.exists():
                        vio_traj = np.loadtxt(str(raw_path))
                        log(f"  Loaded existing VIO from results: "
                            f"{len(vio_traj)} poses")
                    else:
                        log(f"  No existing VIO output found")

        except Exception as e:
            log(f"  ERROR in Config A preparation/run: {e}")
            traceback.print_exc()

        t_config_a = time.time() - t0
        timing_info[session]['Config A (VIO)'] = t_config_a
        log(f"  Config A time: {t_config_a:.1f}s")

        # evaluate Config A
        config_a_result = None
        if vio_traj is not None and len(vio_traj) >= 10:
            try:
                gt = load_ground_truth_tum(session)
                config_a_result = evaluate_trajectory(
                    vio_traj, gt, 'Config A (VIO)', session,
                    config_a_dir, use_sim3=True)
                all_session_results[session]['Config A (VIO)'] = config_a_result
            except Exception as e:
                log(f"  ERROR evaluating Config A: {e}")
                traceback.print_exc()
        else:
            log(f"  Skipping Config A evaluation (no trajectory)")

        # save VIO trajectory for downstream configs
        if vio_traj is not None:
            np.savetxt(config_a_dir / "vio_trajectory.txt", vio_traj,
                       fmt='%.6f', header='timestamp x y z qx qy qz qw')        # config B: VIO + Wheel Odometry
        if not args.skip_config_b and vio_traj is not None:
            log(f"\n--- Config B: VIO + Wheel Odometry Fusion ---")
            config_b_dir = RESULTS_DIR / "config_b_wheel" / session
            config_b_dir.mkdir(parents=True, exist_ok=True)

            t0 = time.time()
            fused_traj = None
            try:
                fused_traj = fuse_wheel_odometry(vio_traj, session)
            except Exception as e:
                log(f"  ERROR in Config B fusion: {e}")
                traceback.print_exc()

            t_config_b = time.time() - t0
            timing_info[session]['Config B (+Wheel)'] = t_config_b
            log(f"  Config B time: {t_config_b:.1f}s")

            if fused_traj is not None and len(fused_traj) >= 10:
                try:
                    gt = load_ground_truth_tum(session)
                    config_b_result = evaluate_trajectory(
                        fused_traj, gt, 'Config B (+Wheel)', session,
                        config_b_dir, use_sim3=False)
                    all_session_results[session]['Config B (+Wheel)'] = config_b_result
                except Exception as e:
                    log(f"  ERROR evaluating Config B: {e}")
                    traceback.print_exc()
            else:
                log(f"  Skipping Config B evaluation (fusion failed)")
        elif args.skip_config_b:
            log(f"\n--- Config B: SKIPPED (--skip-config-b) ---")
        else:
            log(f"\n--- Config B: SKIPPED (no VIO trajectory) ---")        # config C: VIO + Wheel Odometry + GPS
        # use Config B output if available, otherwise fall back to Config A
        base_traj_for_gps = None
        if not args.skip_config_c:
            config_c_dir = RESULTS_DIR / "config_c_gps" / session
            config_c_dir.mkdir(parents=True, exist_ok=True)

            if (not args.skip_config_b and fused_traj is not None
                    and len(fused_traj) >= 10):
                base_traj_for_gps = fused_traj
                log(f"\n--- Config C: VIO + Wheel + GPS (from Config B) ---")
            elif vio_traj is not None and len(vio_traj) >= 10:
                base_traj_for_gps = vio_traj
                log(f"\n--- Config C: VIO + GPS (no wheel odom) ---")
            else:
                log(f"\n--- Config C: SKIPPED (no base trajectory) ---")

            if base_traj_for_gps is not None:
                t0 = time.time()
                gps_traj = None
                try:
                    gps_traj = fuse_gps(base_traj_for_gps, session)
                except Exception as e:
                    log(f"  ERROR in Config C fusion: {e}")
                    traceback.print_exc()

                t_config_c = time.time() - t0
                timing_info[session]['Config C (+GPS)'] = t_config_c
                log(f"  Config C time: {t_config_c:.1f}s")

                if gps_traj is not None and len(gps_traj) >= 10:
                    try:
                        gt = load_ground_truth_tum(session)
                        config_c_result = evaluate_trajectory(
                            gps_traj, gt, 'Config C (+GPS)', session,
                            config_c_dir, use_sim3=False)
                        all_session_results[session]['Config C (+GPS)'] = config_c_result
                    except Exception as e:
                        log(f"  ERROR evaluating Config C: {e}")
                        traceback.print_exc()
                else:
                    log(f"  Skipping Config C evaluation (GPS fusion failed)")
        else:
            log(f"\n--- Config C: SKIPPED (--skip-config-c) ---")        # per-session trajectory comparison plot
        session_results = all_session_results.get(session, {})
        if any(v is not None for v in session_results.values()):
            try:
                plot_trajectory_comparison(session_results, session, RESULTS_DIR)
            except Exception as e:
                log(f"  ERROR generating trajectory plot: {e}")
                traceback.print_exc()    # cross-session summary
    log(f"\n{'#' * 70}")
    log(f"# SUMMARY")
    log(f"{'#' * 70}")

    # ATE comparison plot
    if any(all_session_results[s] for s in all_session_results):
        try:
            plot_ate_comparison(all_session_results, RESULTS_DIR)
        except Exception as e:
            log(f"  ERROR generating ATE comparison plot: {e}")
            traceback.print_exc()

    # generate summary
    generate_summary(all_session_results, timing_info, RESULTS_DIR)

    total_time = time.time() - START_TIME
    log(f"\nTotal experiment time: {total_time:.1f}s ({total_time/60:.1f} min)")
    log(f"Results directory: {RESULTS_DIR}")    # suggest next steps
    log(f"\n{'=' * 70}")
    log("SUGGESTED NEXT STEPS:")
    log(f"{'=' * 70}")
    log("1. Review results in: results/week0_vins_fusion/summary.txt")
    log("2. Update CHANGELOG.md with experiment results")
    log("3. Commit changes:")
    log("   git add results/week0_vins_fusion/ scripts/run_week0_vins_fusion.py")
    log("   git commit -m 'Add VINS-Fusion mono-inertial experiment (Week 0.5)'")
    log(f"{'=' * 70}")

    # close log file
    if _file_logger:
        _file_logger.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
