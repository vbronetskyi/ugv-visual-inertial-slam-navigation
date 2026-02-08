#!/usr/bin/env python3
"""Generate mock NCLT sensor data CSV files for testing.

Creates synthetic sensor data simulating a 10-second trajectory at the
University of Michigan North Campus.

Sensors generated:
    - ms25.csv        : IMU (accelerometer, gyroscope, magnetometer) at 100Hz
    - gps_rtk.csv     : GPS fix at 1Hz
    - odometry_mu_100hz.csv : wheel odometry at 100Hz
    - kvh.csv         : fiber optic gyro heading at 100Hz
    - groundtruth.csv : ground truth pose at 10Hz
    - track.csv       : compatibility file for NCLTDataset (1Hz poses)

Usage:
    python scripts/generate_mock_sensors.py
"""

import os
from pathlib import Path

import numpy as np


def generate_mock_sensors(output_dir: str) -> None:
    """generate all mock sensor CSV files into output_dir"""
    rng = np.random.default_rng(seed=42)
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # common timing parameters
    t0 = 1326044400000000  # ~2012-01-08 in microseconds
    duration_s = 10.0
    dt_100hz = 10000  # 10 ms in microseconds
    dt_10hz = 100000  # 100 ms in microseconds
    dt_1hz = 1000000  # 1 s in microseconds

    n_100hz = 1000
    n_10hz = 100
    n_1hz = 10

    utimes_100hz = t0 + np.arange(n_100hz) * dt_100hz
    utimes_10hz = t0 + np.arange(n_10hz) * dt_10hz
    utimes_1hz = t0 + np.arange(n_1hz) * dt_1hz

    # trajectory: moving slowly northeast at ~1 m/s
    # heading ~45 deg = 0.785 rad (NE direction)
    heading_start = 0.785
    speed = 1.0  # m/s
    vx = speed * np.cos(heading_start)  # ~0.707 m/s
    vy = speed * np.sin(heading_start)  # ~0.707 m/s

    # ----------------------------------------------------------------
    # 1. ms25.csv - IMU at 100Hz (1000 rows)
    #    Columns: utime, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z,
    #             rot_x, rot_y, rot_z
    # ----------------------------------------------------------------
    mag_x = 0.2 + rng.normal(0, 0.005, n_100hz)
    mag_y = -0.05 + rng.normal(0, 0.005, n_100hz)
    mag_z = 0.4 + rng.normal(0, 0.005, n_100hz)

    accel_x = rng.normal(0, 0.02, n_100hz)
    accel_y = rng.normal(0, 0.02, n_100hz)
    accel_z = -9.81 + rng.normal(0, 0.02, n_100hz)

    rot_x = rng.normal(0, 0.01, n_100hz)
    rot_y = rng.normal(0, 0.01, n_100hz)
    rot_z = rng.normal(0, 0.01, n_100hz)

    imu_data = np.column_stack([
        utimes_100hz, mag_x, mag_y, mag_z,
        accel_x, accel_y, accel_z,
        rot_x, rot_y, rot_z,
    ])

    np.savetxt(
        str(out / "ms25.csv"), imu_data, delimiter=",",
        fmt=["%d"] + ["%.8f"] * 9,
    )
    print(f"  ms25.csv          : {imu_data.shape[0]} rows, {imu_data.shape[1]} cols")

    # ----------------------------------------------------------------
    # 2. gps_rtk.csv - GPS at 1Hz (10 rows)
    #    Columns: utime, mode, num_satell, latitude, longitude,
    #             altitude, track, speed
    # ----------------------------------------------------------------
    lat_start = 42.293195
    lon_start = -83.709657
    alt_base = 270.0

    # approximate meters-to-degrees at Ann Arbor latitude
    # 1 deg lat ~ 111,000 m, 1 deg lon ~ 111,000 * cos(42.29 deg) ~ 82,200 m
    m_per_deg_lat = 111000.0
    m_per_deg_lon = 111000.0 * np.cos(np.radians(lat_start))

    t_1hz_s = np.arange(n_1hz)  # 0..9 seconds
    lat = lat_start + (vx * t_1hz_s) / m_per_deg_lat + rng.normal(0, 1e-7, n_1hz)
    lon = lon_start + (vy * t_1hz_s) / m_per_deg_lon + rng.normal(0, 1e-7, n_1hz)
    alt = alt_base + rng.normal(0, 0.5, n_1hz)
    mode = np.full(n_1hz, 2, dtype=int)
    num_satell = np.full(n_1hz, 8, dtype=int)
    track_angle = np.full(n_1hz, np.degrees(heading_start)) + rng.normal(0, 0.5, n_1hz)
    gps_speed = speed + rng.normal(0, 0.05, n_1hz)

    gps_data = np.column_stack([
        utimes_1hz, mode, num_satell, lat, lon, alt, track_angle, gps_speed,
    ])

    np.savetxt(
        str(out / "gps_rtk.csv"), gps_data, delimiter=",",
        fmt=["%d", "%d", "%d", "%.10f", "%.10f", "%.4f", "%.4f", "%.4f"],
    )
    print(f"  gps_rtk.csv       : {gps_data.shape[0]} rows, {gps_data.shape[1]} cols")

    # ----------------------------------------------------------------
    # 3. odometry_mu_100hz.csv - Wheel odometry at 100Hz (1000 rows)
    #    Columns: utime, x, y, z, roll, pitch, yaw
    # ----------------------------------------------------------------
    t_100hz_s = np.arange(n_100hz) * 0.01  # 0..9.99 seconds

    odom_x = vx * t_100hz_s + rng.normal(0, 0.005, n_100hz)
    odom_y = vy * t_100hz_s + rng.normal(0, 0.005, n_100hz)
    odom_z = rng.normal(0, 0.002, n_100hz)
    odom_roll = np.zeros(n_100hz)
    odom_pitch = np.zeros(n_100hz)
    odom_yaw = np.full(n_100hz, heading_start)

    odom_data = np.column_stack([
        utimes_100hz, odom_x, odom_y, odom_z,
        odom_roll, odom_pitch, odom_yaw,
    ])

    np.savetxt(
        str(out / "odometry_mu_100hz.csv"), odom_data, delimiter=",",
        fmt=["%d", "%.6f", "%.6f", "%.6f", "%.6f", "%.6f", "%.6f"],
    )
    print(f"  odometry_mu_100hz : {odom_data.shape[0]} rows, {odom_data.shape[1]} cols")

    # ----------------------------------------------------------------
    # 4. kvh.csv - Fiber optic gyro at 100Hz (1000 rows)
    #    Columns: utime, heading
    # ----------------------------------------------------------------
    # slow linear drift of ~0.001 rad/s + small noise
    heading_drift_rate = 0.001  # rad/s
    heading = (
        heading_start
        + heading_drift_rate * t_100hz_s
        + rng.normal(0, 0.0005, n_100hz)
    )

    kvh_data = np.column_stack([utimes_100hz, heading])

    np.savetxt(
        str(out / "kvh.csv"), kvh_data, delimiter=",",
        fmt=["%d", "%.8f"],
    )
    print(f"  kvh.csv           : {kvh_data.shape[0]} rows, {kvh_data.shape[1]} cols")

    # ----------------------------------------------------------------
    # 5. groundtruth.csv - Ground truth at 10Hz (100 rows)
    #    Columns: utime, x, y, z, roll, pitch, yaw
    # ----------------------------------------------------------------
    t_10hz_s = np.arange(n_10hz) * 0.1  # 0..9.9 seconds

    gt_x = vx * t_10hz_s
    gt_y = vy * t_10hz_s
    gt_z = np.zeros(n_10hz)
    gt_roll = rng.normal(0, 0.001, n_10hz)
    gt_pitch = rng.normal(0, 0.001, n_10hz)
    gt_yaw = heading_start + heading_drift_rate * t_10hz_s

    gt_data = np.column_stack([
        utimes_10hz, gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw,
    ])

    np.savetxt(
        str(out / "groundtruth.csv"), gt_data, delimiter=",",
        fmt=["%d", "%.6f", "%.6f", "%.6f", "%.8f", "%.8f", "%.8f"],
    )
    print(f"  groundtruth.csv   : {gt_data.shape[0]} rows, {gt_data.shape[1]} cols")

    # ----------------------------------------------------------------
    # 6. track.csv - NCLTDataset compatibility (1Hz, 10 rows)
    #    Columns: timestamp, x, y, z, roll, pitch, yaw
    # ----------------------------------------------------------------
    track_x = vx * t_1hz_s
    track_y = vy * t_1hz_s
    track_z = np.zeros(n_1hz)
    track_roll = rng.normal(0, 0.001, n_1hz)
    track_pitch = rng.normal(0, 0.001, n_1hz)
    track_yaw = heading_start + heading_drift_rate * t_1hz_s

    track_data = np.column_stack([
        utimes_1hz, track_x, track_y, track_z,
        track_roll, track_pitch, track_yaw,
    ])

    np.savetxt(
        str(out / "track.csv"), track_data, delimiter=",",
        fmt=["%d", "%.6f", "%.6f", "%.6f", "%.8f", "%.8f", "%.8f"],
    )
    print(f"  track.csv         : {track_data.shape[0]} rows, {track_data.shape[1]} cols")


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent
    output_dir = project_root / "data" / "nclt_mock" / "2012-01-08"

    print(f"Generating mock NCLT sensor data in:\n  {output_dir}\n")
    generate_mock_sensors(str(output_dir))
    print(f"\nDone. All files written to {output_dir}")


if __name__ == "__main__":
    main()
