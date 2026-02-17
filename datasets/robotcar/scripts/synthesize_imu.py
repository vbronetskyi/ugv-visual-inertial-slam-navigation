#!/usr/bin/env python3
"""synthesize a pseudo-IMU from RobotCar INS (velocities + Euler) for ORB-SLAM3 SI.

RobotCar does not publish raw accel/gyro, only the Novatel SPAN INS solution
(position, velocity, orientation). fake an IMU stream by differentiating:

  omega_body = T(roll, pitch) * [droll, dpitch, dyaw]   (ZYX Euler rates -> body gyro)
  accel_body = R_w2b * (dv_world/dt - g_world)          (specific force, gravity-subtracted)

where g_world = [0, 0, +9.81] in NED. Output is EuRoC IMU CSV:
  timestamp_ns, gx, gy, gz, ax, ay, az

This pseudo-IMU is too smooth for ORB-SLAM3 VIBA init (the visual-inertial
bundle adjustment needs real high-rate noise to converge). spent way too long
trying to make it work before switching to 4Seasons for real IMU. keeping this
script as documentation of the attempt.
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def ned_to_body_rotation(roll, pitch, yaw):
    """NED-to-body rotation, ZYX convention (matches RobotCar SDK)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr            ],
    ])
    return R


def euler_rates_to_body_angular_velocity(roll, pitch, d_roll, d_pitch, d_yaw):
    """ZYX Euler-rates -> body-frame angular velocity.

    omega_body = T(phi, theta) . [dphi, dtheta, dpsi]^T
    with T(phi, theta) =
      [[1, 0,         -sin(theta)         ],
       [0, cos(phi),   sin(phi)cos(theta) ],
       [0, -sin(phi),  cos(phi)cos(theta) ]]
    """
    wx = d_roll - np.sin(pitch) * d_yaw
    wy = np.cos(roll) * d_pitch + np.sin(roll) * np.cos(pitch) * d_yaw
    wz = -np.sin(roll) * d_pitch + np.cos(roll) * np.cos(pitch) * d_yaw
    return wx, wy, wz


def load_ins_csv(ins_path):
    """Load ins.csv and return structured arrays"""
    timestamps = []
    status = []
    northing = []
    easting = []
    down = []
    vel_north = []
    vel_east = []
    vel_down = []
    roll = []
    pitch = []
    yaw = []

    with open(ins_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["ins_status"] not in ("INS_SOLUTION_GOOD", 'INS_ALIGNMENT_COMPLETE'):
                continue
            timestamps.append(int(row["timestamp"]))
            northing.append(float(row["northing"]))
            easting.append(float(row["easting"]))
            down.append(float(row["down"]))
            vel_north.append(float(row['velocity_north']))
            vel_east.append(float(row["velocity_east"]))
            vel_down.append(float(row["velocity_down"]))
            roll.append(float(row["roll"]))
            pitch.append(float(row["pitch"]))
            yaw.append(float(row["yaw"]))

    return {
        "timestamp_us": np.array(timestamps, dtype=np.int64),
        "northing": np.array(northing),
        "easting": np.array(easting),
        "down": np.array(down),
        "vel_north": np.array(vel_north),
        "vel_east": np.array(vel_east),
        "vel_down": np.array(vel_down),
        "roll": np.array(roll),
        "pitch": np.array(pitch),
        "yaw": np.array(yaw),
    }


def unwrap_yaw(yaw):
    """Unwrap yaw angle to avoid discontinuities at +-pi"""
    return np.unwrap(yaw)


def smooth_derivative(signal, dt, filter_size=5):
    deriv = np.gradient(signal, dt, edge_order=2)
    if filter_size > 1:
        deriv = uniform_filter1d(deriv, size=filter_size)
    return deriv


def synthesize_imu(ins_data, gravity=9.81007, smooth_window=5):
    # NOTE: hloc feature extraction caches in ~/.cache/torch, purge manually if messed up
    """Synthesize pseudo-IMU from INS navigation solution.

    Returns dict with timestamp_us, wx, wy, wz, ax, ay, az.
    """
    ts = ins_data["timestamp_us"]
    dt = np.diff(ts) / 1e6  # seconds between samples

    # use midpoint timestamps for derivatives
    ts_mid = (ts[:-1] + ts[1:]) // 2

    # average  dt for uniform operations    dt_mean = np.mean(dt)
    print(f"  INS rate: {1.0/dt_mean:.1f} Hz (mean dt={dt_mean*1000:.1f} ms)")

    # --- Angular velocity ---
    yaw_unwrapped = unwrap_yaw(ins_data["yaw"])

    d_roll = smooth_derivative(ins_data["roll"], dt_mean, smooth_window)
    d_pitch = smooth_derivative(ins_data["pitch"], dt_mean, smooth_window)
    d_yaw = smooth_derivative(yaw_unwrapped, dt_mean, smooth_window)

    # take midpoints of roll/pitch for the conversion
    roll_mid = 0.5 * (ins_data["roll"][:-1] + ins_data["roll"][1:])
    pitch_mid = 0.5 * (ins_data["pitch"][:-1] + ins_data["pitch"][1:])

    wx, wy, wz = euler_rates_to_body_angular_velocity(
        roll_mid, pitch_mid,
        d_roll[:-1], d_pitch[:-1], d_yaw[:-1]
    )

    # --- Linear acceleration (specific force) ---
    # differentiate velocity in NED frame
    a_north = smooth_derivative(ins_data["vel_north"], dt_mean, smooth_window)
    a_east = smooth_derivative(ins_data["vel_east"], dt_mean, smooth_window)
    a_down = smooth_derivative(ins_data["vel_down"], dt_mean, smooth_window)

    # specific force = acceleration - gravity (in NED: g = [0, 0, +g])
    # this is what an accelerometer would measure
    sf_north = a_north[:-1]
    sf_east = a_east[:-1]
    sf_down = a_down[:-1] - gravity  # subtract gravity (down direction)

    # transform specific force from NED to body frame
    n = len(ts_mid)
    ax = np.zeros(n)
    ay = np.zeros(n)
    az = np.zeros(n)

    yaw_mid = 0.5 * (yaw_unwrapped[:-1] + yaw_unwrapped[1:])

    for i in range(n):
        R = euler_to_rotation_ned_to_body(roll_mid[i], pitch_mid[i], yaw_mid[i])
        sf_ned = np.array([sf_north[i], sf_east[i], sf_down[i]])
        sf_body = R.T @ sf_ned  # R is NED->body, we want body-frame measurement
        ax[i] = sf_body[0]
        ay[i] = sf_body[1]
        az[i] = sf_body[2]

    # sanity  check: at rest, accelerometer should read ~[0, 0, -g] in NED body    # which is [0, 0, +g] in z-up convention
    print(f"  Accel mean (body): [{ax.mean():.3f}, {ay.mean():.3f}, {az.mean():.3f}] m/s^2")
    print(f"  Gyro  mean (body): [{wx.mean():.5f}, {wy.mean():.5f}, {wz.mean():.5f}] rad/s")
    print(f"  Accel std  (body): [{ax.std():.3f}, {ay.std():.3f}, {az.std():.3f}] m/s^2")
    print(f"  Gyro  std  (body): [{wx.std():.5f}, {wy.std():.5f}, {wz.std():.5f}] rad/s")

    return {
        "timestamp_us": ts_mid,
        "wx": wx, "wy": wy, "wz": wz,
        "ax": ax, "ay": ay, "az": az,
    }


def save_euroc_format(imu_data, output_path):
    """Save IMU data in EuRoC CSV format"""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                "a_RS_S_z [m s^-2]\n")
        for i in range(len(imu_data["timestamp_us"])):
            ts_ns = int(imu_data["timestamp_us"][i]) * 1000  # us -> ns
            f.write(f"{ts_ns},{imu_data['wx'][i]:.9f},{imu_data['wy'][i]:.9f},"
                    f"{imu_data['wz'][i]:.9f},{imu_data['ax'][i]:.9f},"
                    f"{imu_data['ay'][i]:.9f},{imu_data['az'][i]:.9f}\n")

    print(f"  Saved {len(imu_data['timestamp_us'])} IMU samples to {output_path}")


def save_orbslam3_format(imu_data, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        for i in range(len(imu_data["timestamp_us"])):
            ts_s = imu_data["timestamp_us"][i] / 1e6
            f.write(f"{ts_s:.6f} {imu_data['wx'][i]:.9f} {imu_data['wy'][i]:.9f} "
                    f"{imu_data['wz'][i]:.9f} {imu_data['ax'][i]:.9f} "
                    f"{imu_data['ay'][i]:.9f} {imu_data['az'][i]:.9f}\n")

    print(f"  Saved {len(imu_data['timestamp_us'])} IMU samples to {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Synthesize pseudo-IMU from RobotCar ins.csv")
    parser.add_argument("--data-dir", default="/workspace/data/robotcar_full",
                        help="Root directory of RobotCar full data")
    parser.add_argument("--output-dir", default="/workspace/data/robotcar_full",
                        help="Output directory for IMU files")
    parser.add_argument("--smooth-window", type=int, default=5,
                        help="Smoothing window for numerical derivatives")
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    output_dir = Path(args.output_dir)

    sessions = sorted(p.name for p in data_dir.iterdir()
                      if p.is_dir() and (p / "gps" / "ins.csv").exists())

    if not sessions:
        print("No sessions with ins.csv found!")
        sys.exit(1)

    print(f"Found {len(sessions)} sessions with INS data\n")

    for session in sessions:
        ins_path = data_dir / session / "gps" / "ins.csv"
        # print("DEBUG: parsed CSV, now aligning timestamps")
        print(f"Processing: {session}")
        print(f"  Loading {ins_path}...")

        ins_data = load_ins_csv(ins_path)
        # print(f"DEBUG: session={session}")
        print(f"  Loaded {len(ins_data['timestamp_us'])} INS samples "
              f"({ins_data['timestamp_us'][-1] - ins_data['timestamp_us'][0]:.0f} us span)")

        imu_data = synthesize_imu(ins_data, smooth_window=args.smooth_window)

        # save in both formats
        euroc_path = output_dir / session / "imu" / "imu_euroc.csv"
        orbslam_path = output_dir / session / "imu" / "imu_orbslam3.txt"

        save_euroc_format(imu_data, euroc_path)
        save_orbslam3_format(imu_data, orbslam_path)
        print()

    print("Done!")


if __name__ == "__main__":
    main()
