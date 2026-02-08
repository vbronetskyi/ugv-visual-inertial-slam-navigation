#!/usr/bin/env python3
"""Generate GT trajectory from RobotCar INS data, interpolated at camera timestamps

Output: TUM format, relative to first camera frame
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def euler_to_se3(x, y, z, roll, pitch, yaw):
    """Create SE3 matrix from position and Euler angles (ZYX convention)"""
    R = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def load_ins(ins_path):
    """Load INS data and return timestamps + SE3 poses"""
    timestamps_us = []
    poses = []

    with open(ins_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["ins_status"] not in ("INS_SOLUTION_GOOD", "INS_ALIGNMENT_COMPLETE"):
                continue
            ts = int(row["timestamp"])
            n = float(row["northing"])
            e = float(row["easting"])
            d = float(row["down"])
            roll = float(row["roll"])
            pitch = float(row["pitch"])
            yaw = float(row["yaw"])

            T = euler_to_se3(n, e, d, roll, pitch, yaw)
            timestamps_us.append(ts)
            poses.append(T)

    return np.array(timestamps_us), poses


def interpolate_pose(ts_query, ts_array, poses):
    """Interpolate pose at query timestamp using SLERP + linear position"""
    idx = np.searchsorted(ts_array, ts_query)
    if idx <= 0:
        return poses[0]
    if idx >= len(ts_array):
        return poses[-1]

    t0, t1 = ts_array[idx - 1], ts_array[idx]
    alpha = (ts_query - t0) / (t1 - t0) if t1 != t0 else 0.0

    # linear position interpolation
    pos0 = poses[idx - 1][:3, 3]
    pos1 = poses[idx][:3, 3]
    pos = (1 - alpha) * pos0 + alpha * pos1

    # slerp rotation
    R0 = Rotation.from_matrix(poses[idx - 1][:3, :3])
    R1 = Rotation.from_matrix(poses[idx][:3, :3])
    slerp_times = [0, 1]
    from scipy.spatial.transform import Slerp
    slerp = Slerp(slerp_times, Rotation.concatenate([R0, R1]))
    R_interp = slerp([alpha])[0]

    T = np.eye(4)
    T[:3, :3] = R_interp.as_matrix()
    T[:3, 3] = pos
    return T


def main():
    parser = argparse.ArgumentParser(description="Generate GT trajectory from INS")
    parser.add_argument("--ins-path", required=True, help="Path to ins.csv")
    parser.add_argument("--timestamps-path", required=True,
                        help="Path to camera timestamps file (ns)")
    parser.add_argument("--output", required=True, help="Output TUM trajectory file")
    parser.add_argument("--max-images", type=int, default=None,
                        help="Limit to first N images")
    args = parser.parse_args()

    print(f"Loading INS data from {args.ins_path}...")
    ins_ts, ins_poses = load_ins(args.ins_path)
    print(f"  {len(ins_ts)} INS poses loaded")

    print(f"Loading camera timestamps from {args.timestamps_path}...")
    cam_ts_ns = []
    with open(args.timestamps_path) as f:
        for line in f:
            line = line.strip()
            if line:
                cam_ts_ns.append(int(line))

    if args.max_images:
        cam_ts_ns = cam_ts_ns[:args.max_images]

    cam_ts_us = [ts // 1000 for ts in cam_ts_ns]
    print(f"  {len(cam_ts_us)} camera timestamps loaded")

    # get origin pose (first camera frame)
    T_origin = interpolate_pose(cam_ts_us[0], ins_ts, ins_poses)
    T_origin_inv = np.linalg.inv(T_origin)

    # interpolate and transform to local frame
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for i, ts_us in enumerate(cam_ts_us):
            T_world = interpolate_pose(ts_us, ins_ts, ins_poses)
            T_local = T_origin_inv @ T_world

            pos = T_local[:3, 3]
            quat = Rotation.from_matrix(T_local[:3, :3]).as_quat()  # xyzw

            ts_s = ts_us / 1e6
            f.write(f"{ts_s:.6f} {pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f} "
                    f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}\n")

    print(f"Saved {len(cam_ts_us)} GT poses to {output_path}")

    # print trajectory stats
    positions = []
    for ts_us in cam_ts_us:
        T_world = interpolate_pose(ts_us, ins_ts, ins_poses)
        T_local = T_origin_inv @ T_world
        positions.append(T_local[:3, 3])
    positions = np.array(positions)

    total_dist = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
    extent = np.max(positions, axis=0) - np.min(positions, axis=0)
    print(f"  Total distance: {total_dist:.1f} m")
    print(f"  Extent (x,y,z): ({extent[0]:.1f}, {extent[1]:.1f}, {extent[2]:.1f}) m")
    print(f"  Duration: {(cam_ts_us[-1] - cam_ts_us[0]) / 1e6:.1f} s")


if __name__ == "__main__":
    main()
