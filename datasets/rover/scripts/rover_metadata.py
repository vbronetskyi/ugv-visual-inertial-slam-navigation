#!/usr/bin/env python3
"""gather metadata for all 15 ROVER recordings

for each recording computes:
  1. frame counts: cam_left (T265), color/rgb (D435i), depth (D435i)
  2. trajectory length from groundtruth.txt (sum of Euclidean distances)
  3. session duration from first/last GT timestamp
  4. GT frequency (poses / duration)
  5. trajectory bounding box (min/max x, y)
  6. IMU sample count from imu.txt in t265/ folder

outputs JSON to /workspace/datasets/rover/results/session_metadata.json
and prints a formatted table to stdout
"""

import json
import math
import os
import sys

DATA_ROOT = "/workspace/data/rover"
OUTPUT_PATH = "/workspace/datasets/rover/results/session_metadata.json"

EXCLUDE_SUFFIXES = ("_euroc", "_rgbd")
EXCLUDE_NAMES = {"__MACOSX", "calibration", ".cache"}


def get_recording_dirs(root):
    dirs = []
    for name in sorted(os.listdir(root)):
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        if name in EXCLUDE_NAMES:
            continue
        if any(name.endswith(s) for s in EXCLUDE_SUFFIXES):
            continue
        dirs.append(name)
    return dirs


def count_images(folder):
    if not os.path.isdir(folder):
        return 0
    count = 0
    for f in os.listdir(folder):
        if f.lower().endswith((".png", ".jpg", ".jpeg")):
            count += 1
    return count


def parse_groundtruth(gt_path):
    poses = []
    if not os.path.isfile(gt_path):
        return poses
    with open(gt_path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            poses.append((ts, tx, ty, tz, qx, qy, qz, qw))
    return poses


def compute_trajectory_length(poses):
    total = 0.0
    for i in range(1, len(poses)):
        dx = poses[i][1] - poses[i-1][1]
        dy = poses[i][2] - poses[i-1][2]
        dz = poses[i][3] - poses[i-1][3]
        total += math.sqrt(dx*dx + dy*dy + dz*dz)
    return total


def compute_bounding_box(poses):
    if not poses:
        return (0.0, 0.0, 0.0, 0.0)
    xs = [p[1] for p in poses]
    ys = [p[2] for p in poses]
    return (min(xs), max(xs), min(ys), max(ys))


def count_imu_samples(imu_path):
    if not os.path.isfile(imu_path):
        return 0
    count = 0
    with open(imu_path, "r") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                count += 1
    return count


def process_recording(root, name):
    base = os.path.join(root, name)

    # 1. frame counts
    cam_left_dir = os.path.join(base, "realsense_T265", "cam_left")
    d435i_color_dir = os.path.join(base, "realsense_D435i", "rgb")
    d435i_depth_dir = os.path.join(base, "realsense_D435i", "depth")

    n_cam_left = count_images(cam_left_dir)
    n_color = count_images(d435i_color_dir)
    n_depth = count_images(d435i_depth_dir)

    # 2-5. groundtruth analysis
    gt_path = os.path.join(base, "groundtruth.txt")
    poses = parse_groundtruth(gt_path)

    n_gt_poses = len(poses)
    traj_length = compute_trajectory_length(poses)

    if n_gt_poses >= 2:
        duration = poses[-1][0] - poses[0][0]
    else:
        duration = 0.0

    gt_freq = n_gt_poses / duration if duration > 0 else 0.0

    min_x, max_x, min_y, max_y = compute_bounding_box(poses)

    # 6. IMU samples
    imu_path = os.path.join(base, "realsense_T265", "imu", "imu.txt")
    n_imu = count_imu_samples(imu_path)

    return {
        "recording": name,
        "frames": {
            "cam_left_t265": n_cam_left,
            "color_d435i": n_color,
            "depth_d435i": n_depth,
        },
        "gt_poses": n_gt_poses,
        "trajectory_length_m": round(traj_length, 4),
        "duration_s": round(duration, 4),
        "gt_frequency_hz": round(gt_freq, 4),
        "bounding_box": {
            "min_x": round(min_x, 4),
            "max_x": round(max_x, 4),
            "min_y": round(min_y, 4),
            "max_y": round(max_y, 4),
        },
        "imu_samples_t265": n_imu,
    }


def print_table(results):
    hdr = (
        f"{'Recording':<45} "
        f"{'T265':>6} "
        f"{'Color':>6} "
        f"{'Depth':>6} "
        f"{'GT':>6} "
        f"{'TrajLen(m)':>11} "
        f"{'Dur(s)':>9} "
        f"{'GTHz':>7} "
        f"{'BBox X':>16} "
        f"{'BBox Y':>16} "
        f"{'IMU':>8}"
    )
    sep = "-" * len(hdr)
    print(sep)
    print(hdr)
    print(sep)

    for r in results:
        bb = r["bounding_box"]
        bbox_x = f"[{bb['min_x']:.2f}, {bb['max_x']:.2f}]"
        bbox_y = f"[{bb['min_y']:.2f}, {bb['max_y']:.2f}]"
        print(
            f"{r['recording']:<45} "
            f"{r['frames']['cam_left_t265']:>6} "
            f"{r['frames']['color_d435i']:>6} "
            f"{r['frames']['depth_d435i']:>6} "
            f"{r['gt_poses']:>6} "
            f"{r['trajectory_length_m']:>11.4f} "
            f"{r['duration_s']:>9.2f} "
            f"{r['gt_frequency_hz']:>7.2f} "
            f"{bbox_x:>16} "
            f"{bbox_y:>16} "
            f"{r['imu_samples_t265']:>8}"
        )
    print(sep)


def main():
    recording_names = get_recording_dirs(DATA_ROOT)
    print(f"Found {len(recording_names)} recordings:\n")
    for name in recording_names:
        print(f"  {name}")
    print()

    results = []
    for name in recording_names:
        print(f"Processing {name} ...")
        meta = process_recording(DATA_ROOT, name)
        results.append(meta)

    print()
    print_table(results)

    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nJSON written to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
