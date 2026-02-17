#!/usr/bin/env python3
"""prepare data for two SI experiments:
1. D435i IMU + T265 PinHole Stereo (replace IMU in existing pinhole_euroc)
2. KB8 SI (original fisheye images in EuRoC format, with T265 IMU)

Usage:
    python3 prepare_si_experiments.py <recording_name> [recording_name2 ...]
"""

import csv
import os
import shutil
import sys
from pathlib import Path

DATA_DIR = "/workspace/data/rover"


def extract_timestamp(filename):
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


def convert_imu_to_euroc(imu_txt_path, output_csv_path):
    """convert ROVER IMU txt (ts,acc,gyro) to EuRoC csv (ts_ns,gyro,acc)"""
    with open(imu_txt_path) as fin, open(output_csv_path, 'w', newline='') as fout:
        writer = csv.writer(fout)
        writer.writerow(['#timestamp [ns]', 'w_RS_S_x', 'w_RS_S_y', 'w_RS_S_z',
                         'a_RS_S_x', 'a_RS_S_y', 'a_RS_S_z'])
        count = 0
        for line in fin:
            parts = line.strip().split(',')
            if len(parts) < 7:
                continue
            try:
                ts = float(parts[0])
                ax, ay, az = float(parts[1]), float(parts[2]), float(parts[3])
                gx, gy, gz = float(parts[4]), float(parts[5]), float(parts[6])
            except ValueError:
                continue
            ts_ns = int(ts * 1e9)
            writer.writerow([ts_ns, gx, gy, gz, ax, ay, az])
            count += 1
    return count


def prepare_d435i_imu(rec_name):
    """Replace T265 IMU with D435i IMU in pinhole_euroc directory"""
    d435i_imu = Path(DATA_DIR) / rec_name / "realsense_D435i" / "imu" / "imu.txt"
    euroc_dir = Path(DATA_DIR) / f"{rec_name}_pinhole_euroc"
    target_dir = euroc_dir / "mav0" / "imu0_d435i"

    if not d435i_imu.exists():
        print(f"  [D435i IMU] SKIP: no D435i IMU for {rec_name}")
        return False

    if not euroc_dir.exists():
        print(f"  [D435i IMU] SKIP: no pinhole_euroc for {rec_name}")
        return False

    target_dir.mkdir(parents=True, exist_ok=True)
    output_csv = target_dir / "data.csv"

    count = convert_imu_to_euroc(str(d435i_imu), str(output_csv))
    print(f"  [D435i IMU] Converted {count} samples → {output_csv}")
    return True


def prepare_kb8_euroc(rec_name):
    """create EuRoC dir with original fisheye images (no undistortion)"""
    rec_dir = Path(DATA_DIR) / rec_name
    output_dir = Path(DATA_DIR) / f"{rec_name}_fisheye_euroc"

    left_dir = rec_dir / "realsense_T265" / "cam_left"
    right_dir = rec_dir / "realsense_T265" / "cam_right"
    imu_file = rec_dir / "realsense_T265" / "imu" / "imu.txt"
    gt_file = rec_dir / "groundtruth.txt"

    if not left_dir.exists():
        print(f"  [KB8] SKIP: no T265 cam_left for {rec_name}")
        return False

    cam0_dir = output_dir / "mav0" / "cam0" / "data"
    cam1_dir = output_dir / "mav0" / "cam1" / "data"
    imu_out_dir = output_dir / "mav0" / "imu0"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    imu_out_dir.mkdir(parents=True, exist_ok=True)

    # get sorted files
    left_files = sorted([f for f in os.listdir(left_dir) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(right_dir) if f.endswith('.png')])

    left_ts_map = {extract_timestamp(f): f for f in left_files}
    right_ts_map = {extract_timestamp(f): f for f in right_files}
    common_ts = sorted(set(left_ts_map.keys()) & set(right_ts_map.keys()))
    print(f"  [KB8] Matched {len(common_ts)} stereo pairs")

    # symlink original fisheye with ns timestamps
    times_list = []
    for i, ts in enumerate(common_ts):
        ts_ns = str(int(float(ts) * 1e9))

        src_l = os.path.abspath(str(left_dir / left_ts_map[ts]))
        src_r = os.path.abspath(str(right_dir / right_ts_map[ts]))
        dst_l = cam0_dir / f"{ts_ns}.png"
        dst_r = cam1_dir / f"{ts_ns}.png"

        if not dst_l.exists():
            os.symlink(src_l, dst_l)
        if not dst_r.exists():
            os.symlink(src_r, dst_r)
        times_list.append(ts_ns)

    # times.txt
    with open(output_dir / "times.txt", 'w') as f:
        for ts_ns in times_list:
            f.write(f"{ts_ns}\n")

    # imu
    if imu_file.exists():
        imu_csv = imu_out_dir / "data.csv"
        count = convert_imu_to_euroc(str(imu_file), str(imu_csv))
        print(f"  [KB8] IMU: {count} samples")

    # gt
    if gt_file.exists():
        shutil.copy2(str(gt_file), str(output_dir / "gt_tum.txt"))

    print(f"  [KB8] Done: {len(times_list)} pairs → {output_dir}")
    return True


def main():
    recordings = sys.argv[1:] if len(sys.argv) > 1 else [
        "campus_large_day_2024-09-25",
        "campus_large_autumn_2023-11-07",
    ]

    for rec in recordings:
        print(f"\n=== {rec} ===")
        prepare_d435i_imu(rec)
        prepare_kb8_euroc(rec)


if __name__ == "__main__":
    main()
