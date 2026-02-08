#!/usr/bin/env python3
"""convert ROVER T265 stereo to EuRoC MAV format for ORB-SLAM3

ROVER T265 structure:
  {recording}/realsense_T265/cam_left/{timestamp_s}.png   (grayscale 848x800)
  {recording}/realsense_T265/cam_right/{timestamp_s}.png
  {recording}/realsense_T265/imu/imu.txt  (CSV: ts_s,ax,ay,az,gx,gy,gz)
  {recording}/groundtruth.txt  (TUM: ts_s tx ty tz qx qy qz qw)

EuRoC MAV structure (what ORB-SLAM3 expects):
  {out}/mav0/cam0/data/{ts_ns}.png
  {out}/mav0/cam1/data/{ts_ns}.png
  {out}/mav0/imu0/data.csv  (CSV: ts_ns,gx,gy,gz,ax,ay,az -- gyro BEFORE acc!)
  {out}/times.txt  (one ts_ns per line)
  {out}/gt_tum.txt  (ground truth in TUM format)

Usage:
  python3 convert_rover_to_euroc.py /workspace/data/rover/garden_large_day_2024-05-29_1 \
      --output /workspace/data/rover/garden_large_day_2024-05-29_1_euroc
"""

import argparse
import glob
import os
import shutil
import sys


def ts_float_to_ns(ts_str):
    """float-seconds string to nanoseconds string"""
    ts_float = float(ts_str)
    ts_ns = int(round(ts_float * 1e9))
    return str(ts_ns)


def extract_timestamp(filename):
    """extract timestamp from ROVER image filename

    handles two naming conventions:
      - '1716995606.7813609.png'  -> '1716995606.7813609'
      - 'left_img_10000_1692363424.0364683.png'  -> '1692363424.0364683'
    """
    name = filename.replace('.png', '')
    #if underscored name, timestamp is the last part
    if '_' in name:
        parts = name.split('_')
        # find the part that looks like a unix timestamp
        for p in reversed(parts):
            try:
                val = float(p)
                if val > 1e9:  # Unix timestamp
                    return p
            except ValueError:
                continue
    return name


def setup_images(src_dir, dst_dir):
    """symlink images from ROVER to EuRoC format

    ROVER: {ts_seconds}.png or left_img_N_{ts_seconds}.png
    EuRoC: {ts_nanoseconds}.png

    returns sorted list of (ts_seconds_str, ts_ns_str) tuples
    """
    os.makedirs(dst_dir, exist_ok=True)
    pairs = []

    png_files = sorted(glob.glob(os.path.join(src_dir, "*.png")))
    if not png_files:
        print(f"  WARNING: No images found in {src_dir}")
        return pairs

    for src in png_files:
        fname = os.path.basename(src)
        ts_s_str = extract_timestamp(fname)
        ts_ns_str = ts_float_to_ns(ts_s_str)
        dst = os.path.join(dst_dir, ts_ns_str + '.png')
        if not os.path.exists(dst):
            os.symlink(os.path.abspath(src), dst)
        pairs.append((ts_s_str, ts_ns_str))

    print(f"  Images: {len(pairs)} frames -> {dst_dir}")
    return pairs


def convert_imu(imu_src, imu_dst):
    """convert ROVER T265 IMU to EuRoC format

    ROVER:  ts_seconds,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z  (comma-separated)
    EuRoC:  ts_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z       (gyro first!)
    """
    os.makedirs(os.path.dirname(imu_dst), exist_ok=True)
    count = 0
    with open(imu_src, 'r') as fin, open(imu_dst, 'w') as fout:
        fout.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
                   "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],"
                   "a_RS_S_z [m s^-2]\n")
        for line in fin:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) != 7:
                continue
            ts_s = parts[0]
            ax, ay, az = parts[1], parts[2], parts[3]
            gx, gy, gz = parts[4], parts[5], parts[6]
            ts_ns = ts_float_to_ns(ts_s)
            # EuRoC order: ts_ns, gyro_xyz, acc_xyz
            fout.write(f"{ts_ns},{gx},{gy},{gz},{ax},{ay},{az}\n")
            count += 1
    print(f"  IMU: {count} samples -> {imu_dst}")
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROVER T265 data to EuRoC format for ORB-SLAM3")
    parser.add_argument("recording_dir",
                        help="Path to ROVER recording (e.g., .../garden_large_day_2024-05-29_1)")
    parser.add_argument("--output", "-o", default=None,
                        help="Output directory (default: {recording}_euroc)")
    args = parser.parse_args()

    rec_dir = os.path.abspath(args.recording_dir)
    out_dir = args.output or (rec_dir + "_euroc")
    rec_name = os.path.basename(rec_dir)

    print(f"Converting ROVER -> EuRoC: {rec_name}")
    print(f"  Input:  {rec_dir}")
    print(f"  Output: {out_dir}")

    # check source exists
    cam_left = os.path.join(rec_dir, "realsense_T265", "cam_left")
    cam_right = os.path.join(rec_dir, "realsense_T265", "cam_right")
    imu_file = os.path.join(rec_dir, "realsense_T265", "imu", "imu.txt")
    gt_file = os.path.join(rec_dir, "groundtruth.txt")

    for path, name in [(cam_left, "T265 cam_left"), (cam_right, "T265 cam_right"),
                        (imu_file, "T265 IMU"), (gt_file, "Ground truth")]:
        if not os.path.exists(path):
            print(f"  ERROR: {name} not found: {path}")
            sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)

    # 1. setup left camera
    print("\n[1/5] Left camera (cam0)...")
    cam0_dst = os.path.join(out_dir, "mav0", "cam0", "data")
    pairs_left = setup_images(cam_left, cam0_dst)

    # 2. setup right camera
    print("[2/5] Right camera (cam1)...")
    cam1_dst = os.path.join(out_dir, "mav0", "cam1", "data")
    pairs_right = setup_images(cam_right, cam1_dst)

    # 3. verify stereo sync
    ts_left = set(p[0] for p in pairs_left)
    ts_right = set(p[0] for p in pairs_right)
    common = ts_left & ts_right
    if len(common) == len(ts_left) == len(ts_right):
        print(f"  Stereo sync OK: {len(common)} matched pairs")
    else:
        print(f"  WARNING: L={len(ts_left)}, R={len(ts_right)}, common={len(common)}")

    # use left cam timestamps (should match right)
    timestamps_ns = sorted([p[1] for p in pairs_left if p[0] in common])

    # 4. convert IMU
    print("[3/5] Converting IMU...")
    imu_dst = os.path.join(out_dir, "mav0", "imu0", "data.csv")
    convert_imu(imu_file, imu_dst)

    # 5. create times.txt
    print("[4/5] Creating times.txt...")
    times_path = os.path.join(out_dir, "times.txt")
    with open(times_path, 'w') as f:
        for ts in timestamps_ns:
            f.write(ts + '\n')
    print(f"  Times: {len(timestamps_ns)} entries")

    # 6. copy ground truth
    print("[5/5] Copying ground truth...")
    gt_dst = os.path.join(out_dir, "gt_tum.txt")
    shutil.copy2(gt_file, gt_dst)
    print(f"  GT copied to {gt_dst}")

    # summary
    if timestamps_ns:
        t0 = int(timestamps_ns[0]) / 1e9
        t1 = int(timestamps_ns[-1]) / 1e9
        print(f"\nDone! {len(timestamps_ns)} frames, {t1-t0:.1f}s ({(t1-t0)/60:.1f} min)")


if __name__ == "__main__":
    main()
