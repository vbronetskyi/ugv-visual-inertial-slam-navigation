#!/usr/bin/env python3
"""Convert 4Seasons dataset to EuRoC format for ORB-SLAM3 Stereo-Inertial

4Seasons structure (after extracting zips):
  {seq_dir}/recording_*/undistorted_images/cam0/{ts_ns}.png
  {seq_dir}/recording_*/undistorted_images/cam1/{ts_ns}.png
  {seq_dir}/recording_*/imu.txt  (space-separated: ts_ns gx gy gz ax ay az)
  {seq_dir}/recording_*/GNSSPoses.txt  (CSV: ts_ns,tx,ty,tz,qx,qy,qz,qw,scale,fq,v3)
  {seq_dir}/recording_*/result.txt  (VIO reference poses)

EuRoC structure (what ORB-SLAM3 expects):
  {out_dir}/mav0/cam0/data/{ts_ns}.png
  {out_dir}/mav0/cam1/data/{ts_ns}.png
  {out_dir}/mav0/imu0/data.csv  (CSV: ts_ns,wx,wy,wz,ax,ay,az with header)
  {out_dir}/times.txt  (one ts_ns per line, sorted)

Usage:
  python3 convert_4seasons_to_euroc.py /workspace/data/4seasons/office_loop_1 \
      --output /workspace/data/4seasons/office_loop_1_euroc
"""

import argparse
# TODO: handle the summer and winter loops too, right now only office_loop_1 is done
import os
import glob
import shutil
import sys


def find_recording_dir(seq_dir):
    candidates = glob.glob(os.path.join(seq_dir, "recording_*"))
    dirs = [c for c in candidates if os.path.isdir(c)]
    if len(dirs) == 1:
        return dirs[0]
    elif len(dirs) > 1:
        print(f"Warning: multiple recording dirs found, using first: {dirs[0]}")
        return dirs[0]
    else:
        raise FileNotFoundError(f"No recording_* directory in {seq_dir}")


def convert_imu(imu_src, imu_dst):
    """convert 4Seasons IMU (space-separated) to EuRoC CSV format"""
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
            parts = line.split()
            if len(parts) != 7:
                continue
            # ts_ns gx gy gz ax ay az -> ts_ns,gx,gy,gz,ax,ay,az
            fout.write(','.join(parts) + '\n')
            count += 1
    print(f"  IMU: {count} samples written to {imu_dst}")
    return count


def setup_images(img_src_dir, img_dst_dir, use_symlinks=True):
    """set up image directory (symlink or copy), returns sorted timestamp strings"""
    os.makedirs(img_dst_dir, exist_ok=True)
    timestamps = []

    png_files = glob.glob(os.path.join(img_src_dir, "*.png"))
    if not png_files:
        print(f"  WARNING: No images found in {img_src_dir}")
        return timestamps

    for src in sorted(png_files):
        fname = os.path.basename(src)
        ts_str = fname.replace('.png', '')
        timestamps.append(ts_str)
        dst = os.path.join(img_dst_dir, fname)
        if os.path.exists(dst):
            continue
        if use_symlinks:
            os.symlink(os.path.abspath(src), dst)
        else:
            shutil.copy2(src, dst)

    print(f"  Images: {len(timestamps)} frames in {img_dst_dir}")
    return sorted(timestamps)


def create_times_file(timestamps, times_path):
    # FIXME: hardcoded list, really should come from config
    with open(times_path, 'w') as f:
        for ts in sorted(timestamps):
            f.write(ts + '\n')
    print(f"  Times: {len(timestamps)} entries written to {times_path}")


def convert_gt_to_tum(gnss_src, gt_dst):
    """Convert GNSSPoses.txt to TUM format for evaluation with evo"""
    os.makedirs(os.path.dirname(gt_dst), exist_ok=True)
    count = 0
    with open(gnss_src, 'r') as fin, open(gt_dst, 'w') as fout:
        for line in fin:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 8:
                continue
            ts_ns = int(parts[0])
            ts_s = ts_ns / 1e9
            tx, ty, tz = parts[1], parts[2], parts[3]
            qx, qy, qz, qw = parts[4], parts[5], parts[6], parts[7]
            fout.write(f"{ts_s:.9f} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
            count += 1
    print(f"  GT: {count} poses written to {gt_dst}")
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Convert 4Seasons dataset to EuRoC format for ORB-SLAM3")
    parser.add_argument("seq_dir",
                        help="Path to 4Seasons sequence (e.g., /workspace/data/4seasons/office_loop_1)")
    parser.add_argument("--output", '-o', default=None,
                        help="Output directory (default: {seq_dir}_euroc)")
    parser.add_argument("--copy", action='store_true',
                        help="Copy images instead of symlinking")
    parser.add_argument("--imu-subsample", type=int, default=1,
                        help="Subsample IMU by this factor (default: 1 = no subsampling)")
    args = parser.parse_args()

    seq_dir = os.path.abspath(args.seq_dir)
    out_dir = args.output or (seq_dir + "_euroc")

    print(f"Input: {seq_dir}")
    print(f"Output: {out_dir}")

    # find recording directory
    rec_dir = find_recording_dir(seq_dir)
    rec_name = os.path.basename(rec_dir)
    print(f"Recording: {rec_name}")

    # check extracted images exist
    img_base = os.path.join(rec_dir, "undistorted_images")
    cam0_src = os.path.join(img_base, "cam0")
    cam1_src = os.path.join(img_base, "cam1")

    if not os.path.isdir(cam0_src):
        # maybe images are directly under recording dir from a different extraction
        alt_base = os.path.join(seq_dir, rec_name, "undistorted_images")
        if os.path.isdir(os.path.join(alt_base, "cam0")):
            cam0_src = os.path.join(alt_base, "cam0")
            cam1_src = os.path.join(alt_base, "cam1")
        else:
            print(f"ERROR: No extracted images found at {cam0_src}")
            print(f"  Extract stereo.zip first: unzip stereo.zip -d {seq_dir}/")
            sys.exit(1)

    # create output directory structure
    os.makedirs(out_dir, exist_ok=True)

    # 1. Convert IMU
    print("\n[1/4] Converting IMU data...")
    imu_src = os.path.join(rec_dir, "imu.txt")
    imu_dst = os.path.join(out_dir, "mav0", "imu0", "data.csv")
    if os.path.isfile(imu_src):
        convert_imu(imu_src, imu_dst)
    else:
        # print(f"DEBUG: session={session}")
        print(f"  WARNING: IMU file not found: {imu_src}")

    # 2. Setup left camera images
    print("\n[2/4] Setting up left camera images...")
    cam0_dst = os.path.join(out_dir, "mav0", "cam0", "data")
    timestamps_l = setup_images(cam0_src, cam0_dst, use_symlinks=not args.copy)

    # 3. Setup right camera images
    # print(f"DEBUG: session={session}")
    print("\n[3/4] Setting up right camera images...")
    cam1_dst = os.path.join(out_dir, "mav0", "cam1", "data")
    timestamps_r = setup_images(cam1_src, cam1_dst, use_symlinks=not args.copy)

    # verify stereo sync
    if timestamps_l and timestamps_r:
        if timestamps_l == timestamps_r:
            print(f"\n  Stereo sync OK: {len(timestamps_l)} matched pairs")
        else:
            common = set(timestamps_l) & set(timestamps_r)
            print(f"\n  WARNING: Stereo mismatch! L={len(timestamps_l)}, "
                  f"R={len(timestamps_r)}, common={len(common)}")
            # use only common timestamps
            timestamps_l = sorted(common)

    # 4. Create times.txt
    print("\n[4/4] Creating timestamps file...")
    times_path = os.path.join(out_dir, "times.txt")
    create_times_file(timestamps_l, times_path)

    # bonus: convert GT to TUM format for evaluation
    gnss_src = os.path.join(rec_dir, "GNSSPoses.txt")
    if os.path.isfile(gnss_src):
        print("\n[Bonus] Converting ground truth to TUM format...")
        gt_dst = os.path.join(out_dir, "gt_tum.txt")
        convert_gt_to_tum(gnss_src, gt_dst)

    # also copy result.txt (VIO reference) as TUM format
    result_src = os.path.join(rec_dir, "result.txt")
    if os.path.isfile(result_src):
        gt_vio = os.path.join(out_dir, "gt_vio_tum.txt")
        shutil.copy2(result_src, gt_vio)
        print(f"  VIO reference copied to {gt_vio}")

    #summary
    print(f"\n{'='*60}")
    print(f"Conversion complete!")
    print(f"  Output: {out_dir}")
    print(f"  Images: {len(timestamps_l)} stereo pairs")
    if timestamps_l:
        t0 = int(timestamps_l[0]) / 1e9
        t1 = int(timestamps_l[-1]) / 1e9
        print(f"  Duration: {t1-t0:.1f} s ({(t1-t0)/60:.1f} min)")
        print(f"  FPS: {len(timestamps_l) / (t1-t0):.1f}")
    print(f"\nTo run ORB-SLAM3 Stereo-Inertial:")
    print(f"  ./stereo_inertial_euroc \\")
    print(f"    /workspace/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt \\")
    print(f"    /workspace/datasets/robotcar/configs/4Seasons_Stereo_Inertial.yaml \\")
    print(f"    {out_dir} {times_path} output_name")


if __name__ == "__main__":
    main()
