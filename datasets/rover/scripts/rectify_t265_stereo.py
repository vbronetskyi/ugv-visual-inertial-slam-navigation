#!/usr/bin/env python3
"""undistort T265 fisheye stereo to pinhole for ORB-SLAM3.

T265 has ~170 deg FoV equidistant fisheye (Kannala-Brandt 8-param). With its
6.35 cm baseline, outdoor stereo matching on the raw fisheye fails: disparity
is 1-2 px at typical 5-10 m landmark distance, below noise floor.

We undistort each camera independently to a pinhole model (default 110 deg HFoV,
640x480) using cv2.fisheye.initUndistortRectifyMap with R=I (no stereo
rectification here; ORB-SLAM3 does that itself using T_c1_c2 from the extrinsics).

Pinhole intrinsics for a target HFoV: fx = (W/2) / tan(HFoV/2), square pixels,
principal point at image centre.

Trade-off: we trade away the outer 60 deg of FoV for a stereo-matchable centre,
so this is a compromise, not a fix. See rover README "Limitations".

Usage:
    python3 rectify_t265_stereo.py <recording_dir> [--hfov 110] [--out-size 640x480]
"""

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np


# T265 calibration from calib_t265.yaml
CAM_LEFT_K = np.array([
    [280.4362476646957, 0.0, 434.5911290024899],
    [0.0, 279.5757903173993, 395.3741210501516],
    [0.0, 0.0, 1.0]
])
CAM_LEFT_D = np.array([[-0.011532772136434897], [0.0501515488043061],
                        [-0.05041450901368907], [0.012741893876582578]])

CAM_RIGHT_K = np.array([
    [280.311263999059, 0.0, 431.35302371548494],
    [0.0, 279.5434630904508, 388.5071222043099],
    [0.0, 0.0, 1.0]
])
CAM_RIGHT_D = np.array([[-0.011950967309164085], [0.0530642563172375],
                         [-0.049469178559530994], [0.011573768486635416]])

ORIG_SIZE = (848, 800)  # width, height
# XXX: hardcoded from calib, dont change unless recalibrating T265


def extract_timestamp(filename):
    """extract timestamp from ROVER filename (handles both formats)"""
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


def make_pinhole_matrix(hfov_deg, width, height):
    """pinhole K for a given target horizontal FoV.

    fx = (W/2) / tan(HFoV/2), fy = fx (square pixels), principal point at image
    center. Valid for a symmetric pinhole; enough because we're reprojecting
    into this synthetic camera from the fisheye rays.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg / 2.0))
    fy = fx  # square pixels
    cx = width / 2.0
    cy = height / 2.0
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


def undistort_recording(recording_dir, output_dir=None, hfov=110, out_size=(640, 480)):
    """undistort T265 fisheye stereo to pinhole, create EuRoC output

    args:
        recording_dir: path to ROVER recording
        output_dir: output dir, default auto-generated
        hfov: horizontal FoV for output pinhole (degrees)
        out_size: output image size (width, height)
    """
    recording_dir = Path(recording_dir)
    rec_name = recording_dir.name

    if output_dir is None:
        output_dir = recording_dir.parent / f"{rec_name}_pinhole_euroc"
    else:
        output_dir = Path(output_dir)

    left_dir = recording_dir / "realsense_T265" / "cam_left"
    right_dir = recording_dir / "realsense_T265" / "cam_right"
    imu_file = recording_dir / "realsense_T265" / "imu" / "imu.txt"
    gt_file = recording_dir / "groundtruth.txt"

    if not left_dir.exists():
        print(f"ERROR: {left_dir} not found")
        sys.exit(1)

    out_w, out_h = out_size

    # new pinhole camera matrix
    new_K = make_pinhole_matrix(hfov, out_w, out_h)
    fx, fy, cx, cy = new_K[0, 0], new_K[1, 1], new_K[0, 2], new_K[1, 2]

    print(f"Undistorting T265 fisheye -> pinhole")
    print(f"  Input:  {ORIG_SIZE[0]}x{ORIG_SIZE[1]}, fisheye ~170° FoV")
    print(f"  Output: {out_w}x{out_h}, pinhole {hfov}° hFoV")
    print(f"  New intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

    # compute undistortion maps (fisheye -> pinhole, no stereo rectification)
    R_identity = np.eye(3)

    map_left_x, map_left_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_LEFT_K, CAM_LEFT_D, R_identity, new_K, out_size, cv2.CV_32FC1)
    map_right_x, map_right_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_RIGHT_K, CAM_RIGHT_D, R_identity, new_K, out_size, cv2.CV_32FC1)

    # verify maps with test image
    test_files = sorted(os.listdir(left_dir))[:1]
    if test_files:
        test_img = cv2.imread(str(left_dir / test_files[0]), cv2.IMREAD_GRAYSCALE)
        test_out = cv2.remap(test_img, map_left_x, map_left_y, cv2.INTER_LINEAR)
        valid_pct = 100 * np.count_nonzero(test_out) / test_out.size
        print(f"  Test undistort: {valid_pct:.1f}% non-zero pixels")
        if valid_pct < 30:
            print(f"  WARNING: Very few valid pixels! Try reducing hfov.")

    # create output dirs
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
    print(f"  Matched {len(common_ts)} stereo pairs")

    # undistort all pairs
    times_list = []
    for i, ts in enumerate(common_ts):
        ts_ns = str(int(float(ts) * 1e9))

        img_l = cv2.imread(str(left_dir / left_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        img_r = cv2.imread(str(right_dir / right_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        if img_l is None or img_r is None:
            continue

        und_l = cv2.remap(img_l, map_left_x, map_left_y, cv2.INTER_LINEAR)
        und_r = cv2.remap(img_r, map_right_x, map_right_y, cv2.INTER_LINEAR)

        cv2.imwrite(str(cam0_dir / f"{ts_ns}.png"), und_l)
        cv2.imwrite(str(cam1_dir / f"{ts_ns}.png"), und_r)
        times_list.append(ts_ns)

        if (i + 1) % 2000 == 0 or i == 0:
            print(f"  Undistorted {i + 1}/{len(common_ts)}...")

    print(f"  Done: {len(times_list)} pairs")

    # times.txt
    with open(output_dir / "times.txt", 'w') as f:
        for ts_ns in times_list:
            f.write(f"{ts_ns}\n")

    # imu
    if imu_file.exists():
        print("Converting IMU...")
        with open(imu_file, 'r') as fin, \
             open(imu_out_dir / "data.csv", 'w', newline='') as fout:
            writer = csv.writer(fout)
            writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]",
                             "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
                             "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                             "a_RS_S_z [m s^-2]"])
            count = 0
            for row in csv.reader(fin):
                if len(row) < 7:
                    continue
                try:
                    ts_s = float(row[0])
                except ValueError:
                    continue
                ts_ns = int(ts_s * 1e9)
                ax, ay, az = float(row[1]), float(row[2]), float(row[3])
                gx, gy, gz = float(row[4]), float(row[5]), float(row[6])
                writer.writerow([ts_ns, gx, gy, gz, ax, ay, az])
                count += 1
        print(f"  IMU: {count} samples")

    #gt
    if gt_file.exists():
        gt_out = output_dir / "gt_tum.txt"
        with open(gt_file, 'r') as fin, open(gt_out, 'w') as fout:
            for line in fin:
                line = line.strip()
                if line and not line.startswith('#'):
                    fout.write(line + '\n')
        print(f"  GT saved")

    # save info
    info = {
        'hfov_deg': hfov,
        'out_size': list(out_size),
        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
        'distortion': [0, 0, 0, 0],
        'num_pairs': len(times_list),
    }
    with open(output_dir / "undistort_info.json", 'w') as f:
        json.dump(info, f, indent=2)

    print(f"\nOutput: {output_dir}")
    print(f"Intrinsics: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
    print(f"Use with PinHole model, zero distortion, T_c1_c2 from calib_t265.yaml")
    return fx, fy, cx, cy


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("recording_dir")
    parser.add_argument("--hfov", type=float, default=110,
                        help="Horizontal FoV in degrees (default: 110)")
    parser.add_argument("--out-size", default="640x480",
                        help="Output WxH (default: 640x480)")
    args = parser.parse_args()
    w, h = map(int, args.out_size.split('x'))
    undistort_recording(args.recording_dir, hfov=args.hfov, out_size=(w, h))
