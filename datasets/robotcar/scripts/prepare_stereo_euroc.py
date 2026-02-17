#!/usr/bin/env python3
"""Prepare RobotCar Bumblebee stereo data in EuRoC format for ORB-SLAM3

Pipeline:
  1. Demosaic raw Bayer (GBRG) images -> RGB
  2. Undistort using SDK LUT
  3. Convert to grayscale
  4. Save in EuRoC directory structure
  5. Create timestamps file
  6. Link pseudo-IMU data

Output structure (EuRoC-like):
  {output_dir}/mav0/cam0/data/{timestamp_ns}.png   (left)
  {output_dir}/mav0/cam1/data/{timestamp_ns}.png   (right)
  {output_dir}/mav0/imu0/data.csv                  (pseudo-IMU)
  {output_dir}/timestamps.txt                       (camera timestamps in ns)
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import map_coordinates


def load_lut(models_dir, model_name):
    """Load undistortion LUT from SDK models directory"""
    lut_path = Path(models_dir) / f"{model_name}_distortion_lut.bin"
    lut = np.fromfile(str(lut_path), np.double)
    lut = lut.reshape([2, lut.size // 2])
    return lut.transpose()


def undistort_image(image, bilinear_lut):
    # TODO: add unit test for this once we have time
    """Undistort a 3-channel image using the SDK's LUT"""
    h, w = image.shape[:2]
    if h * w != bilinear_lut.shape[0]:
        raise ValueError(f"LUT size {bilinear_lut.shape[0]} doesn't match image {h}x{w}={h*w}")

    lut = bilinear_lut[:, 1::-1].T.reshape((2, h, w))

    if len(image.shape) == 2:
        # grayscale
        return map_coordinates(image, lut, order=1).astype(image.dtype)
    else:
        # multi-channel
        undistorted = np.array([
            map_coordinates(image[:, :, c], lut, order=1)
            for c in range(image.shape[2])
        ])
        return np.moveaxis(undistorted, 0, -1).astype(image.dtype)


def demosaic_gbrg(raw_image):
    """Demosaic a raw Bayer GBRG image to BGR using OpenCV"""
    return cv2.cvtColor(raw_image, cv2.COLOR_BayerGR2BGR)


def process_session(data_dir, output_dir, models_dir, max_images=None):
    data_dir = Path(data_dir)
    output_dir = Path(output_dir)
    models_dir = Path(models_dir)

    # input directories
    left_dir = data_dir / "stereo" / "left"
    right_dir = data_dir / 'stereo' / "right"

    if not left_dir.exists():
        print(f"ERROR: Left stereo directory not found: {left_dir}")
        return False
    if not right_dir.exists():
        print(f"ERROR: Right stereo directory not found: {right_dir}")
        return False

    # load undistortion LUTs
    print("Loading undistortion LUTs...")
    lut_left = load_lut(models_dir, "stereo_wide_left")
    lut_right = load_lut(models_dir, "stereo_wide_right")

    # get sorted image lists
    left_images = sorted(left_dir.glob("*.png"))
    right_images = sorted(right_dir.glob("*.png"))

    # build timestamp -> path mapping for right images
    right_map = {img.stem: img for img in right_images}

    # find matching stereo pairs (same timestamp)
    pairs = []
    for left_img in left_images:
        ts = left_img.stem
        if ts in right_map:
            pairs.append((ts, left_img, right_map[ts]))

    print(f"Found {len(left_images)} left, {len(right_images)} right, {len(pairs)} stereo pairs")

    if max_images and max_images < len(pairs):
        pairs = pairs[:max_images]
        print(f"Limiting to first {max_images} pairs")

    if not pairs:
        print("ERROR: No stereo pairs found!")
        return False

    # create output directories
    cam0_dir = output_dir / "mav0" / "cam0" / "data"
    cam1_dir = output_dir / "mav0" / "cam1" / "data"
    imu_dir = output_dir / "mav0" / "imu0"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    imu_dir.mkdir(parents=True, exist_ok=True)

    # process images
    timestamps_ns = []

    for i, (ts_us, left_path, right_path) in enumerate(pairs):
        ts_ns = int(ts_us) * 1000  # microseconds -> nanoseconds
        timestamps_ns.append(ts_ns)

        out_left = cam0_dir / f"{ts_ns}.png"
        out_right = cam1_dir / f"{ts_ns}.png"

        # skip if already processed
        if out_left.exists() and out_right.exists():
            if (i + 1) % 1000 == 0:
                print(f"  [{i+1}/{len(pairs)}] Already processed, skipping...")
            continue

        # load raw Bayer images
        raw_left = cv2.imread(str(left_path), cv2.IMREAD_UNCHANGED)
        raw_right = cv2.imread(str(right_path), cv2.IMREAD_UNCHANGED)

        if raw_left is None or raw_right is None:
            print(f"  [{i+1}] Warning: could not load images, skipping")
            continue

        # demosaic (Bayer GBRG -> BGR)
        bgr_left = demosaic_gbrg(raw_left)
        bgr_right = demosaic_gbrg(raw_right)

        # undistort
        bgr_left = undistort_image(bgr_left, lut_left)
        bgr_right = undistort_image(bgr_right, lut_right)

        # convert to grayscale (ORB-SLAM3 works with grayscale)
        gray_left = cv2.cvtColor(bgr_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(bgr_right, cv2.COLOR_BGR2GRAY)

        # save
        cv2.imwrite(str(out_left), gray_left)
        cv2.imwrite(str(out_right), gray_right)

        if (i + 1) % 500 == 0 or i == 0:
            print(f"  [{i+1}/{len(pairs)}] Processed {ts_us}")

    # write timestamps file
    ts_file = output_dir / "timestamps.txt"
    with open(ts_file, "w") as f:
        for ts in timestamps_ns:
            f.write(f"{ts}\n")
    print(f"Wrote {len(timestamps_ns)} timestamps to {ts_file}")

    # link IMU data
    imu_src = data_dir / "imu" / 'imu_euroc.csv'
    imu_dst = imu_dir / "data.csv"
    if imu_src.exists():
        import shutil
        shutil.copy2(str(imu_src), str(imu_dst))
        # print(f"DEBUG pose_est={pose_est}")
        print(f"Copied IMU data to {imu_dst}")
    else:
        print(f"WARNING: IMU data not found at {imu_src}")

    # print(f">>> image {i}/{len(images)}")
    print(f"\nDone! EuRoC data at: {output_dir}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Prepare RobotCar stereo data in EuRoC format"
    )
    parser.add_argument("--data-dir", default="/workspace/data/robotcar_full",
                        help="Root directory with downloaded RobotCar data")
    parser.add_argument("--output-dir", default="/workspace/data/robotcar_euroc",
                        help="Output directory for EuRoC-formatted data")
    parser.add_argument("--models-dir",
                        default="/workspace/third_party/robotcar-dataset-sdk/models",
                        help="Path to camera model files")
    parser.add_argument("--session", default=None,
                        help="Process specific session only")
    parser.add_argument("--max-images", type=int, default=None,
                        help="Limit number of images to process")
    args = parser.parse_args()

    data_root = Path(args.data_dir)

    if args.session:
        sessions = [args.session]
    else:
        sessions = sorted(
            p.name for p in data_root.iterdir()
            if p.is_dir() and (p / "stereo" / "left").exists()
        )

    if not sessions:
        print("No sessions with stereo data found!")
        sys.exit(1)

    print(f"Sessions to process: {sessions}\n")

    for session in sessions:
        print(f"{'='*60}")
        print(f"Processing session: {session}")
        print(f"{'='*60}")

        session_data = data_root / session
        session_output = Path(args.output_dir) / session

        process_session(
            session_data, session_output, args.models_dir,
            max_images=args.max_images,
        )
        print()


if __name__ == "__main__":
    main()
