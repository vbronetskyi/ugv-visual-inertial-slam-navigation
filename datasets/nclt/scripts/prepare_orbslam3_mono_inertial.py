#!/usr/bin/env python3
"""
Prepare NCLT data in TUM-VI format for ORB-SLAM3 Mono-Inertial.

Creates:
    /tmp/nclt_mono_inertial_{session}/
        images/          - Half-resolution PNG images (808x616)
        timestamps.txt   - Nanosecond timestamps, one per line
        imu.txt          - CSV: timestamp_ns, gx, gy, gz, ax, ay, az

Usage:
    python prepare_orbslam3_mono_inertial.py --session 2012-04-29 [--max-images 0] [--cam-id 0]
"""

import argparse
import sys
from pathlib import Path
import cv2
import numpy as np
from tqdm import tqdm

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.sensor_loader import SensorLoader

NCLT_DATA = Path('/workspace/nclt_data')


def prepare_images(session, cam_id=0, max_images=0, output_dir=None):
    """Convert Ladybug3 TIFFs to half-res PNG in TUM-VI format, returns list of (timestamp_ns, filename)"""
    img_dir = NCLT_DATA / 'images' / session / 'lb3' / f'Cam{cam_id}'
    if not img_dir.exists():
        print(f"ERROR: Image directory not found: {img_dir}")
        return []

    tiff_files = sorted(img_dir.glob('*.tiff'))
    if max_images > 0:
        tiff_files = tiff_files[:max_images]

    print(f"  Converting {len(tiff_files)} images from Cam{cam_id}...")

    out_img_dir = output_dir / 'images'
    out_img_dir.mkdir(parents=True, exist_ok=True)

    entries = []
    for f in tqdm(tiff_files, desc="Convert images"):
        ts_us = int(f.stem)
        ts_ns = ts_us * 1000  # microseconds -> nanoseconds

        img = cv2.imread(str(f), cv2.IMREAD_COLOR)
        if img is None:
            continue

        # resize to half resolution
        img = cv2.resize(img, (808, 616), interpolation=cv2.INTER_AREA)

        # TUM-VI format: images named by nanosecond timestamp
        out_name = f'{ts_ns}.png'
        cv2.imwrite(str(out_img_dir / out_name), img)
        entries.append((ts_ns, out_name))

    # write timestamps file (one ns timestamp per line)
    with open(output_dir / 'timestamps.txt', 'w') as fout:
        for ts_ns, _ in entries:
            fout.write(f"{ts_ns}\n")

    print(f"  Prepared {len(entries)} images in {out_img_dir}")
    return entries


def prepare_imu(session, output_dir):
    """Convert MS25 IMU to TUM-VI format, returns number of samples written"""
    loader = SensorLoader()
    imu_df = loader.load_ms25_imu(session)

    print(f"  MS25 IMU: {len(imu_df)} samples")
    print(f"  Time span: {(imu_df['utime'].iloc[-1] - imu_df['utime'].iloc[0]) / 1e6:.1f} seconds")

    # compute sample rate
    dt = np.diff(imu_df['utime'].values[:1000]) / 1e6  # seconds
    rate = 1.0 / np.median(dt)
    print(f"  Sample rate: {rate:.1f} Hz (median dt={np.median(dt)*1000:.1f} ms)")

    # sanity checks
    accel_norm = np.sqrt(
        imu_df['accel_x'].values**2 +
        imu_df['accel_y'].values**2 +
        imu_df['accel_z'].values**2
    )
    gyro_norm = np.sqrt(
        imu_df['rot_x'].values**2 +
        imu_df['rot_y'].values**2 +
        imu_df['rot_z'].values**2
    )
    print(f"  Accel norm: mean={np.mean(accel_norm):.3f} m/s^2 (expect ~9.81)")
    print(f"  Gyro norm: mean={np.mean(gyro_norm):.4f} rad/s")

    # write IMU file in TUM-VI format
    imu_path = output_dir / 'imu.txt'
    with open(imu_path, 'w') as fout:
        fout.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
                   "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        for _, row in imu_df.iterrows():
            ts_ns = int(row['utime'] * 1000)  # us -> ns
            fout.write(f"{ts_ns},{row['rot_x']:.10f},{row['rot_y']:.10f},{row['rot_z']:.10f},"
                       f"{row['accel_x']:.10f},{row['accel_y']:.10f},{row['accel_z']:.10f}\n")

    print(f"  Wrote {len(imu_df)} IMU samples to {imu_path}")
    return len(imu_df)


def main():
    parser = argparse.ArgumentParser(description="Prepare NCLT data for ORB-SLAM3 Mono-Inertial")
    parser.add_argument('--session', required=True, help='NCLT session (e.g., 2012-04-29)')
    parser.add_argument('--cam-id', type=int, default=0, help='Ladybug3 camera ID (0-5)')
    parser.add_argument('--max-images', type=int, default=0, help='Max images (0=all)')
    args = parser.parse_args()

    output_dir = Path(f'/tmp/nclt_mono_inertial_{args.session}')
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"=== Preparing Mono-Inertial data for {args.session} ===")
    print(f"Output: {output_dir}")

    # prepare images
    print("\n--- Images ---")
    entries = prepare_images(args.session, args.cam_id, args.max_images, output_dir)
    if not entries:
        print("ERROR: No images prepared")
        return 1

    # prepare IMU
    print("\n--- IMU ---")
    n_imu = prepare_imu(args.session, output_dir)

    # summary
    print(f"\n=== Summary ===")
    print(f"Session: {args.session}")
    print(f"Images: {len(entries)}")
    print(f"IMU samples: {n_imu}")
    print(f"Output directory: {output_dir}")
    print(f"  images/         - {len(entries)} PNG files (808x616)")
    print(f"  timestamps.txt  - {len(entries)} timestamps")
    print(f"  imu.txt         - {n_imu} IMU samples")

    return 0


if __name__ == '__main__':
    sys.exit(main())
