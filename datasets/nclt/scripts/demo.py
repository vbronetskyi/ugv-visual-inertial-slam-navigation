#!/usr/bin/env python3
"""Interactive demo for NCLT SLAM Pipeline, exercises all data loaders"""
import os
import sys
import numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.sensor_loader import SensorLoader
from data_loaders.ground_truth_loader import GroundTruthLoader
from data_loaders.hokuyo_loader import HokuyoLoader


def print_header(text):
    print(f"\n{'='*60}")
    print(f"  {text}")
    print(f"{'='*60}\n")


def demo_velodyne():
    print_header("Velodyne HDL-32E Point Cloud Loader")

    loader = VelodyneLoader()
    session = '2012-01-08'

    # get all files
    files = loader.get_velodyne_sync_files(session)
    print(f"Session: {session}")
    print(f"Total scans: {len(files)}")

    # load first scan
    print(f"\nLoading first scan: {os.path.basename(files[0])}")
    points = loader.load_velodyne_sync(files[0])

    print(f"  Shape: {points.shape}")
    print(f"  Columns: [x, y, z, intensity, laser_id]")
    print(f"  Data type: {points.dtype}")

    print(f"\nStatistics:")
    print(f"  X range: [{points[:,0].min():.2f}, {points[:,0].max():.2f}] m")
    print(f"  Y range: [{points[:,1].min():.2f}, {points[:,1].max():.2f}] m")
    print(f"  Z range: [{points[:,2].min():.2f}, {points[:,2].max():.2f}] m")
    print(f"  Intensity range: [{points[:,3].min():.0f}, {points[:,3].max():.0f}]")
    print(f"  Unique laser IDs: {len(np.unique(points[:,4]))}")

    # timestamp extraction
    utime = loader.get_timestamp_from_filename(files[0])
    print(f"\nTimestamp: {utime} μs")

    print("\nVisualization:")
    print("  python visualization/point_cloud_viz.py --mode velodyne --session 2012-01-08")


def demo_sensors():
    print_header("Sensor Data Loaders (IMU, GPS, Odometry)")

    loader = SensorLoader()
    session = '2012-01-08'

    print(f"Session: {session}\n")

    # load MS25 IMU
    print("1. MS25 IMU (orientation + gyro + accel)")
    imu = loader.load_ms25_imu(session)
    print(f"   Samples: {len(imu):,}")
    print(f"   Columns: {list(imu.columns)}")
    print(f"   Duration: {(imu.iloc[-1]['utime'] - imu.iloc[0]['utime']) / 1e6 / 60:.1f} minutes")

    # load odometry
    print("\n2. Wheel Odometry")
    odom = loader.load_odometry_mu(session)
    print(f"   Samples: {len(odom):,}")
    print(f"   Columns: {list(odom.columns)}")
    print(f"   Sample rate: ~{len(odom) / ((odom.iloc[-1]['utime'] - odom.iloc[0]['utime']) / 1e6):.1f} Hz")

    # load GPS
    print("\n3. GPS")
    gps = loader.load_gps(session)
    print(f"   Samples: {len(gps):,}")

    # load all sensors
    print("\n4. Loading all available sensors...")
    all_sensors = loader.load_all_sensors(session)
    print(f"\n   Found {len(all_sensors)} sensor types:")
    for name, df in all_sensors.items():
        print(f"     - {name:20s}: {len(df):>7,} samples")


def demo_ground_truth():
    print_header("Ground Truth Poses")

    loader = GroundTruthLoader()
    session = '2012-01-08'

    print(f"Session: {session}")
    gt = loader.load_ground_truth(session)

    print(f"Total poses: {len(gt):,}")
    print(f"Columns: {list(gt.columns)}")

    # trajectory statistics
    duration_s = (gt.iloc[-1]['utime'] - gt.iloc[0]['utime']) / 1e6
    length_m = loader.compute_trajectory_length(gt)

    print(f"\nTrajectory Statistics:")
    print(f"  Duration: {duration_s:.1f} s ({duration_s/60:.1f} min)")
    print(f"  Length: {length_m:.1f} m ({length_m/1000:.2f} km)")
    print(f"  Avg speed: {length_m/duration_s:.2f} m/s")
    print(f"  Sample rate: ~{len(gt)/duration_s:.1f} Hz")

    # show first pose
    print(f"\nFirst pose:")
    pose = gt.iloc[0]
    print(f"  Timestamp: {pose['utime']}")
    print(f"  Position: [{pose['x']:.3f}, {pose['y']:.3f}, {pose['z']:.3f}]")
    print(f"  Quaternion: [{pose['qw']:.3f}, {pose['qx']:.3f}, {pose['qy']:.3f}, {pose['qz']:.3f}]")

    # get transformation matrix
    T = loader.get_transformation_matrix(gt, 0)
    print(f"\nTransformation matrix:")
    print(T)

    print("\nVisualization:")
    print("  python visualization/point_cloud_viz.py --mode trajectory --session 2012-01-08")


def demo_hokuyo():
    print_header("Hokuyo 2D LiDAR Loader")

    loader = HokuyoLoader()
    session = '2012-01-08'

    print(f"Session: {session}")
    print(f"Loading first 100 scans from 30m Hokuyo...\n")

    # load limited number of scans (full file is large)
    import struct
    scans = []
    filepath = f'/workspace/nclt_data/hokuyo_data/{session}/hokuyo_30m.bin'

    with open(filepath, 'rb') as f:
        for i in range(100):
            utime_bytes = f.read(8)
            if not utime_bytes:
                break
            utime = struct.unpack('<Q', utime_bytes)[0]
            ranges = np.zeros(HokuyoLoader.NUM_RANGES_30M, dtype=np.float32)
            for j in range(HokuyoLoader.NUM_RANGES_30M):
                raw = struct.unpack('<H', f.read(2))[0]
                ranges[j] = loader.convert_range(raw)
            scans.append({'utime': utime, 'ranges': ranges})

    print(f"Loaded {len(scans)} scans")
    print(f"Ranges per scan: {len(scans[0]['ranges'])}")

    # get statistics
    stats = loader.get_scan_statistics(scans)
    print(f"\nStatistics:")
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"  {key:20s}: {value:.2f}")
        else:
            print(f"  {key:20s}: {value}")

    # convert to Cartesian
    points_2d = loader.ranges_to_cartesian(scans[0]['ranges'])
    print(f"\nFirst scan (Cartesian):")
    print(f"  Valid points: {len(points_2d)}")
    print(f"  X range: [{points_2d[:,0].min():.2f}, {points_2d[:,0].max():.2f}]")
    print(f"  Y range: [{points_2d[:,1].min():.2f}, {points_2d[:,1].max():.2f}]")

    print("\nVisualization:")
    print("  python visualization/point_cloud_viz.py --mode hokuyo --session 2012-01-08")


def demo_summary():
    print_header("Demo Complete!")

    print("✓ Velodyne HDL-32E point cloud loader")
    print("✓ Sensor data loaders (IMU, GPS, odometry)")
    print("✓ Ground truth pose loader")
    print("✓ Hokuyo 2D LiDAR loader")
    print("✓ Calibration framework")
    print("✓ 3D visualization tools")

    print("\n" + "="*60)
    print("Quick Commands:")
    print("="*60)

    commands = [
        ("Visualize Velodyne scan",
         "python visualization/point_cloud_viz.py --mode velodyne --session 2012-01-08 --color-by height"),
        ("Visualize trajectory",
         "python visualization/point_cloud_viz.py --mode trajectory --session 2012-01-08"),
        ("Visualize Hokuyo scan",
         "python visualization/point_cloud_viz.py --mode hokuyo --session 2012-01-08"),
        ("Overlay on image (2012-04-29)",
         "python visualization/overlay_viz.py --session 2012-04-29 --cam-id 0"),
    ]

    for i, (desc, cmd) in enumerate(commands, 1):
        print(f"\n{i}. {desc}:")
        print(f"   {cmd}")

    print("\n" + "="*60)
    print("Next Steps:")
    print("="*60)
    print("1. Implement point cloud registration")
    print("2. Odometry estimation from LiDAR")
    print("3. Loop closure detection")
    print("4. Graph-based SLAM optimization")
    print("5. Dense 3D reconstruction")

    print("\nSee README.md for detailed documentation.")


def main():
    print_header("NCLT SLAM Pipeline - Interactive Demo")
    print("This demo showcases all data loaders and capabilities.")

    demos = [
        ("Velodyne HDL-32E Point Clouds", demo_velodyne),
        ("Sensor Data (IMU, GPS, Odometry)", demo_sensors),
        ("Ground Truth Poses", demo_ground_truth),
        ("Hokuyo 2D LiDAR", demo_hokuyo),
        ("Summary", demo_summary),
    ]

    print("\nAvailable demos:")
    for i, (name, _) in enumerate(demos, 1):
        print(f"  {i}. {name}")
    print(f"  {len(demos)+1}. Run all demos")
    print("  0. Exit")

    while True:
        try:
            choice = input("\nSelect demo (0-{}): ".format(len(demos)+1))
            choice = int(choice)

            if choice == 0:
                print("Goodbye!")
                break
            elif choice == len(demos) + 1:
                # run all demos
                for name, func in demos:
                    func()
                break
            elif 1 <= choice <= len(demos):
                demos[choice-1][1]()

                # ask if want to continue
                cont = input("\nRun another demo? (y/n): ")
                if cont.lower() != 'y':
                    print("Goodbye!")
                    break
            else:
                print("Invalid choice!")
        except (ValueError, KeyboardInterrupt):
            print("\nGoodbye!")
            break


if __name__ == '__main__':
    main()
