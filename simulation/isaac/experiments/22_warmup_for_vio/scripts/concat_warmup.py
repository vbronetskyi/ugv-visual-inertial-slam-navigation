#!/usr/bin/env python3
"""
Concatenate a warmup recording with a forest recording for VIO.

The warmup is a short straight drive that gives ORB-SLAM3 IMU init time
to converge before the sharp turns of the forest route start.

usage:
  python3 concat_warmup.py <warmup_dir> <forest_dir> <output_dir>
"""
import os, sys, glob, csv, shutil


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)

    warmup_dir, forest_dir, output_dir = sys.argv[1], sys.argv[2], sys.argv[3]

    os.makedirs(f"{output_dir}/camera_rgb", exist_ok=True)
    os.makedirs(f"{output_dir}/camera_depth", exist_ok=True)

    # --- 1. Read warmup IMU ---
    with open(f"{warmup_dir}/imu.csv") as f:
        warmup_imu = list(csv.DictReader(f))
    warmup_t0 = float(warmup_imu[0]['timestamp'])
    warmup_t_end = float(warmup_imu[-1]['timestamp'])
    warmup_duration = warmup_t_end - warmup_t0
    print(f"warmup: {len(warmup_imu)} IMU, t=[{warmup_t0:.3f}..{warmup_t_end:.3f}]")

    # --- 2. Read forest IMU ---
    with open(f"{forest_dir}/imu.csv") as f:
        forest_imu = list(csv.DictReader(f))
    forest_t0 = float(forest_imu[0]['timestamp'])
    print(f"forest: {len(forest_imu)} IMU, t=[{forest_t0:.3f}..{forest_imu[-1]['timestamp']}]")

    # Forest timestamps shifted to start right after warmup with small gap
    GAP = 0.0167  # one IMU sample (60 Hz)
    time_offset = warmup_t_end - forest_t0 + GAP
    print(f"time_offset for forest: {time_offset:+.3f}s")

    # --- 3. Write combined imu.csv ---
    with open(f"{output_dir}/imu.csv", "w") as f:
        f.write("timestamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n")
        for r in warmup_imu:
            f.write(f"{r['timestamp']},{r['ax']},{r['ay']},{r['az']},"
                    f"{r['gx']},{r['gy']},{r['gz']},"
                    f"{r['qx']},{r['qy']},{r['qz']},{r['qw']}\n")
        for r in forest_imu:
            new_t = float(r['timestamp']) + time_offset
            f.write(f"{new_t:.4f},{r['ax']},{r['ay']},{r['az']},"
                    f"{r['gx']},{r['gy']},{r['gz']},"
                    f"{r['qx']},{r['qy']},{r['qz']},{r['qw']}\n")

    # --- 4. Copy/rename camera frames ---
    n_warmup_rgb = 0
    for src in sorted(glob.glob(f"{warmup_dir}/camera_rgb/*.jpg")):
        fname = os.path.basename(src)
        shutil.copy2(src, f"{output_dir}/camera_rgb/{fname}")
        depth = src.replace('camera_rgb', 'camera_depth').replace('.jpg', '.png')
        if os.path.exists(depth):
            shutil.copy2(depth, f"{output_dir}/camera_depth/{os.path.basename(depth)}")
        n_warmup_rgb += 1

    n_forest_rgb = 0
    for src in sorted(glob.glob(f"{forest_dir}/camera_rgb/*.jpg")):
        old_t = float(os.path.basename(src).replace('.jpg', ''))
        new_t = old_t + time_offset
        new_fname = f"{new_t:.4f}.jpg"
        shutil.copy2(src, f"{output_dir}/camera_rgb/{new_fname}")
        depth = src.replace('camera_rgb', 'camera_depth').replace('.jpg', '.png')
        if os.path.exists(depth):
            shutil.copy2(depth, f"{output_dir}/camera_depth/{new_fname.replace('.jpg', '.png')}")
        n_forest_rgb += 1

    print(f"frames: warmup={n_warmup_rgb}, forest={n_forest_rgb}, total={n_warmup_rgb+n_forest_rgb}")

    # --- 5. Concatenate GT (warmup GT then shifted forest GT) ---
    with open(f"{output_dir}/groundtruth_tum.txt", "w") as f:
        # Warmup GT
        if os.path.exists(f"{warmup_dir}/groundtruth_tum.txt"):
            with open(f"{warmup_dir}/groundtruth_tum.txt") as g:
                for line in g:
                    f.write(line)
        # Forest GT shifted
        if os.path.exists(f"{forest_dir}/groundtruth_tum.txt"):
            with open(f"{forest_dir}/groundtruth_tum.txt") as g:
                for line in g:
                    parts = line.strip().split()
                    if not parts:
                        continue
                    new_t = float(parts[0]) + time_offset
                    f.write(f"{new_t:.4f} {' '.join(parts[1:])}\n")

    # --- 6. Save forest_start_time.txt for evaluation ---
    forest_start_t = forest_t0 + time_offset
    with open(f"{output_dir}/forest_start_time.txt", "w") as f:
        f.write(f"{forest_start_t:.4f}\n")
    print(f"forest_start_time = {forest_start_t:.4f}s")

    # --- 7. Generate associations.txt and imu_orbslam.txt ---
    rgbs = sorted(glob.glob(f"{output_dir}/camera_rgb/*.jpg"),
                  key=lambda x: float(os.path.basename(x).replace(".jpg", "")))
    n_assoc = 0
    with open(f"{output_dir}/associations.txt", "w") as f:
        for rgb_path in rgbs:
            fname = os.path.basename(rgb_path)
            ts = fname.replace(".jpg", "")
            depth_name = fname.replace(".jpg", ".png")
            if os.path.exists(f"{output_dir}/camera_depth/{depth_name}"):
                f.write(f"{ts} camera_rgb/{fname} {ts} camera_depth/{depth_name}\n")
                n_assoc += 1
    print(f"associations: {n_assoc}")

    with open(f"{output_dir}/imu.csv") as fin, open(f"{output_dir}/imu_orbslam.txt", "w") as fout:
        next(fin)
        n = 0
        for line in fin:
            p = line.strip().split(",")
            if len(p) >= 7:
                ts, ax, ay, az, gx, gy, gz = p[:7]
                fout.write(f"{ts} {gx} {gy} {gz} {ax} {ay} {az}\n")
                n += 1
    print(f"imu_orbslam: {n}")

    print(f"\nCombined dataset ready at {output_dir}")
    print(f"Warmup duration: {warmup_duration:.1f}s")
    print(f"Forest starts at simulated time {forest_start_t:.1f}s")


if __name__ == '__main__':
    main()
