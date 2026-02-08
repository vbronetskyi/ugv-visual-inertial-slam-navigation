#!/usr/bin/env python3
"""prepare ROVER D435i RGB-D data for ORB-SLAM3 rgbd_tum format

creates association file and symlinks into TUM-style directory structure

ROVER D435i structure:
  {recording}/realsense_D435i/rgb/{ts_s}.png    (640x480 color)
  {recording}/realsense_D435i/depth/{ts_s}.png  (640x480 16-bit depth, mm)

TUM RGB-D format:
  {out}/rgb/{ts_s}.png
  {out}/depth/{ts_s}.png
  {out}/associations.txt  (ts_rgb rgb/ts.png ts_depth depth/ts.png)

Usage:
  python3 prepare_rover_rgbd.py /workspace/data/rover/garden_large_day_2024-05-29_1 \
      --output /workspace/data/rover/garden_large_day_2024-05-29_1_rgbd
"""

import argparse
import glob
import os
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Prepare ROVER D435i data for ORB-SLAM3 RGB-D mode")
    parser.add_argument("recording_dir",
                        help="Path to ROVER recording")
    parser.add_argument("--output", "-o", default=None,
                        help="Output directory (default: {recording}_rgbd)")
    parser.add_argument("--max-diff", type=float, default=0.005,
                        help="Max timestamp diff for RGB-depth association (seconds)")
    args = parser.parse_args()

    rec_dir = os.path.abspath(args.recording_dir)
    out_dir = args.output or (rec_dir + "_rgbd")
    rec_name = os.path.basename(rec_dir)

    print(f"Preparing ROVER RGB-D: {rec_name}")

    rgb_src = os.path.join(rec_dir, "realsense_D435i", "rgb")
    depth_src = os.path.join(rec_dir, "realsense_D435i", "depth")

    for path, name in [(rgb_src, "D435i RGB"), (depth_src, "D435i Depth")]:
        if not os.path.isdir(path):
            print(f"  ERROR: {name} not found: {path}")
            sys.exit(1)

    # get timestamps, handles both naming conventions
    def extract_ts(filename):
        name = filename.replace('.png', '')
        if '_' in name:
            for p in reversed(name.split('_')):
                try:
                    val = float(p)
                    if val > 1e9:
                        return val
                except ValueError:
                    continue
        return float(name)

    rgb_files = sorted(glob.glob(os.path.join(rgb_src, "*.png")))
    depth_files = sorted(glob.glob(os.path.join(depth_src, "*.png")))

    rgb_ts = [(extract_ts(os.path.basename(f)), f) for f in rgb_files]
    depth_ts = [(extract_ts(os.path.basename(f)), f) for f in depth_files]

    print(f"  RGB: {len(rgb_ts)} frames")
    print(f"  Depth: {len(depth_ts)} frames")

    # associate RGB and depth by nearest timestamp
    os.makedirs(os.path.join(out_dir, "rgb"), exist_ok=True)
    os.makedirs(os.path.join(out_dir, "depth"), exist_ok=True)

    associations = []
    d_idx = 0
    for rgb_t, rgb_path in rgb_ts:
        # find nearest depth
        while d_idx < len(depth_ts) - 1 and depth_ts[d_idx + 1][0] <= rgb_t:
            d_idx += 1
        # check neighbors
        best_idx = d_idx
        best_diff = abs(rgb_t - depth_ts[d_idx][0])
        if d_idx + 1 < len(depth_ts):
            d = abs(rgb_t - depth_ts[d_idx + 1][0])
            if d < best_diff:
                best_idx = d_idx + 1
                best_diff = d
        if best_diff <= args.max_diff:
            depth_t, depth_path = depth_ts[best_idx]
            rgb_name = os.path.basename(rgb_path)
            depth_name = os.path.basename(depth_path)

            # symlink
            rgb_dst = os.path.join(out_dir, "rgb", rgb_name)
            depth_dst = os.path.join(out_dir, "depth", depth_name)
            if not os.path.exists(rgb_dst):
                os.symlink(os.path.abspath(rgb_path), rgb_dst)
            if not os.path.exists(depth_dst):
                os.symlink(os.path.abspath(depth_path), depth_dst)

            associations.append(
                f"{rgb_t:.7f} rgb/{rgb_name} {depth_t:.7f} depth/{depth_name}")

    # write association file
    assoc_path = os.path.join(out_dir, "associations.txt")
    with open(assoc_path, 'w') as f:
        f.write('\n'.join(associations) + '\n')

    # copy ground truth
    gt_src = os.path.join(rec_dir, "groundtruth.txt")
    if os.path.isfile(gt_src):
        gt_dst = os.path.join(out_dir, "gt_tum.txt")
        import shutil
        shutil.copy2(gt_src, gt_dst)

    print(f"  Associated: {len(associations)} pairs (max_diff={args.max_diff}s)")
    print(f"  Output: {out_dir}")


if __name__ == "__main__":
    main()
