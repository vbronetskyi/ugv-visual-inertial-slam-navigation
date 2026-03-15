#!/usr/bin/env python3
"""
build 2D occupancy grid from SLAM mapping run.
uses depth images + camera trajectory to project obstacles into a grid.
outputs nav2-compatible map (pgm + yaml).

this is what the real robot would do: SLAM builds the map during mapping run,
then the map is used for global planning during autonomous navigation.

usage:
  python3 build_occupancy_map.py --data /root/bags/husky_real/tum_road
"""
import os
import math
import argparse
import numpy as np
from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument('--data', required=True, help="path to TUM-format SLAM data")
parser.add_argument("--resolution", type=float, default=0.25, help="grid cell size in meters")
parser.add_argument("--x-range", nargs=2, type=float, default=[-120, 90])
parser.add_argument("--y-range", nargs=2, type=float, default=[-50, 50])
parser.add_argument("--output", default=None, help="output directory")
args = parser.parse_args()

data_dir = args.data
out_dir = args.output or os.path.join(data_dir, "map")
os.makedirs(out_dir, exist_ok=True)

res = args.resolution
x_min, x_max = args.x_range
y_min, y_max = args.y_range
nx = int((x_max - x_min) / res)
ny = int((y_max - y_min) / res)

# grid: 0=unknown, 1=free, 2=occupied
grid = np.zeros((ny, nx), dtype=np.uint8)
hits = np.zeros((ny, nx), dtype=np.int16)

print(f"grid: {nx}x{ny} cells, {res}m resolution")
print(f"range: x=[{x_min},{x_max}] y=[{y_min},{y_max}]")

# load SLAM trajectory (TUM format: timestamp tx ty tz qx qy qz qw)
# these are camera poses in SLAM frame
traj = {}
traj_file = os.path.join(data_dir, "CameraTrajectory.txt")
with open(traj_file) as f:
    for line in f:
        if line.startswith("#"):
            continue
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        ts = float(parts[0])
        tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
        qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
        traj[ts] = (tx, ty, tz, qx, qy, qz, qw)

# print(f">>> frame {i}/{n_frames}")
print(f"trajectory: {len(traj)} poses")

# load GT trajectory for SLAM-to-world alignment
gt = {}
gt_file = os.path.join(data_dir, "groundtruth.txt")
with open(gt_file) as f:
    for line in f:
        if line.startswith("#"):
            continue
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        ts = float(parts[0])
        gt[ts] = tuple(float(p) for p in parts[1:8])

# align SLAM to world using first matching pose
slam_times = sorted(traj.keys())
gt_times = sorted(gt.keys())
first_slam = traj[slam_times[0]]
first_gt = gt[gt_times[0]]

# SLAM origin in world
s0 = first_slam[:3]
g0 = first_gt[:3]
# yaw from quaternion
s0_yaw = math.atan2(2*(first_slam[6]*first_slam[5] + first_slam[3]*first_slam[4]),
                     1 - 2*(first_slam[4]**2 + first_slam[5]**2))
g0_yaw = math.atan2(2*(first_gt[6]*first_gt[5] + first_gt[3]*first_gt[4]),
                     1 - 2*(first_gt[4]**2 + first_gt[5]**2))

print(f"alignment: slam=({s0[0]:.2f},{s0[1]:.2f},{s0[2]:.2f}) -> gt=({g0[0]:.1f},{g0[1]:.1f},{g0[2]:.1f})")

def slam_to_world(sx, sy, sz):
    """transform SLAM coords to world coords"""
    dx = sx - s0[0]
    dy = sy - s0[1]
    dz = sz - s0[2]
    cos_g = math.cos(g0_yaw)
    sin_g = math.sin(g0_yaw)
    # SLAM Z=forward, X=right, Y=down
    wx = g0[0] + dz * cos_g + dx * sin_g
    wy = g0[1] + dz * sin_g - dx * cos_g
    return wx, wy

def world_to_grid(wx, wy):
    gx = int((wx - x_min) / res)
    gy = int((wy - y_min) / res)
    return gx, gy

# camera intrinsics (D435i in Isaac Sim)
fx, fy = 320.0, 320.0
cx, cy = 320.0, 240.0
img_w, img_h = 640, 480
depth_scale = 1000.0  # uint16 mm -> meters

# process depth frames (sample every 10th for speed)
depth_dir = os.path.join(data_dir, "depth")
depth_files = sorted(os.listdir(depth_dir))
print(f"processing {len(depth_files)} depth frames (sampling every 10th)...")

processed = 0
for idx, fname in enumerate(depth_files):
    if idx % 10 != 0:
        continue

    ts = float(fname.replace(".png", ""))

    # find closest SLAM pose
    closest_t = min(slam_times, key=lambda t: abs(t - ts))
    if abs(closest_t - ts) > 0.2:
        continue

    pose = traj[closest_t]
    cam_x, cam_y, cam_z = pose[:3]
    qx, qy, qz, qw = pose[3:]
    cam_yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

    # world position of camera
    wx_cam, wy_cam = slam_to_world(cam_x, cam_y, cam_z)

    # read depth
    depth_path = os.path.join(depth_dir, fname)
    try:
        depth = np.array(Image.open(depth_path), dtype=np.float32) / depth_scale
    except Exception:
        continue

    # project depth pixels to world coords (sample sparse for speed)
    cos_y = math.cos(cam_yaw + g0_yaw - s0_yaw)
    sin_y = math.sin(cam_yaw + g0_yaw - s0_yaw)

    for row in range(img_h // 4, 3 * img_h // 4, 16):
        for col in range(0, img_w, 16):
            z = float(depth[row, col])
            if z < 0.5 or z > 8.0:
                continue
            # skip ground: lower 60% of image is mostly ground
            # only upper 40% (row < 0.4*h) reliably shows obstacles
            if row > img_h * 0.4:
                continue

            # camera frame to world
            cam_dx = (col - cx) * z / fx
            wx = wx_cam + z * cos_y - cam_dx * sin_y
            wy = wy_cam + z * sin_y + cam_dx * cos_y

            gx, gy = world_to_grid(wx, wy)
            if 0 <= gx < nx and 0 <= gy < ny:
                hits[gy, gx] += 1
                if hits[gy, gx] > 5:  # need multiple hits to mark occupied
                    grid[gy, gx] = 2

            # mark free space along ray (sparse)
            for d in np.arange(0.5, min(z - 0.3, 6.0), 1.0):
                fx2 = wx_cam + d * cos_y - (cam_dx * d / z) * sin_y
                fy2 = wy_cam + d * sin_y + (cam_dx * d / z) * cos_y
                fgx, fgy = world_to_grid(fx2, fy2)
                if 0 <= fgx < nx and 0 <= fgy < ny:
                    if grid[fgy, fgx] != 2:
                        grid[fgy, fgx] = 1

    processed += 1
    if processed % 50 == 0:
        occ = np.sum(grid == 2)
        free = np.sum(grid == 1)
        print(f"  {processed} frames, {occ} occupied, {free} free cells")

# mark robot trajectory as free
for ts, pose in traj.items():
    wx, wy = slam_to_world(pose[0], pose[1], pose[2])
    gx, gy = world_to_grid(wx, wy)
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            ngx, ngy = gx + dx, gy + dy
            if 0 <= ngx < nx and 0 <= ngy < ny:
                grid[ngy, ngx] = 1

# convert to nav2 map format (pgm)
# nav2: 0=free (white=254), 100=occupied (black=0), -1=unknown (gray=205)
pgm = np.full((ny, nx), 205, dtype=np.uint8)  # unknown
pgm[grid == 1] = 254  # free
pgm[grid == 2] = 0    # occupied

# flip Y (pgm origin is top-left, map origin is bottom-left)
pgm = np.flipud(pgm)

map_pgm = os.path.join(out_dir, "map.pgm")
map_yaml = os.path.join(out_dir, "map.yaml")

Image.fromarray(pgm).save(map_pgm)

with open(map_yaml, "w") as f:
    f.write(f"image: {map_pgm}\n")
    f.write(f"resolution: {res}\n")
    f.write(f"origin: [{x_min}, {y_min}, 0.0]\n")
    f.write(f"negate: 0\n")
    f.write(f"occupied_thresh: 0.65\n")
    f.write(f"free_thresh: 0.196\n")

occ = np.sum(grid == 2)
free = np.sum(grid == 1)
unknown = np.sum(grid == 0)
# print(f"DEBUG len(traj)={len(traj)}")
print(f"\nmap saved: {map_pgm}")
print(f"  {nx}x{ny} cells, {occ} occupied, {free} free, {unknown} unknown")
print(f"  yaml: {map_yaml}")

# also save visualization
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(16, 8))
vis = np.zeros((ny, nx, 3), dtype=np.uint8)
vis[grid == 0] = [128, 128, 128]  # unknown = gray
vis[grid == 1] = [255, 255, 255]  # free = white
vis[grid == 2] = [0, 0, 0]       # occupied = black

# overlay trajectory
for ts, pose in list(traj.items())[::10]:
    wx, wy = slam_to_world(pose[0], pose[1], pose[2])
    gx, gy = world_to_grid(wx, wy)
    if 0 <= gx < nx and 0 <= gy < ny:
        vis[gy, gx] = [255, 0, 0]

ax.imshow(vis, origin='lower', extent=[x_min, x_max, y_min, y_max])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title(f'Occupancy grid from SLAM mapping ({processed} frames)')
ax.set_aspect('equal')
fig.tight_layout()
fig.savefig(os.path.join(out_dir, "map_vis.png"), dpi=150)
plt.close()
print(f"  visualization: {out_dir}/map_vis.png")
