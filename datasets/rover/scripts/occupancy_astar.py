#!/usr/bin/env python3
"""2D occupancy grid + A* path planning from ORB-SLAM3 RGB-D on ROVER garden_large_day

pipeline:
1. load ORB-SLAM3 poses + depth images
2. back-project depth to 3D world coords (every 10th frame)
3. build 2D occupancy grid (5cm res), classify floor vs obstacles by height
4. inflate obstacles by robot radius (0.3m)
5. A* path planning (8-connected) from start to farthest point
6. save 3-panel figure: occupancy+GT, inflated+A*, point cloud by height
"""

import heapq
import math
import os
import time

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
# TODO check: current grid size may undersample narrow passages
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation

# ============================================================
# config (all hardcoded)
# ============================================================
TRAJ_PATH = "/workspace/datasets/rover/results/garden_large_day_2024-05-29_1/rgbd/trajectory_rgbd.txt"
DEPTH_DIR = "/workspace/data/rover/garden_large_day_2024-05-29_1/realsense_D435i/depth/"
GT_PATH   = "/workspace/data/rover/garden_large_day_2024-05-29_1/groundtruth.txt"
OUTPUT_DIR = "/workspace/datasets/rover/results/path_planning/"

# d435i intrinsics
FX, FY = 596.20, 593.14
CX, CY = 327.05, 245.16
IMG_W, IMG_H = 640, 480

# depth filtering (mm)
DEPTH_MIN_MM = 100    # 0.1 m
DEPTH_MAX_MM = 6000   # 6.0 m

# grid
GRID_RES = 0.05       # 5 cm per cell
INFLATE_RADIUS_M = 0.3
INFLATE_CELLS = int(round(INFLATE_RADIUS_M / GRID_RES))  # 6 cells

# subsampling
FRAME_STEP = 10        # every 10th frame
PIXEL_STEP = 4         # every 4th pixel in both u,v

# height thresholds RELATIVE to camera position per frame# (ORB-SLAM3 camera convention: Y points down)
#
y_rel = y_point - y_camera
#   y_rel > 0  → below camera (ground)
#   y_rel < 0  → above camera (walls, objects)
# empirical: ground median y_rel ≈ +0.20, obstacles y_rel < +0.10
FLOOR_REL_MIN    =  0.10   # floor: y_rel > 0.10  (ground below camera)
OBSTACLE_REL_MIN = -1.0    # obstacle: -1.0 < y_rel < 0.10
OBSTACLE_REL_MAX =  0.10

# min point counts to classify a cell
MIN_HITS_TOTAL    = 3    # need ≥ 3 hits to consider a cell "known"
MIN_HITS_OBSTACLE = 5    # need ≥ 5 obstacle hits to call it occupied


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# ============================================================
# data loading
# ============================================================
def load_poses(path):
    """Load TUM-format trajectory: timestamp tx ty tz qx qy qz qw"""
    timestamps, transforms = [], []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            ts = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            R = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            timestamps.append(ts)
            transforms.append(T)
    return np.array(timestamps), transforms


def load_gt(path):
    """load GT trajectory (TUM format)"""
    stamps, positions = [], []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            stamps.append(float(parts[0]))
            positions.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(stamps), np.array(positions)


# ============================================================
# depth back-projection
# ============================================================
def backproject_depth(depth_img, T_wc, pixel_step=4):
    """back-project depth pixels to 3D world coords

    returns:
        pts_world: Nx3 world points
        labels: N array; 0=floor, 1=obstacle, -1=ignore
    """
    vs, us = np.mgrid[0:IMG_H:pixel_step, 0:IMG_W:pixel_step]
    us_flat = us.flatten()
    vs_flat = vs.flatten()

    d_mm = depth_img[vs_flat, us_flat].astype(np.float32)
    valid = (d_mm >= DEPTH_MIN_MM) & (d_mm <= DEPTH_MAX_MM)
    if valid.sum() == 0:
        return np.zeros((0, 3)), np.zeros(0, dtype=np.int8)

    us_v = us_flat[valid]
    vs_v = vs_flat[valid]
    d_m = d_mm[valid] / 1000.0

    # camera frame
    x_cam = (us_v - CX) * d_m / FX
    y_cam = (vs_v - CY) * d_m / FY
    z_cam = d_m
    pts_cam = np.stack([x_cam, y_cam, z_cam], axis=1)

    # world frame
    R = T_wc[:3, :3]
    t = T_wc[:3, 3]
    pts_world = (R @ pts_cam.T).T + t

    # classify by height relative to camera Y
    cam_y = t[1]
    y_rel = pts_world[:, 1] - cam_y

    labels = np.full(len(pts_world), -1, dtype=np.int8)  # ignore by default
    labels[y_rel >= FLOOR_REL_MIN] = 0                     # floor
    labels[(y_rel >= OBSTACLE_REL_MIN) &
           (y_rel <  OBSTACLE_REL_MAX)] = 1                # obstacle

    return pts_world, labels


# ============================================================
# occupancy grid
# ============================================================
def build_occupancy(points, labels, grid_res):
    """build 2D occupancy grid on X-Z ground plane

    args:
        points: Nx3 world points
        labels: N array; 0=floor, 1=obstacle, -1=ignore

    returns: occupancy (nz x nx), x_min, z_min, nx, nz
        -1 = unknown, 0 = free, 1 = occupied
    """
    # use only classified points for bounds
    classified = labels >= 0
    pts_c = points[classified]

    x_min = pts_c[:, 0].min() - 0.5
    x_max = pts_c[:, 0].max() + 0.5
    z_min = pts_c[:, 2].min() - 0.5
    z_max = pts_c[:, 2].max() + 0.5

    nx = int((x_max - x_min) / grid_res) + 1
    nz = int((z_max - z_min) / grid_res) + 1
    log(f"  Grid: {nx}x{nz} cells, X=[{x_min:.1f},{x_max:.1f}], Z=[{z_min:.1f},{z_max:.1f}]")

    floor_grid = np.zeros((nz, nx), dtype=np.int32)
    obs_grid   = np.zeros((nz, nx), dtype=np.int32)

    for label_val, grid in [(0, floor_grid), (1, obs_grid)]:
        mask = labels == label_val
        pts = points[mask]
        if len(pts) == 0:
            continue
        xi = np.clip(((pts[:, 0] - x_min) / grid_res).astype(int), 0, nx - 1)
        zi = np.clip(((pts[:, 2] - z_min) / grid_res).astype(int), 0, nz - 1)
        np.add.at(grid, (zi, xi), 1)

    occupancy = np.full((nz, nx), -1, dtype=np.int8)
    total = floor_grid + obs_grid
    has_data = total >= MIN_HITS_TOTAL
    occupancy[has_data & (obs_grid <  MIN_HITS_OBSTACLE)] = 0   # free
    occupancy[has_data & (obs_grid >= MIN_HITS_OBSTACLE)] = 1   # occupied

    n_free = (occupancy == 0).sum()
    n_occ  = (occupancy == 1).sum()
    n_unk  = (occupancy == -1).sum()
    log(f"  Cells: free={n_free}, occupied={n_occ}, unknown={n_unk}")
    return occupancy, x_min, z_min, nx, nz


def inflate_obstacles(occ, radius_cells):
    """inflate occupied cells by circular structuring element"""
    obstacle = (occ == 1)
    sz = 2 * radius_cells + 1
    struct = np.zeros((sz, sz), dtype=bool)
    c = radius_cells
    for y in range(sz):
        for x in range(sz):
            if (y - c) ** 2 + (x - c) ** 2 <= radius_cells ** 2:
                struct[y, x] = True
    inflated_mask = binary_dilation(obstacle, structure=struct)
    result = occ.copy()
    # inflated zone that was not already an obstacle → mark as 2
    result[inflated_mask & (occ != 1)] = 2
    log(f"  After inflation ({radius_cells} cells): blocked={(result > 0).sum()}")
    return result


# ============================================================
# a* path planning (8-connected)
# ============================================================
def find_nearest_free(grid, r, c, max_search=60):
    """find nearest free cell to (r, c)"""
    rows, cols = grid.shape
    if 0 <= r < rows and 0 <= c < cols and grid[r, c] == 0:
        return r, c
    for rad in range(1, max_search):
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                if abs(dr) != rad and abs(dc) != rad:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0:
                    return nr, nc
    return None


def astar(grid, start_rc, goal_rc):
    """A* on 2D grid (8-connected), cells > 0 are blocked

    returns list of (row, col) or None
    """
    rows, cols = grid.shape
    sr, sc = start_rc
    gr, gc = goal_rc

    # snap start/goal to nearest free cell
    start_free = find_nearest_free(grid, sr, sc)
    goal_free  = find_nearest_free(grid, gr, gc)
    if start_free is None or goal_free is None:
        log("  A*: could not find free start/goal")
        return None
    sr, sc = start_free
    gr, gc = goal_free
    log(f"  A*: ({sr},{sc}) → ({gr},{gc})")

    SQRT2 = math.sqrt(2)

    def heuristic(r, c):
        dr, dc = abs(r - gr), abs(c - gc)
        return max(dr, dc) + (SQRT2 - 1) * min(dr, dc)  # octile

    open_set = [(heuristic(sr, sc), 0.0, sr, sc)]
    g_score = {(sr, sc): 0.0}
    came_from = {}
    closed = set()
    dirs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    while open_set:
        f, g, r, c = heapq.heappop(open_set)
        if (r, c) == (gr, gc):
            path = [(r, c)]
            while (r, c) in came_from:
                r, c = came_from[(r, c)]
                path.append((r, c))
            return path[::-1]
        if (r, c) in closed:
            continue
        closed.add((r, c))
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in closed:
                if grid[nr, nc] != 0:   # only free cells (0) are passable
                    continue
                cost = SQRT2 if (dr != 0 and dc != 0) else 1.0
                ng = g + cost
                if ng < g_score.get((nr, nc), float("inf")):
                    g_score[(nr, nc)] = ng
                    came_from[(nr, nc)] = (r, c)
                    heapq.heappush(open_set, (ng + heuristic(nr, nc), ng, nr, nc))

    return None


# ============================================================
# gT → SLAM alignment (Umeyama / Sim3)
# ============================================================
def umeyama_alignment(source, target):
    """compute s, R, t such that target ≈ s*R*source + t.

    source, target: Nx3 matched point sets
    """
    assert len(source) == len(target) and len(source) >= 3
    n = len(source)
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)

    ds = source - mu_s
    dt = target - mu_t

    var_s = (ds ** 2).sum() / n

    cov = dt.T @ ds / n
    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / var_s
    t = mu_t - s * R @ mu_s
    return s, R, t


def align_gt_to_slam(slam_ts, slam_positions, gt_ts, gt_positions):
    """align GT positions into SLAM frame via Sim3"""
    matched_slam, matched_gt = [], []
    for i, ts in enumerate(gt_ts):
        idx = np.argmin(np.abs(slam_ts - ts))
        if abs(slam_ts[idx] - ts) < 0.5:
            matched_slam.append(slam_positions[idx])
            matched_gt.append(gt_positions[i])

    if len(matched_slam) < 10:
        log("  WARNING: too few matches for GT alignment, skipping GT")
        return None

    matched_slam = np.array(matched_slam)
    matched_gt = np.array(matched_gt)

    s, R, t = umeyama_alignment(matched_gt, matched_slam)
    gt_aligned = (s * (R @ gt_positions.T).T) + t
    residual = np.linalg.norm(matched_slam - (s * (R @ matched_gt.T).T + t), axis=1)
    log(f"  Sim3 alignment: scale={s:.4f}, residual mean={residual.mean():.3f}m")
    return gt_aligned


# ============================================================
# trajectory carving: mark robot path as free
# ============================================================
def carve_trajectory(occupancy, positions, x_min, z_min, grid_res, nz, nx,
                     radius_m=0.25):
    """mark cells along SLAM trajectory as free

    the robot drove through these so they must be passable
    uses a circular brush of `radius_m` around each trajectory point.
    """
    radius_cells = int(round(radius_m / grid_res))
    carved = 0
    for pos in positions:
        col = int((pos[0] - x_min) / grid_res)
        row = int((pos[2] - z_min) / grid_res)
        for dr in range(-radius_cells, radius_cells + 1):
            for dc in range(-radius_cells, radius_cells + 1):
                if dr * dr + dc * dc > radius_cells * radius_cells:
                    continue
                r, c = row + dr, col + dc
                if 0 <= r < nz and 0 <= c < nx:
                    if occupancy[r, c] != 0:
                        occupancy[r, c] = 0
                        carved += 1
    return carved


# ============================================================
# coordinate helpers
# ============================================================
def world_to_grid(x, z, x_min, z_min, res, nz, nx):
    col = int((x - x_min) / res)
    row = int((z - z_min) / res)
    return max(0, min(nz - 1, row)), max(0, min(nx - 1, col))


# ============================================================
# ============================================================
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ---- 1. Load data ----
    log("Loading ORB-SLAM3 poses...")
    slam_ts, slam_transforms = load_poses(TRAJ_PATH)
    slam_positions = np.array([T[:3, 3] for T in slam_transforms])
    log(f"  {len(slam_ts)} poses")

    log("Loading GT...")
    gt_ts, gt_positions = load_gt(GT_PATH)
    log(f"  {len(gt_ts)} GT poses")

    log("Indexing depth images...")
    depth_files = sorted(os.listdir(DEPTH_DIR))
    depth_timestamps = np.array([float(f.replace(".png", "")) for f in depth_files])
    log(f"  {len(depth_files)} depth images")

    # ---- 2. Back-project ----
    log(f"Back-projecting depth (frame_step={FRAME_STEP}, pixel_step={PIXEL_STEP})...")
    all_points = []
    all_labels = []
    n_processed = 0
    t0 = time.time()

    for i in range(0, len(slam_ts), FRAME_STEP):
        ts = slam_ts[i]
        didx = np.argmin(np.abs(depth_timestamps - ts))
        if abs(depth_timestamps[didx] - ts) > 0.05:
            continue

        depth_path = os.path.join(DEPTH_DIR, depth_files[didx])
        depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth_img is None:
            continue

        pts, lbl = backproject_depth(depth_img, slam_transforms[i], PIXEL_STEP)
        if len(pts) > 0:
            all_points.append(pts)
            all_labels.append(lbl)
            n_processed += 1

        if n_processed % 100 == 0 and n_processed > 0:
            log(f"  ... {n_processed} frames processed")

    all_points = np.vstack(all_points)
    all_labels = np.concatenate(all_labels)
    n_floor = (all_labels == 0).sum()
    n_obs   = (all_labels == 1).sum()
    n_ign   = (all_labels == -1).sum()
    log(f"  Done: {n_processed} frames, {len(all_points):,} points in {time.time()-t0:.1f}s")
    log(f"  Labels: floor={n_floor:,}, obstacle={n_obs:,}, ignore={n_ign:,}")

    # ---- 3. Build occupancy grid ----
    log("Building occupancy grid...")
    occupancy, x_min, z_min, nx, nz = build_occupancy(all_points, all_labels, GRID_RES)

    # ---- 3b. Trajectory carving: robot drove here → free ----
    log("Carving trajectory into occupancy grid (radius=0.25m)...")
    # subsample trajectory for speed (every 5th pose)
    n_carved = carve_trajectory(occupancy, slam_positions[::5],
                                x_min, z_min, GRID_RES, nz, nx, radius_m=0.25)
    log(f"  Carved {n_carved} cells along trajectory")
    log(f"  Updated: free={(occupancy==0).sum()}, occupied={(occupancy==1).sum()}")

    # ---- 4. Inflate ----
    log(f"Inflating obstacles ({INFLATE_CELLS} cells = {INFLATE_RADIUS_M}m)...")
    inflated = inflate_obstacles(occupancy, INFLATE_CELLS)

    # ---- 5. Align GT → SLAM frame ----
    log("Aligning GT to SLAM frame (Sim3)...")
    gt_aligned = align_gt_to_slam(slam_ts, slam_positions, gt_ts, gt_positions)

    # ---- 6. A* path planning, multiple routes ----
    # 3 navigation queries across different parts of the garden
    n = len(slam_positions)
    route_defs = [
        ("A: Start → Far corner",
         slam_positions[0],                       # origin
         slam_positions[int(n * 0.25)],            # far bottom-right
         "#ee3333"),
        ("B: Top-left → Bottom-right",
         slam_positions[int(n * 0.10)],            # top area
         slam_positions[int(n * 0.60)],            # bottom area
         "#ff8800"),
        ("C: Mid-left → Mid-right",
         slam_positions[int(n * 0.40)],            # left side
         slam_positions[int(n * 0.50)],            # right side
         "#aa22cc"),
    ]

    # find best inflation that works for at least one route
    used_inflated = inflated
    best_inflate = INFLATE_CELLS

    routes = []  # list of (name, start_xyz, goal_xyz, path, length_m, color)

    for route_name, s_xyz, g_xyz, color in route_defs:
        euclid = np.linalg.norm(s_xyz[[0, 2]] - g_xyz[[0, 2]])
        log(f"\n  Route {route_name}")
        log(f"    ({s_xyz[0]:.1f},{s_xyz[2]:.1f}) → ({g_xyz[0]:.1f},{g_xyz[2]:.1f}), euclidean={euclid:.1f}m")

        s_rc = world_to_grid(s_xyz[0], s_xyz[2], x_min, z_min, GRID_RES, nz, nx)
        g_rc = world_to_grid(g_xyz[0], g_xyz[2], x_min, z_min, GRID_RES, nz, nx)

        # try progressively less inflation
        path = None
        for inflate_r in [INFLATE_CELLS, 4, 3, 2, 1, 0]:
            if inflate_r == 0:
                test_grid = occupancy
            elif inflate_r == INFLATE_CELLS:
                test_grid = inflated
            else:
                test_grid = inflate_obstacles(occupancy, inflate_r)

            path = astar(test_grid, s_rc, g_rc)
            if path is not None:
                if inflate_r < best_inflate:
                    used_inflated = test_grid
                    best_inflate = inflate_r
                break

        if path is not None:
            length_m = sum(
                math.sqrt((path[i][0] - path[i-1][0])**2 +
                          (path[i][1] - path[i-1][1])**2)
                for i in range(1, len(path))
            ) * GRID_RES
            log(f"    Path: {len(path)} cells, {length_m:.1f}m (inflation={inflate_r})")
            routes.append((route_name, s_xyz, g_xyz, path, length_m, color))
        else:
            log(f"    NO PATH FOUND")
            routes.append((route_name, s_xyz, g_xyz, None, 0, color))

    log(f"\n  Final inflation used: {best_inflate} cells ({best_inflate * GRID_RES:.2f}m)")

    # ---- 7. Visualize ----
    log("Creating 3-panel figure...")
    fig, axes = plt.subplots(1, 3, figsize=(27, 9))

    extent = [x_min, x_min + nx * GRID_RES, z_min, z_min + nz * GRID_RES]

    def grid_to_rgb(g):
        rgb = np.ones((g.shape[0], g.shape[1], 3))
        rgb[g == -1] = [0.92, 0.92, 0.92]
        rgb[g ==  0] = [1.00, 1.00, 1.00]
        rgb[g ==  1] = [0.12, 0.12, 0.12]
        rgb[g ==  2] = [0.65, 0.68, 0.80]
        return rgb

    step = max(1, len(slam_positions) // 2000)

    # -- Panel 1: Occupancy grid + GT + SLAM trajectories --
    ax1 = axes[0]
    ax1.imshow(grid_to_rgb(occupancy), origin="lower", extent=extent, aspect="equal")
    ax1.plot(slam_positions[::step, 0], slam_positions[::step, 2],
             "-", color="#3366cc", linewidth=0.8, alpha=0.5, label="ORB-SLAM3")
    if gt_aligned is not None:
        ax1.plot(gt_aligned[:, 0], gt_aligned[:, 2],
                 "-", color="#22aa22", linewidth=1.8, alpha=0.9, label="GT (Sim3-aligned)")

    # mark all route endpoints
    for rname, s_xyz, g_xyz, rpath, rlen, rcol in routes:
        short = rname.split(":")[0]  # "A", "B", "C"
        ax1.plot(s_xyz[0], s_xyz[2], "o", color=rcol, markersize=8, zorder=5)
        ax1.plot(g_xyz[0], g_xyz[2], "*", color=rcol, markersize=11, zorder=5)
        ax1.annotate(f"{short}s", (s_xyz[0], s_xyz[2]), fontsize=8,
                     fontweight="bold", color=rcol, xytext=(4, 4),
                     textcoords="offset points")
        ax1.annotate(f"{short}g", (g_xyz[0], g_xyz[2]), fontsize=8,
                     fontweight="bold", color=rcol, xytext=(4, 4),
                     textcoords="offset points")

    handles1 = [
        mpatches.Patch(facecolor=[0.12]*3, label="Obstacle"),
        mpatches.Patch(facecolor=[0.92]*3, edgecolor=[0.7]*3, label="Unknown"),
        mpatches.Patch(facecolor=[1.0]*3,  edgecolor=[0.7]*3, label="Free"),
    ]
    handles1 += ax1.get_legend_handles_labels()[0]
    ax1.legend(handles=handles1, loc="upper right", fontsize=7)
    ax1.set_title("Occupancy Grid + Trajectories", fontsize=13, fontweight="bold")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Z (m)")

    # -- Panel 2: Inflated grid + all A* routes --
    ax2 = axes[1]
    ax2.imshow(grid_to_rgb(used_inflated), origin="lower", extent=extent, aspect="equal")
    ax2.plot(slam_positions[::step, 0], slam_positions[::step, 2],
             "-", color="#3366cc", linewidth=0.5, alpha=0.2)

    for rname, s_xyz, g_xyz, rpath, rlen, rcol in routes:
        short = rname.split(":")[0]
        ax2.plot(s_xyz[0], s_xyz[2], "o", color=rcol, markersize=9, zorder=5)
        ax2.plot(g_xyz[0], g_xyz[2], "*", color=rcol, markersize=12, zorder=5)
        if rpath is not None:
            parr = np.array(rpath)
            px = x_min + parr[:, 1] * GRID_RES
            pz = z_min + parr[:, 0] * GRID_RES
            ax2.plot(px, pz, "-", color=rcol, linewidth=2.5,
                     label=f"{short}: {rlen:.1f}m")
        else:
            ax2.plot([], [], "-", color=rcol, linewidth=2.5,
                     label=f"{short}: NO PATH")

    handles2 = [mpatches.Patch(facecolor=[0.65, 0.68, 0.80], edgecolor=[0.5]*3,
                               label="Inflated")]
    handles2 += ax2.get_legend_handles_labels()[0]
    ax2.legend(handles=handles2, loc="upper right", fontsize=8)
    ax2.set_title("Inflated Grid + A* Routes (3 queries)", fontsize=13, fontweight="bold")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Z (m)")

    # -- Panel 3: Point cloud colored by height --
    ax3 = axes[2]
    sub = max(1, len(all_points) // 150000)
    pts_sub = all_points[::sub]
    y_clipped = np.clip(pts_sub[:, 1], -2.0, 0.5)
    sc = ax3.scatter(pts_sub[:, 0], pts_sub[:, 2], c=y_clipped, s=0.15,
                     cmap="viridis_r", alpha=0.35, rasterized=True)
    cbar = plt.colorbar(sc, ax=ax3, shrink=0.75, pad=0.02)
    cbar.set_label("Height Y (m)  [+ = ground, - = above]", fontsize=9)
    ax3.plot(slam_positions[::step, 0], slam_positions[::step, 2],
             "-", color="#ee3333", linewidth=0.9, alpha=0.8, label="SLAM trajectory")
    ax3.legend(loc="upper right", fontsize=9)
    ax3.set_title("Point Cloud (top-down, colored by height)", fontsize=13, fontweight="bold")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Z (m)")
    ax3.set_aspect("equal")

    plt.tight_layout()
    out_path = os.path.join(OUTPUT_DIR, "occupancy_grid_astar.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    log(f"Saved figure: {out_path}")

    # save data
    npz_path = os.path.join(OUTPUT_DIR, "occupancy_data.npz")
    np.savez_compressed(npz_path,
                        occupancy=occupancy,
                        inflated=used_inflated,
                        x_min=x_min, z_min=z_min,
                        grid_res=GRID_RES,
                        all_points_sub=all_points[::max(1, len(all_points)//500000)])
    log(f"Saved data: {npz_path}")

    # summary
    log("")
    log("=" * 50)
    log("SUMMARY")
    log("=" * 50)
    log(f"  Grid:       {nx} x {nz} cells @ {GRID_RES*100:.0f} cm")
    log(f"  Free:       {(occupancy==0).sum():,}")
    log(f"  Occupied:   {(occupancy==1).sum():,}")
    log(f"  Inflation:  {best_inflate} cells ({best_inflate * GRID_RES:.2f}m)")
    log(f"  Routes:")
    for rname, s_xyz, g_xyz, rpath, rlen, rcol in routes:
        short = rname.split(":")[0]
        euclid = np.linalg.norm(s_xyz[[0, 2]] - g_xyz[[0, 2]])
        if rpath is not None:
            log(f"    {short}: {rlen:.1f}m path (euclidean {euclid:.1f}m, ratio {rlen/euclid:.2f})")
        else:
            log(f"    {short}: NO PATH (euclidean {euclid:.1f}m)")
    log(f"  Output:     {out_path}")
    log("Done!")


if __name__ == "__main__":
    main()
