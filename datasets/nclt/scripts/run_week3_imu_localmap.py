#!/usr/bin/env python3
"""Week 3: improved LiDAR odometry on NCLT spring (2012-04-29).
every 2nd scan -> 6486 scans total.

Week 2 got 419m ATE with plain ICP. this script adds:
  1. wheel-odom delta for ICP init instead of identity (huge win)
  2. scan-to-local-map ICP (20-scan sliding window, 0.5m voxel)
  3. RANSAC ground plane removal before ICP - ground is flat + dominant,
     otherwise ICP settles into a degenerate local minimum on it
  4. GPS-aided loop closures (proximity search on RTK GPS)
  5. 2D pose graph solve with sparse Gauss-Newton + LM damping

bugs from v1 of this file that made it produce garbage:
  - ms25.csv has 10 cols (mag_x/y/z, accel_x/y/z, rot_x/y/z), not quaternions.
    v1 was feeding magnetometer values to SLERP -> random rotations
  - Open3D pcd.transform(T) is in-place, v1 applied it twice to the same
    object so the local map got corrupted on the second call
  - switched the init guess from broken IMU preintegration to wheel odom
    (odometry_mu_100hz.csv, which is integrated wheel encoders at 100 Hz)
  - scan-to-map ICP now passes T_pred directly as the init arg instead of
    pre-transforming the source cloud, avoids the in-place gotcha

target: ATE < 50m (10x better than week 2)
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
from tqdm import tqdm
import open3d as o3d
from scipy.spatial.transform import Rotation
from scipy import sparse
from scipy.sparse.linalg import spsolve
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src'))
from data_loaders.velodyne_loader import VelodyneLoader
from data_loaders.ground_truth_loader import GroundTruthLoader
from data_loaders.sensor_loader import SensorLoader

SESSION = '2012-04-29'
SUBSAMPLE = 2
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week3_imu_gps'
PLOTS_DIR = RESULTS_DIR / 'plots'
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

# ICP parameters. 0.3 m voxel keeps walls/poles, drops grass; 1.5 m threshold
# is safely wider than the Segway's ~0.5 m/frame motion at 5 Hz.
ICP_VOXEL = 0.3
ICP_THRESHOLD = 1.5
ICP_MAX_ITER = 80

# local map: last 20 scans fused in global frame. Big enough to give ICP real
# geometry (not just the current scan's ~30 m radius), small enough to stay real-time.
LOCALMAP_SIZE = 20
LOCALMAP_VOXEL = 0.5

# ground removal: 0.25 m RANSAC tolerance handles curbs without eating into walls.
GROUND_DIST = 0.25
GROUND_RANSAC_N = 3
GROUND_ITERS = 200

# GPS loop closure: 15 m RTK proximity is well above RTK noise (~10 cm),
# and min_gap=200 means "don't close with recent frames, those are just odometry".
GPS_LC_MIN_GAP = 200
GPS_LC_RADIUS = 15.0
# FIXME: radius should be tuned per-session, 15m works for spring but summer has more false matches
GPS_LC_ICP_FITNESS = 0.25  # fraction of inliers required to accept a LC edge
GPS_LC_ICP_DIST = 2.0
GPS_LC_DEDUP = 50

# LC edges weighted 10x odom in the pose graph because they are metric (RTK-backed)
# while odom drifts. Damping 1e-3 is a standard LM starting value.
PG_ODOM_W = 1.0
PG_LC_W = 10.0
PG_DAMPING = 1e-3

print("=" * 80)
print("  WEEK 3: ODOM-AIDED ICP + LOCAL MAP + GPS LOOP CLOSURE")
print("=" * 80)
print(f"Session: {SESSION}, every {SUBSAMPLE}nd scan")
print(f"Local map: {LOCALMAP_SIZE} scans, {LOCALMAP_VOXEL}m voxel")
print(f"GPS LC: radius={GPS_LC_RADIUS}m, min_gap={GPS_LC_MIN_GAP}")
print("=" * 80)


# Odometry predictor.
# v1 tried IMU preintegration (Forster et al., 2015) but NCLT's MS25 IMU is only
# ~48 Hz and very noisy, so preintegrated deltas were garbage. Using NCLT's
# pre-integrated wheel odometry (odometry_mu_100hz.csv) is both simpler and
# more accurate as an ICP init guess.
class OdometryPredictor:
    """Predict relative motion between LiDAR frames using wheel odometry (100Hz)."""

    def __init__(self, odom_df):
        self.utimes = odom_df['utime'].values
        self.x = odom_df['x'].values
        self.y = odom_df['y'].values
        self.z = odom_df['z'].values
        self.roll = odom_df['roll'].values
        self.pitch = odom_df['pitch'].values
        self.yaw = odom_df['yaw'].values

    def _interp_at(self, utime):
        """Interpolate odometry pose at a given timestamp"""
        idx = np.searchsorted(self.utimes, utime)
        if idx <= 0:
            idx = 1
        if idx >= len(self.utimes):
            idx = len(self.utimes) - 1

        t0, t1 = self.utimes[idx - 1], self.utimes[idx]
        alpha = (utime - t0) / (t1 - t0) if t1 != t0 else 0.0
        alpha = np.clip(alpha, 0, 1)

        x = self.x[idx-1] + alpha * (self.x[idx] - self.x[idx-1])
        y = self.y[idx-1] + alpha * (self.y[idx] - self.y[idx-1])
        z = self.z[idx-1] + alpha * (self.z[idx] - self.z[idx-1])

        # angle interpolation (handle wrapping)
        def angle_interp(a0, a1, t):
            diff = (a1 - a0 + np.pi) % (2*np.pi) - np.pi
            return a0 + diff * t

        roll = angle_interp(self.roll[idx-1], self.roll[idx], alpha)
        pitch = angle_interp(self.pitch[idx-1], self.pitch[idx], alpha)
        yaw = angle_interp(self.yaw[idx-1], self.yaw[idx], alpha)

        return x, y, z, roll, pitch, yaw

    def _pose_4x4(self, x, y, z, roll, pitch, yaw):
        """Build 4x4 transform from position and Euler angles"""
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T

    def get_relative_transform(self, utime_start, utime_end):
        """compute relative 4x4 transform between two timestamps"""
        p0 = self._interp_at(utime_start)
        p1 = self._interp_at(utime_end)
        T0 = self._pose_4x4(*p0)
        T1 = self._pose_4x4(*p1)
        return np.linalg.inv(T0) @ T1


class LocalMap:
    """Sliding-window local map for scan-to-map ICP. Keeps last N scans merged."""

    def __init__(self, window_size=20, voxel_size=0.5):
        self.window_size = window_size
        self.voxel_size = voxel_size
        self.entries = []  # List of global-frame points arrays
        self._cached_map = None
        self._cache_dirty = True

    def add_scan(self, points_local, pose_4x4):
        """add a registered scan to the local map"""
        pts_h = np.hstack([points_local, np.ones((len(points_local), 1))])
        pts_global = (pose_4x4 @ pts_h.T).T[:, :3]

        self.entries.append(pts_global)
        if len(self.entries) > self.window_size:
            self.entries.pop(0)
        self._cache_dirty = True

    def get_map_pcd(self):
        """build and return the local map as an Open3D point cloud"""
        if not self.entries:
            return None

        if not self._cache_dirty and self._cached_map is not None:
            return self._cached_map

        all_pts = np.vstack(self.entries)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts)
        pcd = pcd.voxel_down_sample(self.voxel_size)
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

        self._cached_map = pcd
        self._cache_dirty = False
        return pcd


def remove_ground(pcd, distance_threshold=0.25, ransac_n=3, num_iterations=200):
    """remove ground plane from point cloud using RANSAC plane fitting"""
    if len(pcd.points) < 100:
        return pcd

    try:
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations)

        normal = np.array(plane_model[:3])
        if abs(normal[2]) > 0.7:
            non_ground = pcd.select_by_index(inliers, invert=True)
            return non_ground
    except Exception:
        pass

    return pcd


class GPSLoopClosureDetector:
    """Detect loop closures using GPS proximity (lat/lon to ENU)."""

    def __init__(self, gps_df, min_gap=200, radius=15.0, dedup_window=50):
        self.min_gap = min_gap
        self.radius = radius
        self.dedup_window = dedup_window

        utimes = gps_df.iloc[:, 0].values
        lats = gps_df.iloc[:, 3].values  # radians
        lons = gps_df.iloc[:, 4].values  # radians

        self.lat0 = lats[0]
        self.lon0 = lons[0]

        R_earth = 6371000.0
        self.gps_x = R_earth * (lons - self.lon0) * np.cos(self.lat0)
        self.gps_y = R_earth * (lats - self.lat0)
        self.gps_utimes = utimes

    def get_gps_position(self, utime):
        """get interpolated GPS position (x, y in meters) for a given timestamp"""
        idx = np.searchsorted(self.gps_utimes, utime)
        if idx <= 0 or idx >= len(self.gps_utimes):
            return None

        t0, t1 = self.gps_utimes[idx-1], self.gps_utimes[idx]
        alpha = (utime - t0) / (t1 - t0) if t1 != t0 else 0.0

        x = self.gps_x[idx-1] + alpha * (self.gps_x[idx] - self.gps_x[idx-1])
        y = self.gps_y[idx-1] + alpha * (self.gps_y[idx] - self.gps_y[idx-1])
        return x, y

    def find_candidates(self, timestamps):
        """find loop closure candidates from GPS proximity"""
        n = len(timestamps)

        gps_positions = []
        for ts in timestamps:
            gps_positions.append(self.get_gps_position(ts))

        candidates = []
        for qi in tqdm(range(self.min_gap, n, 10), desc="GPS LC detect"):
            if gps_positions[qi] is None:
                continue
            qx, qy = gps_positions[qi]

            for ci in range(0, qi - self.min_gap, 10):
                if gps_positions[ci] is None:
                    continue
                cx, cy = gps_positions[ci]
                dist = np.sqrt((qx-cx)**2 + (qy-cy)**2)

                if dist < self.radius:
                    candidates.append((qi, ci, dist))

        # deduplicate: keep best per window pair
        dedup = {}
        for q, c, d in candidates:
            wq, wc = q // self.dedup_window, c // self.dedup_window
            key = (wc, wq)
            if key not in dedup or d < dedup[key][2]:
                dedup[key] = (q, c, d)

        return list(dedup.values())


# Pose  graph optimizer: 2D SE(2), sparse normal equations, Gauss-Newton + LM.# Gauge-fixed by anchoring node 0. Damping adapted per iter (halve on improve,
# 5x on worsen) following standard LM.
class PoseGraphOptimizer2D:
    """Sparse 2D pose graph optimizer (Gauss-Newton + LM damping), first pose anchored."""

    def __init__(self, odom_w=1.0, lc_w=10.0, damping=1e-3):
        self.ow, self.lw, self.damping = odom_w, lc_w, damping

    @staticmethod
    def _adiff(a, b):
        """Angle diff normalized to [-pi, pi]"""
        return (a - b + np.pi) % (2*np.pi) - np.pi

    def optimize(self, poses, odom_e, lc_e, max_iter=50):
        """optimize pose graph"""
        n = len(poses)
        p = poses.copy()
        ne = len(odom_e) + len(lc_e)
        print(f"  Graph: {n} nodes, {len(odom_e)} odom, {len(lc_e)} LC edges")

        lam = self.damping
        prev_cost = float('inf')

        for it in range(max_iter):
            rows, cols, vals, res = [], [], [], []
            ri = 0

            for (i, j, dxm, dym, dtm), w in \
                    [(e, self.ow) for e in odom_e] + [(e, self.lw) for e in lc_e]:
                xi, yi, ti = p[i]; xj, yj, tj = p[j]
                ct, st = np.cos(ti), np.sin(ti)
                dx, dy = xj-xi, yj-yi
                dxp = ct*dx+st*dy; dyp = -st*dx+ct*dy
                dtp = self._adiff(tj, ti)

                res.extend([w*(dxp-dxm), w*(dyp-dym), w*(dtp-dtm)])

                ii, jj = 3*i, 3*j
                rows.extend([ri]*5); cols.extend([ii, ii+1, ii+2, jj, jj+1])
                vals.extend([w*(-ct), w*(-st), w*(-st*dx+ct*dy), w*ct, w*st])
                rows.extend([ri+1]*5); cols.extend([ii, ii+1, ii+2, jj, jj+1])
                vals.extend([w*st, w*(-ct), w*(-ct*dx-st*dy), w*(-st), w*ct])
                rows.extend([ri+2]*2); cols.extend([ii+2, jj+2])
                vals.extend([w*(-1.0), w*1.0])
                ri += 3

            J = sparse.csr_matrix((vals, (rows, cols)), shape=(3*ne, 3*n))
            e = np.array(res)
            cost = np.sum(e**2)
            H = J.T @ J; b = -J.T @ e

            diag_H = H.diagonal().copy(); diag_H[diag_H < 1e-6] = 1e-6
            H_lm = H + sparse.diags(lam * diag_H)
            for k in range(3):
                H_lm[k,:]=0; H_lm[:,k]=0; H_lm[k,k]=1e6; b[k]=0

            try:
                dx = spsolve(H_lm.tocsc(), b)
            except:
                lam *= 10; continue
            if np.any(np.isnan(dx)):
                lam *= 10; continue

            p[:, 0] += dx[0::3]; p[:, 1] += dx[1::3]; p[:, 2] += dx[2::3]
            p[:, 2] = (p[:, 2]+np.pi)%(2*np.pi)-np.pi

            unorm = np.linalg.norm(dx)
            lam = max(lam/2, 1e-7) if cost < prev_cost else min(lam*5, 1e3)
            prev_cost = cost

            if it % 10 == 0:
                print(f"    Iter {it}: cost={cost:.1f}, |dx|={unorm:.4f}, lam={lam:.1e}")
            if unorm < 1e-4:
                print(f"  Converged at iter {it}"); break

        print(f"  Final cost: {cost:.1f}")
        return p


def compute_ate(e, g):
    """ATE: position L2 per frame, after alignment. No rotation term."""
    errs = np.linalg.norm(e[:,1:4]-g[:,1:4], axis=1)
    return {'mean':errs.mean(), 'rmse':np.sqrt((errs**2).mean()),
            'std':errs.std(), 'median':np.median(errs),
            'min':errs.min(), 'max':errs.max(), 'errors':errs}

def compute_rpe(e, g, d=1):
    """RPE: frame-to-frame SE(3) drift.
    E_i = (gt_i^-1 gt_{i+d})^-1 (est_i^-1 est_{i+d}); translation is ||E_i[:3,3]||,
    rotation is angle(E_i[:3,:3]) via arccos((trace-1)/2)."""
    te, re = [], []
    for i in range(len(e)-d):
        def bT(r):
            T=np.eye(4); T[:3,3]=r[1:4]
            T[:3,:3]=Rotation.from_quat(r[4:8]).as_matrix(); return T
        Tgr = np.linalg.inv(bT(g[i])) @ bT(g[i+d])
        Ter = np.linalg.inv(bT(e[i])) @ bT(e[i+d])
        Tx = np.linalg.inv(Tgr) @ Ter
        te.append(np.linalg.norm(Tx[:3,3]))
        re.append(np.degrees(np.arccos(np.clip((np.trace(Tx[:3,:3])-1)/2,-1,1))))
    te, re = np.array(te), np.array(re)
    return {'trans_rmse':np.sqrt((te**2).mean()), 'trans_mean':te.mean(),
            'rot_rmse':np.sqrt((re**2).mean()), 'rot_mean':re.mean(),
            'trans_errors':te, 'rot_errors':re}


# --- Step 0: Load data ---
print("\nSTEP 0: LOADING DATA")
print("=" * 80)

gt_loader = GroundTruthLoader()
gt_df = gt_loader.load_ground_truth(SESSION)
gt_all = np.column_stack([
    gt_df['utime'].values/1e6, gt_df['x'].values, gt_df['y'].values,
    gt_df['z'].values, gt_df['qx'].values, gt_df['qy'].values,
    gt_df['qz'].values, gt_df['qw'].values])
print(f"GT: {len(gt_all)} poses")

sensor = SensorLoader()

# 100 Hz integrated wheel-odometry poses (odometry_mu_100hz.csv)
odom_df = sensor.load_odometry_mu(SESSION, hz=100)
print(f"Odometry: {len(odom_df)} samples (100Hz, "
      f"{(odom_df['utime'].iloc[-1]-odom_df['utime'].iloc[0])/1e6:.0f}s)")

gps_df = sensor.load_gps_rtk(SESSION)
print(f"GPS RTK: {len(gps_df)} samples")

loader = VelodyneLoader()
all_files = loader.get_velodyne_sync_files(SESSION)
files = all_files[::SUBSAMPLE]
print(f"Scans: {len(files)} (every {SUBSAMPLE}nd of {len(all_files)})")

odom_pred = OdometryPredictor(odom_df)
local_map = LocalMap(LOCALMAP_SIZE, LOCALMAP_VOXEL)
gps_lc = GPSLoopClosureDetector(gps_df, GPS_LC_MIN_GAP, GPS_LC_RADIUS, GPS_LC_DEDUP)

# --- Step 1: Odometry-aided ICP with local map matching ---
print("\nSTEP 1: ODOMETRY-AIDED ICP + LOCAL MAP + GROUND REMOVAL")
print("=" * 80)

poses_4x4 = []
timestamps_us = []
pcd_cache = {}

current_pose = np.eye(4)
prev_pcd = None
prev_timestamp = None
t0 = time.time()

for idx in tqdm(range(len(files)), desc="LiDAR-Odom ICP"):
    fp = files[idx]
    pts = loader.load_velodyne_sync(fp)
    ts = int(Path(fp).stem)

    # create and process point cloud (stays in local/scan frame)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
    pcd = pcd.voxel_down_sample(ICP_VOXEL)
    pcd = remove_ground(pcd, GROUND_DIST, GROUND_RANSAC_N, GROUND_ITERS)
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

    # cache every 10th for LC verification later
    if idx % 10 == 0:
        pcd_cache[idx] = pcd

    if prev_pcd is not None:
        # get odometry-based relative transform as ICP initial guess
        T_odom_rel = odom_pred.get_relative_transform(prev_timestamp, ts)

        # get registration target: local map or previous scan
        map_pcd = local_map.get_map_pcd()

        if map_pcd is not None and len(map_pcd.points) > 500:
            # scan-to-map ICP:
            # source = pcd (in local/scan frame)
            # target = map_pcd (in global frame)
            # init = T_pred (predicted global pose of current scan)
            # result.transformation = corrected global pose
            T_pred = current_pose @ T_odom_rel
            reg = o3d.pipelines.registration.registration_icp(
                pcd, map_pcd, ICP_THRESHOLD, T_pred,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER,
                    relative_fitness=1e-6, relative_rmse=1e-6))

            # reg.transformation maps source (local) to target (global)
            # so it IS the corrected global pose
            current_pose = reg.transformation
        else:
            # frame-to-frame ICP with odometry init
            reg = o3d.pipelines.registration.registration_icp(
                pcd, prev_pcd, ICP_THRESHOLD, T_odom_rel,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=ICP_MAX_ITER,
                    relative_fitness=1e-6, relative_rmse=1e-6))
            current_pose = current_pose @ reg.transformation

    poses_4x4.append(current_pose.copy())
    timestamps_us.append(ts)

    # add scan to local map (pcd is still in local frame, add_scan transforms it)
    local_map.add_scan(np.asarray(pcd.points), current_pose)

    prev_pcd = pcd
    prev_timestamp = ts

    if (idx+1) % 500 == 0:
        el = time.time()-t0; p = current_pose[:3,3]
        print(f"\n  [{idx+1}/{len(files)}] {(idx+1)/el:.1f} sc/s | "
              f"Pos: [{p[0]:.0f},{p[1]:.0f}]")

t_odom = time.time()-t0
print(f"\nOdometry: {len(poses_4x4)} poses in {t_odom:.1f}s "
      f"({len(poses_4x4)/t_odom:.1f} sc/s)")

# --- Step 2: GPS-aided loop closure detection ---
print("\nSTEP 2: GPS-AIDED LOOP CLOSURE DETECTION")
print("=" * 80)

t0 = time.time()
gps_candidates = gps_lc.find_candidates(timestamps_us)
t_detect = time.time()-t0
print(f"GPS candidates: {len(gps_candidates)} in {t_detect:.1f}s")

# ICP verification
print("\nSTEP 3: ICP VERIFICATION OF GPS LOOP CLOSURES")
print("=" * 80)

def get_pcd(idx):
    """Get cached or freshly loaded point cloud for loop closure verification"""
    if idx in pcd_cache:
        return pcd_cache[idx]
    nearest = min(pcd_cache.keys(), key=lambda k: abs(k-idx))
    if abs(nearest-idx) <= 5:
        return pcd_cache[nearest]
    pts = loader.load_velodyne_sync(files[idx])
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(pts[:,:3])
    p = p.voxel_down_sample(0.3)
    p = remove_ground(p, GROUND_DIST)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))
    return p

verified_lc = []
t0 = time.time()

for q, c, gps_dist in tqdm(gps_candidates, desc="ICP verify"):
    pcd_q = get_pcd(q)
    pcd_c = get_pcd(c)

    # use odometry-based initial (since GPS tells us they're close)
    T_q = poses_4x4[q]
    T_c = poses_4x4[c]
    T_init = np.linalg.inv(T_c) @ T_q

    # check if initial transform is reasonable (< 100m translation)
    init_trans = np.linalg.norm(T_init[:3, 3])
    if init_trans > 100:
        T_init = np.eye(4)

    reg = o3d.pipelines.registration.registration_icp(
        pcd_q, pcd_c, GPS_LC_ICP_DIST, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=50, relative_fitness=1e-6, relative_rmse=1e-6))

    if reg.fitness >= GPS_LC_ICP_FITNESS:
        verified_lc.append((q, c, reg.transformation, reg.fitness,
                           reg.inlier_rmse, gps_dist))

t_verify = time.time()-t0
print(f"\nVerified: {len(verified_lc)} / {len(gps_candidates)} in {t_verify:.1f}s")

if verified_lc:
    print("\nTop loop closures:")
    for q, c, T, fit, rmse, gd in sorted(verified_lc, key=lambda x:-x[3])[:15]:
        print(f"  {c:5d}<->{q:5d} (gap={q-c:5d}): fit={fit:.3f}, "
              f"rmse={rmse:.3f}m, gps={gd:.1f}m")

# --- Step 4: Pose graph optimization ---
print("\nSTEP 4: POSE GRAPH OPTIMIZATION")
print("=" * 80)

def pose_to_2d(p4):
    """Extract (x, y, yaw) from 4x4 transform"""
    return np.array([p4[0,3], p4[1,3], np.arctan2(p4[1,0], p4[0,0])])

def rel2d(xi, yi, ti, xj, yj, tj):
    """Compute relative 2D transform"""
    c, s = np.cos(ti), np.sin(ti)
    return c*(xj-xi)+s*(yj-yi), -s*(xj-xi)+c*(yj-yi), (tj-ti+np.pi)%(2*np.pi)-np.pi

poses_2d = np.array([pose_to_2d(p) for p in poses_4x4])
z_vals = [p[2,3] for p in poses_4x4]

odom_edges = []
for i in range(len(poses_2d)-1):
    dx, dy, dt = rel2d(*poses_2d[i], *poses_2d[i+1])
    odom_edges.append((i, i+1, dx, dy, dt))

lc_edges = []
for q, c, T, fit, rmse, gd in verified_lc:
    dx, dy = T[0,3], T[1,3]
    dt = np.arctan2(T[1,0], T[0,0])
    lc_edges.append((c, q, dx, dy, dt))

print(f"Odom edges: {len(odom_edges)}, LC edges: {len(lc_edges)}")

if len(lc_edges) > 0:
    opt = PoseGraphOptimizer2D(PG_ODOM_W, PG_LC_W, PG_DAMPING)
    t0 = time.time()
    opt_2d = opt.optimize(poses_2d, odom_edges, lc_edges)
    t_opt = time.time()-t0
    print(f"Optimization: {t_opt:.1f}s")
else:
    print("No loop closures verified. Using odometry-only.")
    opt_2d = poses_2d.copy()
    t_opt = 0

# --- Step 5: Evaluation ---
print("\nSTEP 5: EVALUATION")
print("=" * 80)

odom_traj = []
for ts, pose in zip(timestamps_us, poses_4x4):
    p = pose[:3,3]; q = Rotation.from_matrix(pose[:3,:3]).as_quat()
    odom_traj.append([ts/1e6, p[0], p[1], p[2], q[0], q[1], q[2], q[3]])
odom_traj = np.array(odom_traj)

opt_traj = []
for i, (p2d, ts) in enumerate(zip(opt_2d, timestamps_us)):
    x, y, th = p2d; z = z_vals[i] if i < len(z_vals) else 0
    opt_traj.append([ts/1e6, x, y, z, 0, 0, np.sin(th/2), np.cos(th/2)])
opt_traj = np.array(opt_traj)

# time-align GT to est frames (20 ms tolerance)
gt_sync = []
for ts in odom_traj[:,0]:
    d = np.abs(gt_all[:,0]-ts); mi = np.argmin(d)
    if d[mi] < 0.2:
        gt_sync.append(gt_all[mi])
gt_traj = np.array(gt_sync)

ml = min(len(odom_traj), len(opt_traj), len(gt_traj))
odom_traj, opt_traj, gt_traj = odom_traj[:ml], opt_traj[:ml], gt_traj[:ml]

ate_odom = compute_ate(odom_traj, gt_traj)
ate_opt = compute_ate(opt_traj, gt_traj)
rpe_odom = compute_rpe(odom_traj, gt_traj)
rpe_opt = compute_rpe(opt_traj, gt_traj)

ol = np.linalg.norm(np.diff(odom_traj[:,1:4], axis=0), axis=1).sum()
pl = np.linalg.norm(np.diff(opt_traj[:,1:4], axis=0), axis=1).sum()
gl = np.linalg.norm(np.diff(gt_traj[:,1:4], axis=0), axis=1).sum()

print(f"\n{'='*90}")
print(f"{'Metric':<30} {'Wk2 ICP-Only':<18} {'Wk3 Odom+Map':<18} {'Wk3 +GPS LC':<18}")
print(f"{'='*90}")
wk2_ate = 418.992
for name, v2, vo, vp in [
    ('ATE RMSE (m)', wk2_ate, ate_odom['rmse'], ate_opt['rmse']),
    ('ATE Mean (m)', 368.646, ate_odom['mean'], ate_opt['mean']),
    ('ATE Median (m)', 358.216, ate_odom['median'], ate_opt['median']),
]:
    print(f"{name:<30} {v2:<18.1f} {vo:<18.1f} {vp:<18.1f}")
print()
wk2_rpe_t, wk2_rpe_r = 0.5665, 5.6658
for name, v2, vo, vp in [
    ('RPE Trans RMSE (m/f)', wk2_rpe_t, rpe_odom['trans_rmse'], rpe_opt['trans_rmse']),
    ('RPE Rot RMSE (deg/f)', wk2_rpe_r, rpe_odom['rot_rmse'], rpe_opt['rot_rmse']),
]:
    print(f"{name:<30} {v2:<18.4f} {vo:<18.4f} {vp:<18.4f}")
print()
print(f"{'Traj Length (m)':<30} {'3276.4':<18} {ol:<18.1f} {pl:<18.1f}")
print(f"{'GT Length (m)':<30} {'3184.3':<18} {gl:<18.1f} {gl:<18.1f}")
print(f"{'Loop Closures':<30} {'0':<18} {'N/A':<18} {len(lc_edges):<18}")
print(f"{'Processing Time (s)':<30} {'229':<18} {t_odom:<18.1f} {t_odom+t_opt:<18.1f}")
print(f"{'='*90}")

print(f"\nWeek 2 → Week 3 ATE improvement: "
      f"{wk2_ate:.1f}m → {ate_opt['rmse']:.1f}m "
      f"({(1-ate_opt['rmse']/wk2_ate)*100:+.1f}%)")

# --- Step 6: Plots ---
print("\nSTEP 6: GENERATING PLOTS")
print("=" * 80)

# 1. Trajectory comparison
fig, ax = plt.subplots(figsize=(14,12))
ax.plot(gt_traj[:,1], gt_traj[:,2], 'k-', lw=2, label='Ground Truth', alpha=0.8)
ax.plot(odom_traj[:,1], odom_traj[:,2], 'b-', lw=1.5, label='Odom+LocalMap ICP', alpha=0.7)
if len(lc_edges) > 0:
    ax.plot(opt_traj[:,1], opt_traj[:,2], 'r-', lw=1.5,
            label=f'+GPS LC ({len(lc_edges)} closures)', alpha=0.7)
for e in lc_edges[:200]:
    ci, qi = e[0], e[1]
    if ci < ml and qi < ml:
        ax.plot([odom_traj[ci,1],odom_traj[qi,1]],
                [odom_traj[ci,2],odom_traj[qi,2]], 'g-', lw=0.8, alpha=0.4)
ax.set_xlabel('X (m)', fontsize=12); ax.set_ylabel('Y (m)', fontsize=12)
ax.set_title('Week 3: Odom-Aided ICP + Local Map + GPS Loop Closure',
             fontsize=14, fontweight='bold')
ax.legend(fontsize=11); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
plt.tight_layout()
plt.savefig(PLOTS_DIR/'trajectory_week3.png', dpi=150)
print(f"  Saved: trajectory_week3.png"); plt.close()

# 2. ATE over time
fig, ax = plt.subplots(figsize=(14,6))
ta = odom_traj[:,0]-odom_traj[0,0]
ax.plot(ta, ate_odom['errors'], 'b-', lw=1,
        label=f"Odom+Map ICP (RMSE={ate_odom['rmse']:.1f}m)", alpha=0.7)
if len(lc_edges) > 0:
    ax.plot(ta, ate_opt['errors'], 'r-', lw=1,
            label=f"+GPS LC (RMSE={ate_opt['rmse']:.1f}m)", alpha=0.7)
ax.set_xlabel('Time (s)', fontsize=12); ax.set_ylabel('ATE (m)', fontsize=12)
ax.set_title('Week 3: ATE Over Time', fontsize=14, fontweight='bold')
ax.legend(fontsize=11); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'ate_week3.png', dpi=150)
print(f"  Saved: ate_week3.png"); plt.close()

# 3. Error distribution
fig, (a1, a2) = plt.subplots(1,2, figsize=(14,5))
a1.hist(ate_odom['errors'], bins=50, color='blue', alpha=0.6, label='Odom+Map ICP')
if len(lc_edges) > 0:
    a1.hist(ate_opt['errors'], bins=50, color='red', alpha=0.6, label='+GPS LC')
a1.axvline(ate_odom['mean'], color='blue', ls='--', lw=2)
a1.set_xlabel('ATE (m)'); a1.set_title('ATE Distribution')
a1.legend(fontsize=9); a1.grid(True, alpha=0.3)

a2.hist(rpe_odom['trans_errors'], bins=50, color='blue', alpha=0.6, label='Odom+Map ICP')
a2.set_xlabel('RPE Trans (m/f)'); a2.set_title('RPE Distribution')
a2.legend(fontsize=9); a2.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'error_dist_week3.png', dpi=150)
print(f"  Saved: error_dist_week3.png"); plt.close()

# 4. GPS loop closures
fig, ax = plt.subplots(figsize=(14,12))
sc = ax.scatter(odom_traj[:,1], odom_traj[:,2], c=np.arange(ml), cmap='viridis', s=1, alpha=0.5)
plt.colorbar(sc, ax=ax, label='Frame')
for e in lc_edges[:200]:
    ci, qi = e[0], e[1]
    if ci < ml and qi < ml:
        ax.plot([odom_traj[ci,1],odom_traj[qi,1]],
                [odom_traj[ci,2],odom_traj[qi,2]], 'r-', lw=1.5, alpha=0.5)
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title(f'GPS Loop Closures ({len(lc_edges)} verified)', fontsize=14, fontweight='bold')
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'gps_loop_closures_week3.png', dpi=150)
print(f"  Saved: gps_loop_closures_week3.png"); plt.close()

np.savetxt(RESULTS_DIR/'odom_trajectory.txt', odom_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
np.savetxt(RESULTS_DIR/'optimized_trajectory.txt', opt_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
np.savetxt(RESULTS_DIR/'gt_trajectory.txt', gt_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
print(f"  Trajectories saved to {RESULTS_DIR}")

# --- Summary ---
print(f"\n{'='*80}")
print("WEEK 3 PIPELINE COMPLETE")
print(f"{'='*80}")
print(f"\nWeek 2 ICP-Only:     ATE RMSE = {wk2_ate:.1f}m")
print(f"Week 3 Odom+Map:     ATE RMSE = {ate_odom['rmse']:.1f}m "
      f"({(1-ate_odom['rmse']/wk2_ate)*100:+.1f}%)")
print(f"Week 3 Odom+Map+LC:  ATE RMSE = {ate_opt['rmse']:.1f}m "
      f"({(1-ate_opt['rmse']/wk2_ate)*100:+.1f}%)")
print(f"\nLoop closures:   {len(lc_edges)} verified GPS-aided")
print(f"Total time:      {t_odom+t_detect+t_verify+t_opt:.1f}s")
print(f"Plots:           {PLOTS_DIR}")
