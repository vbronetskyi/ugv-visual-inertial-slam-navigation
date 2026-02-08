"""
Odometry-aided ICP: wheel-odom prediction, sliding local map, ground removal.

Why: plain frame-to-frame ICP drifts on low-texture NCLT paths. Three fixes:
 1. init ICP from wheel-odom delta (so we start near the minimum, not at I)
 2. register scan-to-local-map (last N scans fused) so each frame sees more geometry
 3. drop the ground plane before ICP so the cost doesn't lock onto the flat dominant
    surface at the expense of real structure (walls, poles, tree trunks)

The file is named imu_fusion.py for historical reasons; in practice NCLT's MS25
IMU is too slow (~48 Hz) to preintegrate reliably, so we lean on wheel odometry
which is already at 100 Hz in the dataset.
"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from tqdm import tqdm


class OdometryPredictor:
    """Predict relative motion between LiDAR frames from wheel odometry.
    Uses NCLT's 100Hz integrated odom poses, interpolates for any timestamp.
    """

    def __init__(self, odom_df):
        self.utimes = odom_df['utime'].values
        self.x = odom_df['x'].values
        self.y = odom_df['y'].values
        self.z = odom_df['z'].values
        self.roll = odom_df['roll'].values
        self.pitch = odom_df['pitch'].values
        self.yaw = odom_df['yaw'].values

    def _interp_at(self, utime):
        """Interpolate odom pose at timestamp"""
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
        """relative 4x4 between two timestamps (T_end = T_start @ T_rel)"""
        p0 = self._interp_at(utime_start)
        p1 = self._interp_at(utime_end)
        T0 = self._pose_4x4(*p0)
        T1 = self._pose_4x4(*p1)
        return np.linalg.inv(T0) @ T1


class LocalMap:
    """Sliding-window local map for scan-to-map ICP.
    Keeps last N scans in global frame, merged + voxel-downsampled.
    """

    def __init__(self, window_size=20, voxel_size=0.5):
        self.window_size = window_size
        self.voxel_size = voxel_size
        self.entries = []
        self._cached_map = None
        self._cache_dirty = True

    def add_scan(self, points_local, pose_4x4):
        """add scan to local map (transforms from scan frame to global)"""
        pts_h = np.hstack([points_local, np.ones((len(points_local), 1))])
        pts_global = (pose_4x4 @ pts_h.T).T[:, :3]

        self.entries.append(pts_global)
        if len(self.entries) > self.window_size:
            self.entries.pop(0)
        self._cache_dirty = True

    def get_map_pcd(self):
        """Build local map as Open3D PointCloud, or None if empty"""
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


class GPSLoopClosureDetector:
    """GPS-based loop closure detection.

    Converts lat/lon to a local ENU-ish planar frame (small-angle flat-earth,
    good enough for NCLT's ~1 km campus radius), then finds pairs that are
    spatially close but separated in time (> min_gap frames). This is a cheap
    LC proposal step; we still verify each candidate with FPFH + ICP before
    adding to the pose graph.
    """

    def __init__(self, gps_df, min_gap=200, radius=15.0, dedup_window=50):
        # min_gap=200 frames (~40 s at 5 Hz): don't propose LC against recent
        # frames since those are just odometry continuation.
        # radius=15 m: larger -> more false matches; smaller -> miss real closures.
        # NCLT RTK GPS is ~10 cm so 15 m is well above noise floor.
        self.min_gap = min_gap
        self.radius = radius
        self.dedup_window = dedup_window

        utimes = gps_df.iloc[:, 0].values
        lats = gps_df.iloc[:, 3].values
        lons = gps_df.iloc[:, 4].values

        self.lat0 = lats[0]
        self.lon0 = lons[0]

        # flat-earth  projection: valid within a few km of the reference lat.        # For NCLT that's fine (campus is <2 km across).
        R_earth = 6371000.0
        self.gps_x = R_earth * (lons - self.lon0) * np.cos(self.lat0)
        self.gps_y = R_earth * (lats - self.lat0)
        self.gps_utimes = utimes

    def get_gps_position(self, utime):
        """interpolated GPS (x, y) in meters, or None if out of range"""
        idx = np.searchsorted(self.gps_utimes, utime)
        if idx <= 0 or idx >= len(self.gps_utimes):
            return None

        t0, t1 = self.gps_utimes[idx-1], self.gps_utimes[idx]
        alpha = (utime - t0) / (t1 - t0) if t1 != t0 else 0.0

        x = self.gps_x[idx-1] + alpha * (self.gps_x[idx] - self.gps_x[idx-1])
        y = self.gps_y[idx-1] + alpha * (self.gps_y[idx] - self.gps_y[idx-1])
        return x, y

    def find_candidates(self, timestamps):
        """Find LC candidates from GPS proximity, returns list of (query_idx, candidate_idx, gps_distance)"""
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

        dedup = {}
        for q, c, d in candidates:
            wq, wc = q // self.dedup_window, c // self.dedup_window
            key = (wc, wq)
            if key not in dedup or d < dedup[key][2]:
                dedup[key] = (q, c, d)

        return list(dedup.values())


def remove_ground(pcd, distance_threshold=0.25, ransac_n=3, num_iterations=200):
    """Remove ground plane via RANSAC. Only strips the plane if its normal
    is roughly vertical (|n_z| > 0.7) so we don't accidentally delete a wall
    when the robot is on a tilted surface. 0.25 m distance threshold tolerates
    NCLT's curbs and slight slope without eating into real vertical objects.
    """
    if len(pcd.points) < 100:
        return pcd

    try:
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=ransac_n,
            num_iterations=num_iterations)

        # plane: [a, b, c, d] with ax+by+cz+d=0, normal is (a,b,c).
        # abs(n_z) > 0.7 means mostly horizontal plane i.e. likely the ground.
        normal = np.array(plane_model[:3])
        if abs(normal[2]) > 0.7:
            non_ground = pcd.select_by_index(inliers, invert=True)
            return non_ground
    except Exception:
        # RANSAC can throw if pcd is too sparse. Fall through and return original.
        pass

    return pcd
