"""
Custom ICP-based LiDAR odometry.

Point-to-plane ICP via Open3D for frame-to-frame pose estimation.
Error minimised: sum_i ((T p_i - q_i) . n_i)^2, where n_i is the target normal.
Point-to-plane >> point-to-point on outdoor NCLT because most NCLT surfaces
(ground, building walls) are locally planar.
"""
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


class CustomICP:
    """Frame-to-frame ICP odometry via Open3D point-to-plane.

    Pipeline per scan: voxel downsample -> normal estimation -> ICP against
    previous (downsampled) scan. Global pose is accumulated by right-multiplying
    the per-frame transform: T_k = T_{k-1} . delta.
    """

    def __init__(self, voxel_size=0.3, threshold=1.5, max_iter=100):
        # voxel 0.3m: keeps walls and poles, drops grass noise. Smaller (0.1m)
        # is 10x slower and doesn't help ICP; larger (0.5m) starts losing pole/edge features.
        self.voxel_size = voxel_size
        # max correspondence distance. 1.5m caps on NCLT because the Segway
        # moves <0.5m/frame at 10Hz so anything wider matches wrong ground points.
        self.threshold = threshold
        self.max_iter = max_iter
        # max_iter=100 felt generous. 80 also works, 50 starts to leave residual.
        self.poses = []
        self.timestamps = []
        self.current_pose = np.eye(4)
        self.prev_pcd = None

    def register_scan(self, points, timestamp, init_transform=None):
        """register new scan, update trajectory, returns current 4x4 global pose"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        # normals  needed for point-to-plane error. Hybrid KD-tree: 1m radius or 30 nn,        # whichever is reached first. For HDL-32E outdoor scans this gives smooth normals
        # on walls without over-smoothing edges.
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))

        if self.prev_pcd is None:
            # first scan: identity pose, nothing to register against
            self.poses.append(self.current_pose.copy())
            self.timestamps.append(timestamp)
            self.prev_pcd = pcd
            return self.current_pose

        if init_transform is None:
            # cold start with identity. In practice the caller passes IMU- or
            # constant-velocity-predicted delta which dramatically improves convergence.
            init_transform = np.eye(4)

        reg = o3d.pipelines.registration.registration_icp(
            pcd, self.prev_pcd, self.threshold, init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.max_iter,
                relative_fitness=1e-6, relative_rmse=1e-6))

        # accumulate: global pose = prev_global . delta_(k-1 -> k)
        self.current_pose = self.current_pose @ reg.transformation
        self.poses.append(self.current_pose.copy())
        self.timestamps.append(timestamp)
        self.prev_pcd = pcd

        return self.current_pose

    def get_trajectory(self):
        """Nx8 array [timestamp_s, x, y, z, qx, qy, qz, qw]"""
        traj = []
        for timestamp, pose in zip(self.timestamps, self.poses):
            pos = pose[:3, 3]
            quat = Rotation.from_matrix(pose[:3, :3]).as_quat()
            traj.append([timestamp / 1e6, pos[0], pos[1], pos[2],
                         quat[0], quat[1], quat[2], quat[3]])
        return np.array(traj)
