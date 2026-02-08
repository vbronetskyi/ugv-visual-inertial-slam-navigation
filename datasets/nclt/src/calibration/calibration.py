"""
NCLT calibration: body/velodyne/camera frame transforms

For accurate projections, download cam_params from NCLT site
Ref: https://github.com/aljosaosep/NCLT-dataset-tools
"""
import numpy as np
from scipy.spatial.transform import Rotation
from typing import Tuple


class NCLTCalibration:
    """Calibration manager for NCLT sensor transformations"""

    # body to velodyne (lidar3) [x, y, z, roll_deg, pitch_deg, yaw_deg]
    BODY_TO_LIDAR3 = np.array([0.035, 0.002, -1.23, -179.93, -0.23, 0.50])

    def __init__(self, calib_dir: str = None):
        """calib_dir: dir with K_cam*.csv, x_lb3_c*.csv (None = defaults)"""
        self.calib_dir = calib_dir
        self.camera_intrinsics = {}  # K matrices
        self.lidar_to_camera = {}     # Extrinsic transformations

        # load or init calibrations
        if calib_dir:
            self._load_calibration_files()
        else:
            self._initialize_default_calibrations()

    def _initialize_default_calibrations(self):
        """Approximate placeholder calibrations, not production-accurate"""
        # placeholder intrinsics (typical Ladybug3)
        # real values from K_cam[0-5].csv
        for cam_id in range(6):
            self.camera_intrinsics[cam_id] = np.array([
                [400.0, 0.0, 512.0],
                [0.0, 400.0, 384.0],
                [0.0, 0.0, 1.0]
            ])

            # placeholder extrinsic (lidar to camera)
            # real values from x_lb3_c[0-5].csv
            angle = cam_id * 60.0 * np.pi / 180.0  # cameras spaced 60 degrees
            self.lidar_to_camera[cam_id] = np.eye(4)
            self.lidar_to_camera[cam_id][:3, :3] = Rotation.from_euler('z', angle).as_matrix()

    def _load_calibration_files(self):
        """Load calibration files from dir"""
        import os
        if not os.path.exists(self.calib_dir):
            print(f"Warning: Calibration directory not found: {self.calib_dir}")
            self._initialize_default_calibrations()
            return

        try:
            for cam_id in range(6):
                # camera intrinsics
                k_file = os.path.join(self.calib_dir, f'K_cam{cam_id}.csv')
                if os.path.exists(k_file):
                    self.camera_intrinsics[cam_id] = np.loadtxt(k_file, delimiter=',')

                # lidar to camera extrinsics
                x_file = os.path.join(self.calib_dir, f'x_lb3_c{cam_id}.csv')
                if os.path.exists(x_file):
                    self.lidar_to_camera[cam_id] = np.loadtxt(x_file, delimiter=',')
        except Exception as e:
            print(f"Warning: Error loading calibration files: {e}")
            self._initialize_default_calibrations()

    @staticmethod
    def ssc_to_homo(ssc: np.ndarray) -> np.ndarray:
        """6-DOF [x,y,z,roll_deg,pitch_deg,yaw_deg] to 4x4 homogeneous"""
        # translation
        t = ssc[:3]

        # rotation (deg to rad)
        roll = np.radians(ssc[3])
        pitch = np.radians(ssc[4])
        yaw = np.radians(ssc[5])

        # ZYX Euler angles
        R = Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()

        # homogeneous transform
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T

    def get_body_to_lidar(self) -> np.ndarray:
        """4x4 body to Velodyne (lidar3)"""
        return self.ssc_to_homo(self.BODY_TO_LIDAR3)

    def get_lidar_to_camera(self, cam_id: int) -> np.ndarray:
        """4x4 Velodyne to camera transform"""
        if cam_id not in self.lidar_to_camera:
            raise ValueError(f"Invalid camera ID: {cam_id}. Must be 0-5")
        return self.lidar_to_camera[cam_id]

    def get_camera_intrinsics(self, cam_id: int) -> np.ndarray:
        """3x3 intrinsic matrix K for cam_id (0-5)"""
        if cam_id not in self.camera_intrinsics:
            raise ValueError(f"Invalid camera ID: {cam_id}. Must be 0-5")
        return self.camera_intrinsics[cam_id]

    def project_velodyne_to_camera(self, points: np.ndarray, cam_id: int,
                                   in_body_frame: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """Project 3D points to camera image, returns (Nx2 pixels, valid_mask)"""
        # homogeneous coords if needed
        if points.shape[1] == 3:
            points_homo = np.hstack([points, np.ones((len(points), 1))])
        else:
            points_homo = points

        # to lidar frame if needed
        if in_body_frame:
            T_body_to_lidar = self.get_body_to_lidar()
            points_homo = (np.linalg.inv(T_body_to_lidar) @ points_homo.T).T

        # to camera frame
        T_lidar_to_cam = self.get_lidar_to_camera(cam_id)
        points_cam = (np.linalg.inv(T_lidar_to_cam) @ points_homo.T).T

        # filter points in front of camera (z > 0)
        valid_mask = points_cam[:, 2] > 0

        # project to image plane
        K = self.get_camera_intrinsics(cam_id)
        points_2d = (K @ points_cam[:, :3].T).T

        # normalize by depth
        pixels = points_2d[:, :2] / points_2d[:, 2:3]

        return pixels, valid_mask


if __name__ == '__main__':
    # example usage
    calib = NCLTCalibration()

    print("NCLT Calibration Example\n")
    print("Body to Lidar transformation:")
    print(calib.get_body_to_lidar())

    print("\nCamera 0 intrinsics:")
    print(calib.get_camera_intrinsics(0))

    print("\nLidar to Camera 0 transformation:")
    print(calib.get_lidar_to_camera(0))

    # test projection
    test_points = np.array([
        [1.0, 0.0, 0.0],
        [2.0, 1.0, -0.5],
        [3.0, -1.0, 0.5]
    ])

    pixels, valid = calib.project_velodyne_to_camera(test_points, cam_id=0, in_body_frame=False)
    print(f"\nTest projection to Camera 0:")
    print(f"Points: {test_points}")
    print(f"Pixels: {pixels}")
    print(f"Valid: {valid}")
