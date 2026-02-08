"""
NCLT ground truth pose loader.

CSV layout per row: utime, x, y, z, qx, qy, qz (NO qw!). qw has to be
recovered from the unit-quaternion constraint:

    qw^2 + qx^2 + qy^2 + qz^2 = 1   =>   qw = sqrt(1 - qx^2 - qy^2 - qz^2)

numerical safety: due to float rounding the sum under the sqrt can go very
slightly negative, so we clamp at 0 with np.maximum. also the first row of
every groundtruth csv has NaN, easy to miss, drop it before computing.

Ref: https://robots.engin.umich.edu/nclt/
"""
import os
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation


class GroundTruthLoader:
    """loader for NCLT ground truth poses"""

    def __init__(self, data_root: str = '/workspace/nclt_data/ground_truth'):
        self.data_root = data_root

    def load_ground_truth(self, session: str) -> pd.DataFrame:
        """load GT poses, qw computed from unit quaternion constraint"""
        filepath = os.path.join(self.data_root, f'groundtruth_{session}.csv')
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Ground truth file not found: {filepath}")

        # CSV format: utime, x, y, z, qx, qy, qz
        df = pd.read_csv(filepath, header=None,
                        names=['utime', 'x', 'y', 'z', 'qx', 'qy', 'qz'],
                        skipinitialspace=True)

        # drop NaN rows (usually first row)
        df = df.dropna()
        df = df.reset_index(drop=True)

        # qw from unit quaternion constraint
        qw_squared = 1.0 - (df['qx']**2 + df['qy']**2 + df['qz']**2)
        df['qw'] = np.sqrt(np.maximum(qw_squared, 0))  # clamp for numerical safety

        # reorder columns to [utime, x, y, z, qw, qx, qy, qz]
        df = df[['utime', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']]

        return df

    def get_pose_at_timestamp(self, df: pd.DataFrame, utime: int,
                              tolerance_us: int = 100000) -> dict:
        """nearest pose within tolerance, returns dict or None"""
        # find closest timestamp
        time_diffs = np.abs(df['utime'].values - utime)
        min_idx = np.argmin(time_diffs)

        if time_diffs[min_idx] > tolerance_us:
            return None

        row = df.iloc[min_idx]
        return {
            'utime': row['utime'],
            'position': np.array([row['x'], row['y'], row['z']]),
            'quaternion': np.array([row['qw'], row['qx'], row['qy'], row['qz']]),
            'time_diff_us': time_diffs[min_idx]
        }

    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        # scipy uses [qx, qy, qz, qw] format
        q_scipy = np.array([q[1], q[2], q[3], q[0]])
        rot = Rotation.from_quat(q_scipy)
        return rot.as_matrix()

    def get_transformation_matrix(self, df: pd.DataFrame, index: int) -> np.ndarray:
        """4x4 homogeneous transform for pose at given index"""
        row = df.iloc[index]
        q = np.array([row['qw'], row['qx'], row['qy'], row['qz']])
        t = np.array([row['x'], row['y'], row['z']])

        T = np.eye(4)
        T[:3, :3] = self.quaternion_to_rotation_matrix(q)
        T[:3, 3] = t

        return T

    def compute_trajectory_length(self, df: pd.DataFrame) -> float:
        positions = df[['x', 'y', 'z']].values
        diffs = np.diff(positions, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return np.sum(distances)

    def compute_velocities(self, df: pd.DataFrame) -> np.ndarray:
        positions = df[['x', 'y', 'z']].values
        times = df['utime'].values / 1e6  # Convert to seconds

        velocities = np.zeros_like(positions)
        velocities[1:] = np.diff(positions, axis=0) / np.diff(times)[:, np.newaxis]

        return velocities
