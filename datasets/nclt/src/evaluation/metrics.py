"""
ATE and RPE trajectory metrics. trajectories are in TUM format:
[timestamp, x, y, z, qx, qy, qz, qw]

ATE = Absolute Trajectory Error, per-frame position L2 error after time sync.
RPE = Relative Pose Error, per-step delta-pose error (translation + rotation).
"""
import numpy as np
from scipy.spatial.transform import Rotation


def compute_ate(traj_est, traj_gt):
    """ATE: per-frame ||p_est - p_gt||_2. caller should pre-align Sim(3)
    (Umeyama) for monocular or SE(3) for metric systems.
    returns dict with mean/rmse/std/median/min/max + raw errors array"""
    errs = np.linalg.norm(traj_est[:, 1:4] - traj_gt[:, 1:4], axis=1)
    return {
        'mean': errs.mean(),
        'rmse': np.sqrt((errs**2).mean()),
        'std': errs.std(),
        'median': np.median(errs),
        'min': errs.min(),
        'max': errs.max(),
        'errors': errs
    }


def compute_rpe(traj_est, traj_gt, delta=1):
    """RPE over step size `delta` frames. formula per pair (i, i+delta):
        E_i = (T_gt_i^-1 T_gt_{i+d})^-1  (T_est_i^-1 T_est_{i+d})
    translation err = ||E_i.t||, rotation err = acos((tr(E_i.R) - 1) / 2)

    delta=1 gives frame-to-frame drift (local quality). larger delta (e.g. 10s worth of
    frames) reveals medium-horizon drift which ATE can hide via global alignment.
    """
    te, re = [], []
    for i in range(len(traj_est) - delta):
        def build_T(row):
            T = np.eye(4)
            T[:3, 3] = row[1:4]
            T[:3, :3] = Rotation.from_quat(row[4:8]).as_matrix()
            return T

        Tgr = np.linalg.inv(build_T(traj_gt[i])) @ build_T(traj_gt[i + delta])
        Ter = np.linalg.inv(build_T(traj_est[i])) @ build_T(traj_est[i + delta])
        Tx = np.linalg.inv(Tgr) @ Ter
        te.append(np.linalg.norm(Tx[:3, 3]))
        # clip handles floating-point overshoot on trace ~= 3 (zero rotation)
        re.append(np.degrees(
            np.arccos(np.clip((np.trace(Tx[:3, :3]) - 1) / 2, -1, 1))))

    te, re = np.array(te), np.array(re)
    return {
        'trans_rmse': np.sqrt((te**2).mean()),
        'trans_mean': te.mean(),
        'rot_rmse': np.sqrt((re**2).mean()),
        'rot_mean': re.mean(),
        'trans_errors': te,
        'rot_errors': re
    }


def sync_trajectories(est_traj, gt_all, tolerance=0.2):
    """align estimated + gt by timestamp (nearest-neighbour, 0.2s window).
    NCLT gt runs at ~150 Hz so any reasonable SLAM output finds a gt match.
    tolerance 0.2s: big enough for 5 Hz Ladybug3, tight enough to avoid jumps."""
    gt_sync = []
    est_sync = []
    for i, ts in enumerate(est_traj[:, 0]):
        d = np.abs(gt_all[:, 0] - ts)
        mi = np.argmin(d)
        if d[mi] < tolerance:
            gt_sync.append(gt_all[mi])
            est_sync.append(est_traj[i])

    return np.array(est_sync), np.array(gt_sync)
