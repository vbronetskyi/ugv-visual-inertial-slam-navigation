#!/usr/bin/env python3
"""Honest comparison: what ORB-SLAM3 actually estimated vs what really happened"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

RESULTS = Path('/workspace/datasets/nclt/results')
OUT = RESULTS / 'week0_orbslam3_proper/plots'


def load_traj(path):
    ts, xyz = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) < 4:
                continue
            try:
                ts.append(float(p[0]))
                xyz.append([float(p[1]), float(p[2]), float(p[3])])
            except:
                continue
    ts, xyz = np.array(ts), np.array(xyz)
    if ts[0] > 1e15:
        ts /= 1e9
    return ts, xyz


def umeyama(src, tgt):
    n = src.shape[0]
    mu_s, mu_t = src.mean(0), tgt.mean(0)
    sc, tc = src - mu_s, tgt - mu_t
    var_s = np.sum(sc**2) / n
    cov = tc.T @ sc / n
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    s = np.trace(np.diag(D) @ S) / var_s if var_s > 1e-10 else 1.0
    t = mu_t - s * R @ mu_s
    return (s * (R @ src.T).T) + t, s


gt_ts, gt_xyz = load_traj(RESULTS / 'week0_seasonal/2012-04-29/gt_trajectory.txt')
est_ts, est_xyz = load_traj(
    RESULTS / 'week0_orbslam3_proper/phase_b/B004_imu_v4/f_nclt.txt')

# match timestampsme, mg, mt = [], [], []
for i in range(len(est_ts)):
    j = np.argmin(np.abs(gt_ts - est_ts[i]))
    if np.abs(gt_ts[j] - est_ts[i]) < 0.5:
        me.append(est_xyz[i])
        mg.append(gt_xyz[j])
        mt.append(est_ts[i])
me, mg, mt = np.array(me), np.array(mg), np.array(mt)

# Sim(3) alignment for panel 3 (Umeyama 1991): monocular VIO has unknown
# metric scale, so we fit 7-DoF (R, t, s) least-squares to GT before scoring ATE.
aligned, sim3_scale = umeyama(me, mg)
errs_sim3 = np.linalg.norm(aligned[:, :2] - mg[:, :2], axis=1)
ate_sim3 = np.sqrt(np.mean(errs_sim3**2))

# path lengths: used to quantify the raw-VIO scale error vs GT
gt_path = np.sum(np.linalg.norm(np.diff(mg, axis=0), axis=1))
raw_path = np.sum(np.linalg.norm(np.diff(me, axis=0), axis=1))

# shift each trajectory so it starts at (0,0) for plotting side-by-side
gt_from0 = mg[:, :2] - mg[0, :2]
est_from0 = me[:, :2] - me[0, :2]
aligned_from0 = aligned[:, :2] - aligned[0, :2]
gt_from0_shifted = mg[:, :2] - mg[0, :2]  # same as gt_from0

# plot
fig, axes = plt.subplots(1, 3, figsize=(24, 9))
fig.suptitle('Experiment 0.6 (B004): What ORB-SLAM3 VIO estimated vs reality',
             fontsize=16, fontweight='bold')

# panel 1: ground truth
ax = axes[0]
ax.plot(gt_from0[:, 0], gt_from0[:, 1], 'k-', linewidth=2.5)
ax.plot(0, 0, 'go', markersize=12, zorder=10)
ax.plot(gt_from0[-1, 0], gt_from0[-1, 1], 'rs', markersize=10, zorder=10)
ax.set_title(f'Reality (Ground Truth)\nPath: {gt_path:.0f} m over {mt[-1]-mt[0]:.0f} s',
             fontsize=13, fontweight='bold')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# panel 2: raw VIO output
ax = axes[1]
ax.plot(est_from0[:, 0], est_from0[:, 1], '-', color='#E91E63', linewidth=2.5)
ax.plot(0, 0, 'go', markersize=12, zorder=10)
ax.plot(est_from0[-1, 0], est_from0[-1, 1], 's', color='#E91E63', markersize=10, zorder=10)
ax.set_title(f'ORB-SLAM3 VIO output (raw, no alignment)\n'
             f'Path: {raw_path:,.0f} m, scale error {raw_path/gt_path:.0f}x',
             fontsize=13, fontweight='bold', color='#E91E63')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

# panel 3: after Sim(3) alignment
ax = axes[2]
# both from common start
a0 = aligned[:, :2] - aligned[0, :2]
g0 = mg[:, :2] - mg[0, :2]
# pin starting point so panel is a fair overlay
a0_pinned = a0 + (g0[0] - a0[0])

ax.plot(g0[:, 0], g0[:, 1], 'k-', linewidth=2.5, label='Ground Truth')
ax.plot(a0_pinned[:, 0], a0_pinned[:, 1], '-', color='#E91E63', linewidth=2,
        label=f'VIO after Sim(3)\nATE = {ate_sim3:.1f} m')
ax.plot(0, 0, 'go', markersize=12, zorder=10)
ax.set_title(f'After Sim(3) alignment\n'
             f'(rescaled by {sim3_scale:.2e} to best-fit GT)',
             fontsize=13, fontweight='bold')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.legend(fontsize=11, loc='best')
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)

plt.tight_layout()
fig.savefig(OUT / 'exp06_vs_gt.png', dpi=150, bbox_inches='tight')
print(f"Saved: {OUT / 'exp06_vs_gt.png'}")
plt.close()
