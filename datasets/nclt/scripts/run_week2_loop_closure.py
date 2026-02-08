#!/usr/bin/env python3
"""ICP odometry + loop closure + pose graph (v3).

fixes from earlier versions:
- scan context only for LC candidate detection, never spatial proximity from
  the drifted trajectory (which gave loads of false positives)
- FPFH + RANSAC global registration verifies candidates without relying on
  the trajectory prior (handles arbitrary relative poses)
- hard dedup: at most 1 LC per 50-frame window pair
- LM damping in pose graph optimisation - plain Gauss-Newton diverges when
  outlier edges sneak in
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

SESSION = '2012-04-29'
SUBSAMPLE = 2
RESULTS_DIR = Path(__file__).resolve().parent.parent / 'results' / 'week2_icp_loop_closure'
PLOTS_DIR = RESULTS_DIR / 'plots'
PLOTS_DIR.mkdir(parents=True, exist_ok=True)

# scan context: 60 sectors x 20 rings out to 80 m. 6 deg yaw bins are fine
# enough for Segway motion but cheap to slide for rotation invariance.
SC_SECTORS = 60
SC_RINGS = 20
SC_MAX_RANGE = 80.0

# loop-closure gating
LC_MIN_GAP = 200              # do not close with recent frames (just odometry continuation)
LC_SC_THRESH = 0.35           # relaxed SC threshold (rotation-invariant cosine distance)
LC_CHECK_INTERVAL = 10        # check every 10 frames, not every frame
LC_RANSAC_DIST = 0.5          # FPFH + RANSAC correspondence distance
LC_RANSAC_FITNESS = 0.20      # min fitness for RANSAC global registration
LC_ICP_REFINE_DIST = 1.0      # ICP correspondence cap after RANSAC
LC_ICP_REFINE_FITNESS = 0.30  # min ICP fitness to accept the edge
LC_DEDUP_WINDOW = 50          # keep 1 closure per (window_i, window_j) pair

# pose graph weights: LC 5x odom here (softer than week3's 10x because
# week2's LC edges are SC/ICP-only, no RTK-GPS backing).
PG_ODOM_WEIGHT = 1.0
PG_LC_WEIGHT = 5.0
PG_DAMPING = 1e-3             # LM damping factor

print("=" * 80)
print("  ICP + LOOP CLOSURE + POSE GRAPH (v3 - FPFH+RANSAC)")
print("=" * 80)
print(f"Session: {SESSION}, every {SUBSAMPLE}nd scan")
print(f"LC: SC_thresh={LC_SC_THRESH}, RANSAC_fitness>{LC_RANSAC_FITNESS}, "
      f"ICP_fitness>{LC_ICP_REFINE_FITNESS}")
print(f"Dedup window: {LC_DEDUP_WINDOW} frames")
print("=" * 80)


# ============================================================================
# Scan context descriptor (Kim & Kim 2018), rotation-invariant.
# ============================================================================
class ScanContext:
    def __init__(self, ns=60, nr=20, max_r=80.0):
        self.ns, self.nr, self.max_r = ns, nr, max_r

    def compute(self, pts):
        xy, z = pts[:, :2], pts[:, 2]
        r = np.linalg.norm(xy, axis=1)
        a = np.arctan2(xy[:, 1], xy[:, 0]) + np.pi
        m = (r > 0.5) & (r < self.max_r)
        if m.sum() == 0:
            return np.zeros((self.nr, self.ns))
        r, a, z = r[m], a[m], z[m]
        ri = np.minimum((r / self.max_r * self.nr).astype(int), self.nr - 1)
        si = np.minimum((a / (2*np.pi) * self.ns).astype(int), self.ns - 1)
        desc = np.full((self.nr, self.ns), -100.0)
        np.maximum.at(desc, (ri, si), z)
        desc[desc == -100.0] = 0.0
        return desc

    def ring_key(self, desc):
        return desc.mean(axis=1)

    def distance_rot(self, s1, s2, n_shifts=20):
        v1 = s1.flatten()
        n1 = np.linalg.norm(v1)
        if n1 < 1e-6:
            return 1.0
        v1n = v1 / n1
        best = 1.0
        step = max(1, self.ns // n_shifts)
        for sh in range(0, self.ns, step):
            v2 = np.roll(s2, sh, axis=1).flatten()
            n2 = np.linalg.norm(v2)
            if n2 < 1e-6:
                continue
            best = min(best, 1.0 - np.dot(v1n, v2 / n2))
        return best


# ============================================================================
# FPFH + RANSAC Global Registration
# ============================================================================
def compute_fpfh(pcd, voxel_size=0.5):
    """Compute FPFH features for global registration"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return pcd_down, fpfh


def global_registration(pcd_src, pcd_tgt, fpfh_src, fpfh_tgt, voxel_size=0.5):
    """RANSAC-based global registration using FPFH features"""
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        pcd_src, pcd_tgt, fpfh_src, fpfh_tgt,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * 1.5,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def refine_registration(pcd_src, pcd_tgt, init_transform, distance=1.0):
    """Refine registration with ICP after global registration"""
    result = o3d.pipelines.registration.registration_icp(
        pcd_src, pcd_tgt, distance, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=30, relative_fitness=1e-6, relative_rmse=1e-6))
    return result


# ============================================================================
# Sparse 2D pose graph optimiser with Gauss-Newton + LM damping.
# ============================================================================
class PoseGraphOptimizer2D:
    def __init__(self, odom_w=1.0, lc_w=5.0, damping=1e-3):
        self.ow, self.lw, self.damping = odom_w, lc_w, damping

    @staticmethod
    def _adiff(a, b):
        return (a - b + np.pi) % (2*np.pi) - np.pi

    def optimize(self, poses, odom_e, lc_e, max_iter=50):
        n = len(poses)
        p = poses.copy()
        ne = len(odom_e) + len(lc_e)
        print(f"  Graph: {n} nodes, {len(odom_e)} odom, {len(lc_e)} LC edges")

        prev_cost = float('inf')
        lam = self.damping

        for it in range(max_iter):
            rows, cols, vals = [], [], []
            res = []
            ri = 0

            edges = [(e, self.ow) for e in odom_e] + [(e, self.lw) for e in lc_e]
            for (i, j, dxm, dym, dtm), w in edges:
                xi, yi, ti = p[i]
                xj, yj, tj = p[j]
                ct, st = np.cos(ti), np.sin(ti)
                dx, dy = xj - xi, yj - yi

                dxp = ct*dx + st*dy
                dyp = -st*dx + ct*dy
                dtp = self._adiff(tj, ti)

                res.extend([w*(dxp-dxm), w*(dyp-dym), w*(dtp-dtm)])

                ii, jj = 3*i, 3*j
                # jacobian row 0 (ex)
                rows.extend([ri]*5); cols.extend([ii, ii+1, ii+2, jj, jj+1])
                vals.extend([w*(-ct), w*(-st), w*(-st*dx+ct*dy), w*ct, w*st])
                # row 1 (ey)
                rows.extend([ri+1]*5); cols.extend([ii, ii+1, ii+2, jj, jj+1])
                vals.extend([w*st, w*(-ct), w*(-ct*dx-st*dy), w*(-st), w*ct])
                # row 2 (et)
                rows.extend([ri+2]*2); cols.extend([ii+2, jj+2])
                vals.extend([w*(-1.0), w*1.0])
                ri += 3

            J = sparse.csr_matrix((vals, (rows, cols)), shape=(3*ne, 3*n))
            e = np.array(res)
            cost = np.sum(e**2)

            H = J.T @ J
            b = -J.T @ e

            # lM damping: H + lambda * diag(H)
            diag_H = H.diagonal().copy()
            diag_H[diag_H < 1e-6] = 1e-6
            H_lm = H + sparse.diags(lam * diag_H)

            # fix first pose (anchor)
            for k in range(3):
                H_lm[k, :] = 0; H_lm[:, k] = 0; H_lm[k, k] = 1e6; b[k] = 0

            try:
                dx = spsolve(H_lm.tocsc(), b)
            except Exception as ex:
                print(f"    Solver failed at iter {it}: {ex}")
                lam *= 10
                continue

            if np.any(np.isnan(dx)):
                lam *= 10
                continue

            # trial update
            p_trial = p.copy()
            p_trial[:, 0] += dx[0::3]
            p_trial[:, 1] += dx[1::3]
            p_trial[:, 2] += dx[2::3]
            p_trial[:, 2] = (p_trial[:, 2] + np.pi) % (2*np.pi) - np.pi

            # check cost reduction
            if cost < prev_cost:
                lam = max(lam / 2, 1e-7)
            else:
                lam = min(lam * 5, 1e3)

            p = p_trial
            unorm = np.linalg.norm(dx)

            if it % 5 == 0:
                print(f"    Iter {it}: cost={cost:.1f}, |dx|={unorm:.4f}, lam={lam:.1e}")

            prev_cost = cost
            if unorm < 1e-4:
                print(f"  Converged at iter {it}")
                break

        print(f"  Final cost: {cost:.1f}")
        return p


# ============================================================================
# helpers
# ============================================================================
def pose_to_2d(p4):
    return np.array([p4[0,3], p4[1,3], np.arctan2(p4[1,0], p4[0,0])])

def rel2d(xi, yi, ti, xj, yj, tj):
    c, s = np.cos(ti), np.sin(ti)
    return c*(xj-xi)+s*(yj-yi), -s*(xj-xi)+c*(yj-yi), (tj-ti+np.pi)%(2*np.pi)-np.pi

def compute_ate(e, g):
    errs = np.linalg.norm(e[:,1:4]-g[:,1:4], axis=1)
    return {'mean':errs.mean(), 'rmse':np.sqrt((errs**2).mean()),
            'std':errs.std(), 'median':np.median(errs),
            'min':errs.min(), 'max':errs.max(), 'errors':errs}

def compute_rpe(e, g, d=1):
    te, re = [], []
    for i in range(len(e)-d):
        def bT(r):
            T=np.eye(4); T[:3,3]=r[1:4]
            T[:3,:3]=Rotation.from_quat(r[4:8]).as_matrix(); return T
        Tgr = np.linalg.inv(bT(g[i])) @ bT(g[i+d])
        Ter = np.linalg.inv(bT(e[i])) @ bT(e[i+d])
        Tx = np.linalg.inv(Tgr) @ Ter
        te.append(np.linalg.norm(Tx[:3,3]))
        re.append(np.degrees(np.arccos(np.clip((np.trace(Tx[:3,:3])-1)/2, -1, 1))))
    te, re = np.array(te), np.array(re)
    return {'trans_rmse':np.sqrt((te**2).mean()), 'trans_mean':te.mean(),
            'rot_rmse':np.sqrt((re**2).mean()), 'rot_mean':re.mean(),
            'trans_errors':te, 'rot_errors':re}


# ============================================================================
# MAIN
# ============================================================================

# --- Load data ---
print("\nSTEP 0: LOADING DATA")
print("=" * 80)
gt_loader = GroundTruthLoader()
gt_df = gt_loader.load_ground_truth(SESSION)
gt_all = np.column_stack([
    gt_df['utime'].values/1e6, gt_df['x'].values, gt_df['y'].values,
    gt_df['z'].values, gt_df['qx'].values, gt_df['qy'].values,
    gt_df['qz'].values, gt_df['qw'].values])
print(f"GT: {len(gt_all)} poses")

loader = VelodyneLoader()
all_files = loader.get_velodyne_sync_files(SESSION)
files = all_files[::SUBSAMPLE]
print(f"Scans: {len(files)} (every {SUBSAMPLE}nd of {len(all_files)})")

# --- ICP Odometry + Scan Context ---
print("\nSTEP 1: ICP ODOMETRY + SCAN CONTEXT")
print("=" * 80)

sc = ScanContext(SC_SECTORS, SC_RINGS, SC_MAX_RANGE)
descriptors = []
ring_keys = []
poses_4x4 = []
timestamps = []
pcd_cache = {}  # Store every 10th for LC verification

current_pose = np.eye(4)
prev_pcd = None
t0 = time.time()

for idx in tqdm(range(len(files)), desc="ICP+SC"):
    fp = files[idx]
    pts = loader.load_velodyne_sync(fp)
    ts = int(Path(fp).stem)

    descriptors.append(sc.compute(pts[:, :3]))
    ring_keys.append(sc.ring_key(descriptors[-1]))

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3])
    pcd = pcd.voxel_down_sample(0.3)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))

    if idx % 10 == 0:
        pcd_cache[idx] = pcd

    if prev_pcd is not None:
        reg = o3d.pipelines.registration.registration_icp(
            pcd, prev_pcd, 1.5, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100, relative_fitness=1e-6, relative_rmse=1e-6))
        current_pose = current_pose @ reg.transformation

    poses_4x4.append(current_pose.copy())
    timestamps.append(ts)
    prev_pcd = pcd

    if (idx+1) % 500 == 0:
        el = time.time()-t0; p = current_pose[:3,3]
        print(f"\n  [{idx+1}/{len(files)}] {(idx+1)/el:.1f} sc/s | "
              f"Pos: [{p[0]:.0f},{p[1]:.0f}]")

t_icp = time.time()-t0
print(f"\nICP: {len(poses_4x4)} poses in {t_icp:.1f}s ({len(poses_4x4)/t_icp:.1f} sc/s)")
print(f"PCD cache: {len(pcd_cache)} clouds")

# --- Loop Closure Detection (Scan Context ONLY) ---
print("\nSTEP 2: LOOP CLOSURE DETECTION (Scan Context)")
print("=" * 80)

rk_mat = np.array(ring_keys)
candidates = []  # (query, candidate, sc_dist)
t0 = time.time()

for qi in tqdm(range(LC_MIN_GAP, len(files), LC_CHECK_INTERVAL), desc="SC detect"):
    se = qi - LC_MIN_GAP
    if se <= 0:
        continue

    # ring key pre-filter
    rk_d = np.linalg.norm(rk_mat[:se] - rk_mat[qi], axis=1)
    top_rk = np.argsort(rk_d)[:30]

    for ci in top_rk:
        if rk_d[ci] > 3.0:
            continue
        sd = sc.distance_rot(descriptors[qi], descriptors[ci], n_shifts=20)
        if sd < LC_SC_THRESH:
            candidates.append((qi, int(ci), sd))

t_detect = time.time()-t0
print(f"SC candidates: {len(candidates)} in {t_detect:.1f}s")

# deduplicate: 1 best per (window_i, window_j) pair
dedup = {}
for q, c, sd in candidates:
    wq, wc = q // LC_DEDUP_WINDOW, c // LC_DEDUP_WINDOW
    key = (wc, wq)  # ordered
    if key not in dedup or sd < dedup[key][2]:
        dedup[key] = (q, c, sd)

candidates = list(dedup.values())
print(f"After dedup ({LC_DEDUP_WINDOW}-frame windows): {len(candidates)} candidates")

# --- FPFH + RANSAC Verification ---
print("\nSTEP 3: FPFH + RANSAC VERIFICATION")
print("=" * 80)

def get_pcd(idx):
    if idx in pcd_cache:
        return pcd_cache[idx]
    nearest = min(pcd_cache.keys(), key=lambda k: abs(k-idx))
    if abs(nearest-idx) <= 5:
        return pcd_cache[nearest]
    pts = loader.load_velodyne_sync(files[idx])
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(pts[:,:3])
    p = p.voxel_down_sample(0.3)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(1.0, 30))
    return p

verified = []
t0 = time.time()

for q, c, sd in tqdm(candidates, desc="FPFH+RANSAC verify"):
    pcd_q = get_pcd(q)
    pcd_c = get_pcd(c)

    # FPFH features
    try:
        pcd_q_down, fpfh_q = compute_fpfh(pcd_q, voxel_size=1.0)
        pcd_c_down, fpfh_c = compute_fpfh(pcd_c, voxel_size=1.0)

        # RANSAC global registration
        ransac_result = global_registration(pcd_q_down, pcd_c_down,
                                            fpfh_q, fpfh_c, voxel_size=1.0)

        if ransac_result.fitness < LC_RANSAC_FITNESS:
            continue

        # ICP refinement
        icp_result = refine_registration(pcd_q, pcd_c,
                                         ransac_result.transformation,
                                         LC_ICP_REFINE_DIST)

        if icp_result.fitness >= LC_ICP_REFINE_FITNESS:
            verified.append((q, c, icp_result.transformation,
                           icp_result.fitness, icp_result.inlier_rmse, sd))
    except Exception as ex:
        continue  # Skip problematic scans

t_verify = time.time()-t0
print(f"\nVerified: {len(verified)} / {len(candidates)} in {t_verify:.1f}s")

if verified:
    print("\nTop loop closures:")
    for q, c, T, fit, rmse, sd in sorted(verified, key=lambda x: -x[3])[:15]:
        trans = np.linalg.norm(T[:3, 3])
        angle = np.degrees(np.arccos(np.clip((np.trace(T[:3,:3])-1)/2, -1, 1)))
        print(f"  {c:5d}<->{q:5d} (gap={q-c:5d}): fit={fit:.3f}, rmse={rmse:.3f}m, "
              f"trans={trans:.1f}m, rot={angle:.1f}deg [sc={sd:.3f}]")

# --- Pose Graph ---
print("\nSTEP 4: POSE GRAPH OPTIMIZATION")
print("=" * 80)

poses_2d = np.array([pose_to_2d(p) for p in poses_4x4])
z_vals = [p[2,3] for p in poses_4x4]

odom_edges = []
for i in range(len(poses_2d)-1):
    dx, dy, dt = rel2d(*poses_2d[i], *poses_2d[i+1])
    odom_edges.append((i, i+1, dx, dy, dt))

lc_edges = []
for q, c, T, fit, rmse, sd in verified:
    dx, dy = T[0,3], T[1,3]
    dt = np.arctan2(T[1,0], T[0,0])
    lc_edges.append((c, q, dx, dy, dt))

print(f"Odom edges: {len(odom_edges)}, LC edges: {len(lc_edges)}")

if len(lc_edges) > 0:
    opt = PoseGraphOptimizer2D(PG_ODOM_WEIGHT, PG_LC_WEIGHT, PG_DAMPING)
    t0 = time.time()
    opt_2d = opt.optimize(poses_2d, odom_edges, lc_edges)
    t_opt = time.time()-t0
    print(f"Optimization: {t_opt:.1f}s")
else:
    print("No loop closures. Using ICP-only.")
    opt_2d = poses_2d.copy()
    t_opt = 0

# --- Evaluate ---
print("\nSTEP 5: EVALUATION")
print("=" * 80)

icp_traj = []
for ts, pose in zip(timestamps, poses_4x4):
    p = pose[:3,3]; q = Rotation.from_matrix(pose[:3,:3]).as_quat()
    icp_traj.append([ts/1e6, p[0], p[1], p[2], q[0], q[1], q[2], q[3]])
icp_traj = np.array(icp_traj)

opt_traj = []
for i, (p2d, ts) in enumerate(zip(opt_2d, timestamps)):
    x, y, th = p2d; z = z_vals[i] if i < len(z_vals) else 0
    opt_traj.append([ts/1e6, x, y, z, 0, 0, np.sin(th/2), np.cos(th/2)])
opt_traj = np.array(opt_traj)

gt_sync = []
for ts in icp_traj[:,0]:
    d = np.abs(gt_all[:,0]-ts); mi = np.argmin(d)
    if d[mi] < 0.2:
        gt_sync.append(gt_all[mi])
gt_traj = np.array(gt_sync)

ml = min(len(icp_traj), len(opt_traj), len(gt_traj))
icp_traj, opt_traj, gt_traj = icp_traj[:ml], opt_traj[:ml], gt_traj[:ml]

ate_icp = compute_ate(icp_traj, gt_traj)
ate_opt = compute_ate(opt_traj, gt_traj)
rpe_icp = compute_rpe(icp_traj, gt_traj)
rpe_opt = compute_rpe(opt_traj, gt_traj)

il = np.linalg.norm(np.diff(icp_traj[:,1:4], axis=0), axis=1).sum()
ol = np.linalg.norm(np.diff(opt_traj[:,1:4], axis=0), axis=1).sum()
gl = np.linalg.norm(np.diff(gt_traj[:,1:4], axis=0), axis=1).sum()

# --- Comparison Table ---
print(f"\n{'='*81}")
print(f"{'Metric':<30} {'ICP-Only':<18} {'ICP+LC':<18} {'Change':<15}")
print(f"{'='*81}")
for name, vi, vo in [
    ('ATE RMSE (m)', ate_icp['rmse'], ate_opt['rmse']),
    ('ATE Mean (m)', ate_icp['mean'], ate_opt['mean']),
    ('ATE Median (m)', ate_icp['median'], ate_opt['median']),
    ('ATE Std (m)', ate_icp['std'], ate_opt['std']),
    ('ATE Max (m)', ate_icp['max'], ate_opt['max']),
]:
    pct = (vo-vi)/vi*100 if vi != 0 else 0
    print(f"{name:<30} {vi:<18.3f} {vo:<18.3f} {pct:+.1f}%")
print()
for name, vi, vo in [
    ('RPE Trans RMSE (m/f)', rpe_icp['trans_rmse'], rpe_opt['trans_rmse']),
    ('RPE Rot RMSE (deg/f)', rpe_icp['rot_rmse'], rpe_opt['rot_rmse']),
]:
    pct = (vo-vi)/vi*100 if vi != 0 else 0
    print(f"{name:<30} {vi:<18.4f} {vo:<18.4f} {pct:+.1f}%")
print()
print(f"{'Traj Length (m)':<30} {il:<18.1f} {ol:<18.1f} (GT: {gl:.1f})")
print(f"{'Loop Closures':<30} {'N/A':<18} {len(lc_edges):<18}")
print(f"{'Processing Time (s)':<30} {t_icp:<18.1f} {t_icp+t_opt:<18.1f}")
print(f"{'='*81}")

# --- Plots ---
print("\nSTEP 6: PLOTS")
print("=" * 80)

# 1. Trajectory comparison
fig, ax = plt.subplots(figsize=(14,12))
ax.plot(gt_traj[:,1], gt_traj[:,2], 'k-', lw=2, label='Ground Truth', alpha=0.8)
ax.plot(icp_traj[:,1], icp_traj[:,2], 'b-', lw=1.5, label='ICP-Only', alpha=0.7)
ax.plot(opt_traj[:,1], opt_traj[:,2], 'r-', lw=1.5,
        label=f'ICP+LC ({len(lc_edges)} closures)', alpha=0.7)
for e in lc_edges[:200]:
    ci, qi = e[0], e[1]
    if ci < ml and qi < ml:
        ax.plot([icp_traj[ci,1],icp_traj[qi,1]], [icp_traj[ci,2],icp_traj[qi,2]],
                'g-', lw=0.8, alpha=0.4)
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title('Trajectory: ICP-Only vs ICP+Loop Closure vs GT', fontsize=14, fontweight='bold')
ax.legend(fontsize=11); ax.grid(True, alpha=0.3); ax.set_aspect('equal')
plt.tight_layout()
plt.savefig(PLOTS_DIR/'trajectory_icp_vs_lc.png', dpi=150)
print(f"  Saved: trajectory_icp_vs_lc.png"); plt.close()

# 2. ATE over time
fig, ax = plt.subplots(figsize=(14,6))
ta = icp_traj[:,0]-icp_traj[0,0]
ax.plot(ta, ate_icp['errors'], 'b-', lw=1, label=f"ICP-Only (RMSE={ate_icp['rmse']:.1f}m)", alpha=0.7)
ax.plot(ta, ate_opt['errors'], 'r-', lw=1, label=f"ICP+LC (RMSE={ate_opt['rmse']:.1f}m)", alpha=0.7)
ax.set_xlabel('Time (s)'); ax.set_ylabel('ATE (m)')
ax.set_title('Absolute Trajectory Error Over Time', fontsize=14, fontweight='bold')
ax.legend(fontsize=11); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'ate_icp_vs_lc.png', dpi=150)
print(f"  Saved: ate_icp_vs_lc.png"); plt.close()

# 3. Error distributions
fig, (a1, a2) = plt.subplots(1,2, figsize=(14,5))
a1.hist(ate_icp['errors'], bins=50, color='blue', alpha=0.6, label='ICP-Only')
a1.hist(ate_opt['errors'], bins=50, color='red', alpha=0.6, label='ICP+LC')
a1.axvline(ate_icp['mean'], color='blue', ls='--', lw=2, label=f"ICP: {ate_icp['mean']:.0f}m")
a1.axvline(ate_opt['mean'], color='red', ls='--', lw=2, label=f"LC: {ate_opt['mean']:.0f}m")
a1.set_xlabel('ATE (m)'); a1.set_title('ATE Distribution'); a1.legend(fontsize=9); a1.grid(True, alpha=0.3)
a2.hist(rpe_icp['trans_errors'], bins=50, color='blue', alpha=0.6, label='ICP-Only')
a2.hist(rpe_opt['trans_errors'], bins=50, color='red', alpha=0.6, label='ICP+LC')
a2.set_xlabel('RPE Trans (m/f)'); a2.set_title('RPE Distribution'); a2.legend(fontsize=9); a2.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'error_distribution_icp_vs_lc.png', dpi=150)
print(f"  Saved: error_distribution_icp_vs_lc.png"); plt.close()

# 4. Loop closures on trajectory
fig, ax = plt.subplots(figsize=(14,12))
sc_plot = ax.scatter(icp_traj[:,1], icp_traj[:,2], c=np.arange(ml), cmap='viridis', s=1, alpha=0.5)
plt.colorbar(sc_plot, ax=ax, label='Frame')
for e in lc_edges[:200]:
    ci, qi = e[0], e[1]
    if ci < ml and qi < ml:
        ax.plot([icp_traj[ci,1],icp_traj[qi,1]], [icp_traj[ci,2],icp_traj[qi,2]],
                'r-', lw=1.5, alpha=0.5)
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
ax.set_title(f'Loop Closures ({len(lc_edges)} verified)', fontsize=14, fontweight='bold')
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(PLOTS_DIR/'loop_closures_map.png', dpi=150)
print(f"  Saved: loop_closures_map.png"); plt.close()

np.savetxt(RESULTS_DIR/'icp_only_trajectory.txt', icp_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
np.savetxt(RESULTS_DIR/'icp_lc_trajectory.txt', opt_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
np.savetxt(RESULTS_DIR/'gt_synced_trajectory.txt', gt_traj, fmt='%.6f',
           header='timestamp x y z qx qy qz qw')
print(f"\n  Trajectories saved to {RESULTS_DIR}")

# --- Summary ---
print(f"\n{'='*80}")
print("PIPELINE COMPLETE")
print(f"{'='*80}")
print(f"\nICP-Only:        ATE RMSE = {ate_icp['rmse']:.1f}m")
print(f"ICP+LoopClosure: ATE RMSE = {ate_opt['rmse']:.1f}m")
if ate_icp['rmse'] > 0:
    imp = (1 - ate_opt['rmse']/ate_icp['rmse'])*100
    print(f"Improvement:     {imp:+.1f}%")
print(f"\nLoop closures:   {len(lc_edges)} verified (from {len(candidates)} candidates)")
print(f"Total time:      {t_icp+t_detect+t_verify+t_opt:.1f}s")
print(f"Plots:           {PLOTS_DIR}")
