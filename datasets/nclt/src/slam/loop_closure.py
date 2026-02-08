"""
Loop closure detection and pose graph optimization.

Scan context descriptors (Kim & Kim 2018) for LC candidate search, FPFH+RANSAC
global registration for coarse pose, point-to-plane ICP for refinement, and a
sparse 2D pose graph solved with Gauss-Newton + Levenberg-Marquardt damping.
Error form per edge e = (i,j): || T_i^-1 . T_j . z_ij^-1 ||_w^2, linearised in
[dx, dy, dtheta]. First node is anchored by setting a huge H diagonal.
"""
import numpy as np
import open3d as o3d
from scipy import sparse
from scipy.sparse.linalg import spsolve


class ScanContext:
    """Rotation-invariant scan descriptor for LC detection.

    2D polar grid (rings x sectors) with max z per bin. Rotation invariance is
    achieved by sliding the sectors at query time (column roll). See Kim & Kim,
    "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within
    3D Point Cloud Map", IROS 2018.
    """

    def __init__(self, ns=60, nr=20, max_r=80.0):
        # ns=60 sectors -> 6 deg yaw bins; fine enough for NCLT but cheap to slide.
        # nr=20 rings out to 80m picks up buildings and tree lines on campus.
        self.ns, self.nr, self.max_r = ns, nr, max_r

    def compute(self, pts):
        """Scan context from Nx3 points, returns nr x ns matrix"""
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
        """Per-ring mean z. Used as a cheap pre-filter before the full descriptor check."""
        return desc.mean(axis=1)

    def distance_rot(self, s1, s2, n_shifts=20):
        """Rotation-invariant distance between two scan contexts, returns [0,1]"""
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


def compute_fpfh(pcd, voxel_size=0.5):
    """FPFH features for global registration.

    FPFH (Fast Point Feature Histograms, Rusu 2009) is faster than PPF and
    doesn't need a trained model, so it's a good fit for rough LC pose guess
    before ICP refinement.

    returns (downsampled_pcd, fpfh_features)
    """
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # normals over 2x voxel radius: large enough to be stable but keep local shape
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    # FPFH radius 5x voxel: enough neighbours for the histogram to stay discriminative
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    return pcd_down, fpfh


def global_registration(pcd_src, pcd_tgt, fpfh_src, fpfh_tgt, voxel_size=0.5):
    """RANSAC global registration via FPFH features

    returns Open3D registration result
    """
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
    """Refine with point-to-plane ICP after global registration"""
    result = o3d.pipelines.registration.registration_icp(
        pcd_src, pcd_tgt, distance, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=30, relative_fitness=1e-6, relative_rmse=1e-6))
    return result


class PoseGraphOptimizer2D:
    """Sparse 2D pose graph optimizer (Gauss-Newton + LM damping).

    State: stacked [x_i, y_i, theta_i] for all N nodes.
    Each edge contributes J, residual r; normal equations H dx = -J^T r with
    LM damping H + lam * diag(H). lam is adapted per iteration: halved on
    cost decrease, multiplied by 5 on increase (classic LM). First pose
    anchored to keep the system determined (gauge freedom).
    """

    def __init__(self, odom_w=1.0, lc_w=10.0, damping=1e-3):
        # LC edges weighted 10x higher than odom because they are metric
        # (ICP-verified) constraints and should pull harder than drift-prone odom.
        self.ow, self.lw, self.damping = odom_w, lc_w, damping

    @staticmethod
    def _adiff(a, b):
        """Angle diff normalized to [-pi, pi]"""
        return (a - b + np.pi) % (2*np.pi) - np.pi

    def optimize(self, poses, odom_e, lc_e, max_iter=50):
        """optimize pose graph, returns Nx3 optimized poses"""
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


def pose_to_2d(p4):
    """Extract (x, y, yaw) from 4x4 transform"""
    return np.array([p4[0,3], p4[1,3], np.arctan2(p4[1,0], p4[0,0])])


def rel2d(xi, yi, ti, xj, yj, tj):
    """Compute relative 2D transform between two poses"""
    c, s = np.cos(ti), np.sin(ti)
    return c*(xj-xi)+s*(yj-yi), -s*(xj-xi)+c*(yj-yi), (tj-ti+np.pi)%(2*np.pi)-np.pi
