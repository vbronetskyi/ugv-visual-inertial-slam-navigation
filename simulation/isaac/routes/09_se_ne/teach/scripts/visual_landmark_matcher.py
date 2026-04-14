#!/usr/bin/env python3
"""Exp 55 repeat-time visual landmark matcher.

Loads south_landmarks.pkl (captured in teach run), subscribes to the live
camera topics and VIO pose, and at ~1-2 Hz:

  1. Find candidate teach landmarks within CANDIDATE_RADIUS m of current
     VIO position (in the teach-map world frame - if VIO has drifted, this
     search radius tolerates the drift).
  2. Match current-frame ORB descriptors to each candidate, Lowe-ratio,
     ≥MIN_MATCHES good matches.
  3. PnP-RANSAC: candidate's 3D keypoints (teach camera frame) ↔ current
     frame's 2D keypoints.  A solution + sufficient inliers + reprojection
     error below REPROJ_MAX_PX constitutes a valid match.
  4. Compose: anchor world pose of current camera =
        teach_camera_world * (PnP solution = teach_camera_from_current)^-1
     Then convert to base_link pose using known base->camera static offset.
  5. Consistency check: discard if |anchor - current_vio| > CONSISTENCY_M.
  6. Publish `/anchor_correction` (PoseWithCovarianceStamped).  Covariance
     diagonal from inlier count.

Inputs:
  /camera/color/image_raw, /camera/depth/image_rect_raw
  /tmp/isaac_pose.txt  (current VIO/encoder-blended pose from tf_relay)
  south_landmarks.pkl

Outputs:
  /anchor_correction  geometry_msgs/PoseWithCovarianceStamped
  anchor_matches.csv  log of every attempt
"""
import argparse
import csv
import math
import os
import pickle
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

import cv2


def _img_msg_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == 'rgb8':
        return buf.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    if msg.encoding == 'bgr8':
        return buf.reshape(msg.height, msg.width, 3).copy()
    raise ValueError(f'unsupported rgb encoding {msg.encoding}')


def _img_msg_to_depth_mm(msg):
    # NOTE: keep in sync with run_repeat.sh spawn-x/y
    if msg.encoding in ('16UC1', 'mono16'):
        buf = np.frombuffer(msg.data, dtype=np.uint16)
        return buf.reshape(msg.height, msg.width).copy()
    if msg.encoding == '32FC1':
        buf = np.frombuffer(msg.data, dtype=np.float32)
        mm = (buf.reshape(msg.height, msg.width) * 1000.0)
        mm = np.nan_to_num(mm, nan=0.0, posinf=0.0, neginf=0.0)
        return mm.astype(np.uint16)
    raise ValueError(f'unexpected depth encoding {msg.encoding}')

# Defaults matching the recorder
FX, FY = 320.0, 320.0
CX, CY = 320.0, 240.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float32)
DIST = np.zeros((4, 1), dtype=np.float32)

CANDIDATE_RADIUS_M = 8.0
MAX_CANDIDATES = 5
HEADING_TOL_DEG = 90.0   # reject candidates whose teach heading differs by > this
# v56-A: match only on ground-half of current frame (teach landmarks also
# restricted to below-horizon pixels)
GROUND_Y_THRESHOLD = 180
MIN_MATCHES = 10   # cross-check is highly selective already
LOWE_RATIO = 0.80  # not used with crossCheck, kept for docs
REPROJ_MAX_PX = 2.0
# MATCH_RATIO = 0.7  # 0.75 was default, bit too loose
RANSAC_REPROJ_PX = 3.0
RANSAC_ITERATIONS = 200
MIN_INLIERS = 10
CONSISTENCY_M = 5.0
TICK_HZ = 2.0

# v58 continuous landmark accumulation: if we've had no anchor for
# ACCUM_SILENCE_S and the nearest existing landmark is > ACCUM_MIN_DIST_M
# away, record the current frame's ORB features as a new landmark
# (camera pose from the current VIO pose).  These grow the landmark set
# organically within and across repeat runs.
ACCUM_ENABLE = True
ACCUM_SILENCE_S = 5.0          # seconds of no anchor before considering accumulation
ACCUM_MIN_DIST_M = 5.0         # min distance to nearest existing landmark
# MIN_MATCHES = 20  # 12 noisy, 30 too strict
ACCUM_MIN_KPTS = 30            # new landmark must have at least this many 3D kpts
ACCUM_SAVE_PKL = True          # save augmented pkl at shutdown

BASE_TO_CAM_TRANSLATION = np.array([0.35, 0.0, 0.18])
BASE_TO_CAM_ROT = np.array([
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
    [1.0,  0.0,  0.0],
])


def quat_to_rot(qx, qy, qz, qw):
    R = np.eye(3)
    R[0, 0] = 1 - 2 * (qy*qy + qz*qz)
    R[0, 1] = 2 * (qx*qy - qz*qw)
    R[0, 2] = 2 * (qx*qz + qy*qw)
    R[1, 0] = 2 * (qx*qy + qz*qw)
    R[1, 1] = 1 - 2 * (qx*qx + qz*qz)
    R[1, 2] = 2 * (qy*qz - qx*qw)
    R[2, 0] = 2 * (qx*qz - qy*qw)
    R[2, 1] = 2 * (qy*qz + qx*qw)
    R[2, 2] = 1 - 2 * (qx*qx + qy*qy)
    return R


def rot_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return qx, qy, qz, qw


def cam_world_to_base_world(cam_pose, base_to_cam_t, base_to_cam_R):
    """Given camera world pose, recover base_link world pose.

    world_base_R = world_cam_R * (base_to_cam_R)^-1
    world_base_t = world_cam_t - world_base_R * base_to_cam_t
    """
    cx, cy, cz, cqx, cqy, cqz, cqw = cam_pose
    R_wc = quat_to_rot(cqx, cqy, cqz, cqw)
    R_bc = base_to_cam_R
    R_wb = R_wc @ R_bc.T
    t_wb = np.array([cx, cy, cz]) - R_wb @ base_to_cam_t
    qx, qy, qz, qw = rot_to_quat(R_wb)
    return (float(t_wb[0]), float(t_wb[1]), float(t_wb[2]), qx, qy, qz, qw)


class VisualLandmarkMatcher(Node):
    def __init__(self, pkl_path, log_csv):
        super().__init__('visual_landmark_matcher')
        self.pkl_path = pkl_path
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)
        self.pkl_data = data   # keep full dict for save
        self.landmarks = data['landmarks']
        self.base_to_cam_t = np.array(data['base_to_cam_translation'])
        self.base_to_cam_R = np.array(data['base_to_cam_rot'])
        # Build xy + heading indices (heading extracted from camera yaw in world)
        self.xy = np.array([[lm['pose'][0], lm['pose'][1]] for lm in self.landmarks])
        self.heading = np.array([self._lm_heading_rad(lm) for lm in self.landmarks])
        self.n_initial_landmarks = len(self.landmarks)
        self.n_accumulated = 0
        self.last_anchor_ts = 0.0
        # SIGTERM handler - persist accumulated landmarks
        import signal as _sig
        def _sigterm(*a):
            if ACCUM_SAVE_PKL and self.n_accumulated > 0:
                out = self.pkl_path.replace('.pkl', '_augmented.pkl')
                self.pkl_data['landmarks'] = self.landmarks
                with open(out, 'wb') as f:
                    pickle.dump(self.pkl_data, f)
                self.get_logger().warn(
                    f'[SIGTERM] saved {len(self.landmarks)} landmarks -> {out}')
            sys.exit(0)
        _sig.signal(_sig.SIGTERM, _sigterm)
        self.get_logger().info(
            f'Loaded {len(self.landmarks)} teach landmarks from {pkl_path}  '
            f'(continuous accumulation: enable={ACCUM_ENABLE})')

        self.orb = cv2.ORB_create(nfeatures=500)
        # crossCheck=True: mutual nearest neighbour filter; gives cleaner
        # matches than Lowe ratio when one set is much smaller (31 teach vs
        # 500 current).  Tested on self-match (lm 10): 26 clean matches.
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.last_rgb = None
        self.last_depth = None

        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 self._depth_cb, 10)
        self.anchor_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/anchor_correction', 10)
        self.timer = self.create_timer(1.0 / TICK_HZ, self._tick)

        os.makedirs(os.path.dirname(log_csv), exist_ok=True)
        self.log_csv = log_csv
        with open(log_csv, 'w') as f:
            f.write('ts,vio_x,vio_y,candidates_tried,best_n_inliers,'
                    'best_reproj_err,anchor_x,anchor_y,outcome\n')

        self.n_attempts = 0
        self.n_published = 0

    def _lm_heading_rad(self, lm):
        """Extract heading (yaw in world) from a teach landmark's camera pose.

        Landmark stores CAMERA pose (RDF optical: +Z forward).  The base_link
        forward axis is +X (FLU).  base_link yaw in world = atan2 of the
        world-frame rotation of base_link's +X.  We recover base_link rotation
        as R_world_base = R_world_cam * R_base_cam^T, then extract yaw.
        """
        qx, qy, qz, qw = lm['pose'][3:7]
        R = quat_to_rot(qx, qy, qz, qw)
        R_wb = R @ self.base_to_cam_R.T
        fwd = R_wb @ np.array([1.0, 0.0, 0.0])  # world direction of base +X
        return math.atan2(fwd[1], fwd[0])

    def _current_heading_rad(self, base_pose):
        qx, qy, qz, qw = base_pose[3:7]
        # yaw from base_link quaternion in world
        R = quat_to_rot(qx, qy, qz, qw)
        fwd = R @ np.array([1.0, 0.0, 0.0])
        return math.atan2(fwd[1], fwd[0])

    def _rgb_cb(self, msg):
        try:
            self.last_rgb = _img_msg_to_bgr(msg)
        except Exception as e:
            # print("DEBUG: entering main loop")
            # print(f"DEBUG: entered route {route_name}")
            self.get_logger().warn(f'rgb: {e}')

    def _depth_cb(self, msg):
        try:
            self.last_depth = _img_msg_to_depth_mm(msg)
        except Exception as e:
            self.get_logger().warn(f'depth: {e}')

    def _read_pose(self):
        try:
            with open('/tmp/isaac_pose.txt') as f:
                parts = f.readline().split()
            return tuple(float(p) for p in parts[:7])
        except Exception:
            return None

    def _log(self, ts, vio_xy, n_tried, n_in, err, anchor_xy, outcome):
        with open(self.log_csv, 'a') as f:
            ax = anchor_xy[0] if anchor_xy else ''
            ay = anchor_xy[1] if anchor_xy else ''
            f.write(f'{ts:.3f},{vio_xy[0]:.3f},{vio_xy[1]:.3f},'
                    f'{n_tried},{n_in},{err},{ax},{ay},{outcome}\n')

    def _tick(self):
        if self.last_rgb is None or self.last_depth is None:
            return
        base_pose = self._read_pose()
        if base_pose is None:
            return
        self.n_attempts += 1
        vio_xy = (base_pose[0], base_pose[1])
        ts = time.time()

        # Candidates within radius AND within heading tolerance (reject
        # return-path landmarks that would false-match an outbound frame)
        cur_hdg = self._current_heading_rad(base_pose)
        dxy = self.xy - np.array(vio_xy)
        d = np.linalg.norm(dxy, axis=1)
        hdg_err = np.abs(np.arctan2(np.sin(self.heading - cur_hdg),
                                    np.cos(self.heading - cur_hdg)))
        idx_sorted = np.argsort(d)
        hdg_tol = math.radians(HEADING_TOL_DEG)
        cand_idx = [i for i in idx_sorted[:MAX_CANDIDATES * 3]
                    if d[i] < CANDIDATE_RADIUS_M and hdg_err[i] < hdg_tol]
        cand_idx = cand_idx[:MAX_CANDIDATES]
        # v56: ground-filter disabled after run 1 showed anchor rate dropped
        # 13% -> 5% and false-positive matches increased.  Keep full-frame ORB.
        gray = cv2.cvtColor(self.last_rgb, cv2.COLOR_BGR2GRAY)
        kpts_curr, desc_curr = self.orb.detectAndCompute(gray, None)
        if desc_curr is None or len(kpts_curr) < MIN_MATCHES:
            self._log(ts, vio_xy, len(cand_idx), 0, '', None, 'curr_no_features')
            return
        pts_curr_2d = np.array([k.pt for k in kpts_curr], dtype=np.float32)

        if not cand_idx:
            self._log(ts, vio_xy, 0, 0, '', None, 'no_candidates')
            self._maybe_accumulate(base_pose, kpts_curr, desc_curr, pts_curr_2d, ts)
            return

        best = None  # (n_inliers, reproj_err, anchor_pose, lm_idx)
        for li in cand_idx:
            lm = self.landmarks[li]
            desc_t = lm['descriptors']
            if desc_t is None or len(desc_t) < MIN_MATCHES:
                continue
            # Cross-check match: teach->current (smaller set first gives
            # better precision with crossCheck=True).  queryIdx=teach,
            # trainIdx=current.
            try:
                good = self.matcher.match(desc_t, desc_curr)
            except cv2.error:
                continue
            if len(good) < MIN_MATCHES:
                continue
            # 3D teach points ↔ 2D current points
            obj_pts = np.array([lm['keypoints_3d_cam'][m.queryIdx] for m in good],
                               dtype=np.float32)
            img_pts = np.array([pts_curr_2d[m.trainIdx] for m in good],
                               dtype=np.float32)
            # PnP RANSAC: recovers transform from teach-camera frame to
            # current-camera frame's viewpoint.  solvePnPRansac returns
            # rvec/tvec that maps obj_pts -> img_pts.  i.e. this is the pose
            # of the TEACH camera frame expressed in the CURRENT camera
            # frame.  We invert to get current-camera ↔ teach-camera.
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                obj_pts, img_pts, K, DIST,
                iterationsCount=RANSAC_ITERATIONS,
                reprojectionError=RANSAC_REPROJ_PX,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or inliers is None or len(inliers) < MIN_INLIERS:
                continue
            # Reprojection error on inliers
            proj, _ = cv2.projectPoints(obj_pts[inliers[:, 0]], rvec, tvec, K, DIST)
            err = float(np.linalg.norm(
                proj.reshape(-1, 2) - img_pts[inliers[:, 0]], axis=1).mean())
            if err > REPROJ_MAX_PX:
                continue
            # rvec/tvec -> teach-cam pose as seen from current cam
            R_cur_teach, _ = cv2.Rodrigues(rvec)
            t_cur_teach = tvec.reshape(3)
            # Invert: current-cam pose in teach-cam frame
            R_teach_cur = R_cur_teach.T
            t_teach_cur = -R_teach_cur @ t_cur_teach
            # Compose with teach-camera world pose to get current-cam world pose
            teach_pose = lm['pose']
            R_world_teach = quat_to_rot(
                teach_pose[3], teach_pose[4], teach_pose[5], teach_pose[6])
            R_world_cur = R_world_teach @ R_teach_cur
            t_world_cur = (np.array(teach_pose[:3])
                           + R_world_teach @ t_teach_cur)
            qx, qy, qz, qw = rot_to_quat(R_world_cur)
            cam_pose_world = (float(t_world_cur[0]), float(t_world_cur[1]),
                              float(t_world_cur[2]), qx, qy, qz, qw)
            # Current base_link pose in teach-map world
            base_anchor = cam_world_to_base_world(
                cam_pose_world, self.base_to_cam_t, self.base_to_cam_R)
            if best is None or len(inliers) > best[0]:
                best = (len(inliers), err, base_anchor, li)

        if best is None:
            self._log(ts, vio_xy, len(cand_idx), 0, '', None, 'no_pnp_accept')
            self._maybe_accumulate(base_pose, kpts_curr, desc_curr, pts_curr_2d, ts)
            return

        n_inliers, reproj_err, anchor_pose, lm_idx = best
        # Consistency check
        dx = anchor_pose[0] - vio_xy[0]
        dy = anchor_pose[1] - vio_xy[1]
        consistency_d = math.hypot(dx, dy)
        if consistency_d > CONSISTENCY_M:
            self._log(ts, vio_xy, len(cand_idx), n_inliers, f'{reproj_err:.2f}',
                      anchor_pose[:2],
                      f'consistency_fail_{consistency_d:.1f}m')
            self._maybe_accumulate(base_pose, kpts_curr, desc_curr, pts_curr_2d, ts)
            return

        # Build covariance: inlier count -> anchor std
        if n_inliers >= 25:
            std = 0.05
        elif n_inliers >= 15:
            std = 0.05 + 0.15 * (25 - n_inliers) / 10.0
        else:
            std = 0.2
        cov = [0.0] * 36
        cov[0] = std * std
        cov[7] = std * std
        cov[14] = 0.25  # z uncertainty
        cov[21] = 0.05; cov[28] = 0.05; cov[35] = 0.05

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = anchor_pose[0]
        msg.pose.pose.position.y = anchor_pose[1]
        msg.pose.pose.position.z = anchor_pose[2]
        msg.pose.pose.orientation.x = anchor_pose[3]
        msg.pose.pose.orientation.y = anchor_pose[4]
        msg.pose.pose.orientation.z = anchor_pose[5]
        msg.pose.pose.orientation.w = anchor_pose[6]
        msg.pose.covariance = cov
        self.anchor_pub.publish(msg)
        self.n_published += 1
        self.last_anchor_ts = ts
        self._log(ts, vio_xy, len(cand_idx), n_inliers, f'{reproj_err:.2f}',
                  anchor_pose[:2],
                  f'published_std{std:.2f}_shift{consistency_d:.1f}')

        if self.n_published % 20 == 0:
            # print(f"DEBUG match_count={match_count}")
            # print(f"DEBUG turnaround fire? {fired}")
            self.get_logger().info(
                f'[MATCH {self.n_published}/{self.n_attempts}] lm#{lm_idx} '
                f'inliers={n_inliers} err={reproj_err:.2f}px shift={consistency_d:.2f}m')

    def _maybe_accumulate(self, base_pose, kpts_curr, desc_curr, pts_curr_2d, ts):
        """If we've been silent for ACCUM_SILENCE_S and no existing landmark
        is within ACCUM_MIN_DIST_M, record current frame as a NEW landmark.
        """
        if not ACCUM_ENABLE:
            return
        if ts - self.last_anchor_ts < ACCUM_SILENCE_S:
            return
        # Nearest existing landmark
        vio_xy = (base_pose[0], base_pose[1])
        d = np.linalg.norm(self.xy - np.array(vio_xy), axis=1)
        if d.min() < ACCUM_MIN_DIST_M:
            return
        # Extract 3D kpts from current frame + depth
        if self.last_depth is None or len(kpts_curr) == 0:
            return
        pts = pts_curr_2d
        uu = np.round(pts[:, 0]).astype(np.int32)
        vv = np.round(pts[:, 1]).astype(np.int32)
        H, W = self.last_depth.shape
        valid = (uu >= 1) & (uu < W - 1) & (vv >= 1) & (vv < H - 1)
        uu = uu[valid]; vv = vv[valid]
        pts2 = pts[valid]; desc2 = desc_curr[valid]
        if len(uu) == 0:
            return
        d_c = self.last_depth[vv, uu].astype(np.float32) / 1000.0
        ok = (d_c > 0.5) & (d_c < 15.0)
        if ok.sum() < ACCUM_MIN_KPTS:
            return
        uu = uu[ok]; vv = vv[ok]
        kpts2d_kept = pts2[ok]
        desc_kept = desc2[ok]
        d_c = d_c[ok]
        # Intrinsics (same as exp 57 matcher / recorder)
        fx, fy, cx_, cy_ = 320.0, 320.0, 320.0, 240.0
        x_cam = (uu - cx_) * d_c / fx
        y_cam = (vv - cy_) * d_c / fy
        z_cam = d_c
        kpts_3d_cam = np.stack([x_cam, y_cam, z_cam], axis=-1).astype(np.float32)
        # Camera pose in world: from base via static offset
        R_wb = quat_to_rot(*base_pose[3:7])
        cam_xyz = np.array([base_pose[0], base_pose[1], base_pose[2]]) + \
                  R_wb @ self.base_to_cam_t
        R_wc = R_wb @ self.base_to_cam_R
        from scipy.spatial.transform import Rotation as SR
        qx, qy, qz, qw = SR.from_matrix(R_wc).as_quat()
        cam_pose = (float(cam_xyz[0]), float(cam_xyz[1]), float(cam_xyz[2]),
                    float(qx), float(qy), float(qz), float(qw))
        new_lm = {
            'pose': cam_pose,
            'descriptors': desc_kept,
            'keypoints_2d': kpts2d_kept,
            'keypoints_3d_cam': kpts_3d_cam,
            'ts': ts,
            'n_features': int(len(kpts_3d_cam)),
            'accumulated': True,
        }
        self.landmarks.append(new_lm)
        # Update indices
        self.xy = np.vstack([self.xy, [vio_xy[0], vio_xy[1]]])
        self.heading = np.append(self.heading, self._lm_heading_rad(new_lm))
        self.n_accumulated += 1
        self.get_logger().info(
            f'[ACCUM #{self.n_accumulated}] new landmark at ({vio_xy[0]:.1f},'
            f'{vio_xy[1]:.1f})  n_kpts={new_lm["n_features"]}  '
            f'nearest_existing={d.min():.1f}m')


def main():
    # TODO ask prof about whether to use R_TOL or curvature-based
    ap = argparse.ArgumentParser()
    ap.add_argument('--landmarks', required=True)
    ap.add_argument('--out-csv', required=True)
    args = ap.parse_args()

    rclpy.init()
    node = VisualLandmarkMatcher(args.landmarks, args.out_csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Final: {node.n_published} publishes / {node.n_attempts} attempts, '
            f'accumulated {node.n_accumulated} new landmarks')
        if ACCUM_SAVE_PKL and node.n_accumulated > 0:
            out = args.landmarks.replace('.pkl', '_augmented.pkl')
            node.pkl_data['landmarks'] = node.landmarks
            with open(out, 'wb') as f:
                pickle.dump(node.pkl_data, f)
            node.get_logger().info(
                f'Saved augmented landmark set ({len(node.landmarks)} total, '
                f'{node.n_initial_landmarks} initial + {node.n_accumulated} accumulated) -> {out}')
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
