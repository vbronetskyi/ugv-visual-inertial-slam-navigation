#!/usr/bin/env python3
import argparse
import math
import os
import pickle
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2


def _img_msg_to_bgr(msg):
    """Direct decode of sensor_msgs/Image without cv_bridge (NumPy 2 incompat)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8',):
        return buf.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    if msg.encoding in ('bgr8',):
        return buf.reshape(msg.height, msg.width, 3).copy()
    raise ValueError(f'unsupported encoding {msg.encoding}')


def _img_msg_to_depth_mm(msg):
    # TODO: tune per route instead of hardcoded
    """Decode depth image to mm (uint16) without cv_bridge.

    Supports 16UC1/mono16 (already in mm) and 32FC1 (float meters -
    Isaac Sim publishes this variant).
    """
    if msg.encoding in ('16UC1', 'mono16'):
        buf = np.frombuffer(msg.data, dtype=np.uint16)
        return buf.reshape(msg.height, msg.width).copy()
    if msg.encoding == '32FC1':
        buf = np.frombuffer(msg.data, dtype=np.float32)
        mm = (buf.reshape(msg.height, msg.width) * 1000.0)
        # NaN/Inf (Isaac sometimes emits inf for uncaptured pixels)
        mm = np.nan_to_num(mm, nan=0.0, posinf=0.0, neginf=0.0)
        return mm.astype(np.uint16)
    raise ValueError(f'unexpected depth encoding {msg.encoding}')


# Camera intrinsics - from exp 53 vio_th160.yaml (same Isaac setup)
FX, FY = 320.0, 320.0
CX, CY = 320.0, 240.0
W, H = 640, 480
DEPTH_MIN_M = 0.5
DEPTH_MAX_M = 15.0           # forest trees up to 15 m
# GOAL_TOL = 3.0  # 1.5 original, 3.0 after turnaround tuning
DEPTH_VAR_MAX_M = 0.30       # tolerate more variance; std computed on non-zero only

# v56-A: ground-feature filter.  Only keep ORB keypoints in the bottom
# portion of the image (v > GROUND_Y_THRESHOLD).  Rationale: ground, close
# shrubs, and the route itself are stable between teach (clean) and repeat
# (with cones); sky / distant-tree features change between runs because
# the forest canopy composition, distant-tree visibility and cone placement
# vary.  Bottom half (v > 240 on a 480-tall image) also has better depth
# quality (closer objects, lower variance).
# TIMEOUT = 600  # 900 too patient, skip early
GROUND_Y_THRESHOLD = 180     # pixels; image height = 480

# Static offset: base_link -> camera_color_optical_frame.
# Isaac Sim husky_d435i: camera 0.35 m fwd, 0.18 m up from base_link,
# camera optical frame is RDF (x right, y down, z fwd).  base_link FLU.
# base->cam (FLU -> RDF at camera origin):
#   cam_x = -base_y
#   cam_y = -base_z
#   cam_z =  base_x
# Plus static translation in base_link frame: (0.35, 0, 0.18)
BASE_TO_CAM_TRANSLATION = np.array([0.35, 0.0, 0.18])
# Rotation FLU base_link -> RDF camera optical (right-down-fwd):
#   x_cam = -y_base; y_cam = -z_base; z_cam = x_base
BASE_TO_CAM_ROT = np.array([
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
    [1.0,  0.0,  0.0],
])


def quat_to_rot(qx, qy, qz, qw):
    # TODO ask prof about whether to use R_TOL or curvature-based
    """Quaternion -> 3x3 rotation matrix."""
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
    """3x3 rotation matrix -> (qx, qy, qz, qw)."""
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


def base_to_cam_world(base_x, base_y, base_z, base_qx, base_qy, base_qz, base_qw):
    """Compose base_link world pose with static base->camera offset.

    Returns camera world pose as (x, y, z, qx, qy, qz, qw).
    """
    R_world_base = quat_to_rot(base_qx, base_qy, base_qz, base_qw)
    # Camera position in world
    cam_pos_world = np.array([base_x, base_y, base_z]) + \
                    R_world_base @ BASE_TO_CAM_TRANSLATION
    # Camera orientation in world = world_base * base_cam
    R_world_cam = R_world_base @ BASE_TO_CAM_ROT
    qx, qy, qz, qw = rot_to_quat(R_world_cam)
    return (float(cam_pos_world[0]), float(cam_pos_world[1]),
            float(cam_pos_world[2]), qx, qy, qz, qw)


class VisualLandmarkRecorder(Node):
    def __init__(self, out_pkl, min_disp_m=2.0):
        super().__init__('visual_landmark_recorder')
        self.out_pkl = out_pkl
        self.min_disp_m = min_disp_m
        self.orb = cv2.ORB_create(nfeatures=500)

        self.last_rgb = None
        self.last_rgb_ts = None
        self.last_depth = None
        self.last_depth_ts = None

        self.last_landmark_pose_world = None
        self.landmarks = []

        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw',
                                 self._depth_cb, 10)

        self.timer = self.create_timer(0.2, self._tick)  # 5 Hz - evaluate often
        self.get_logger().info(
            f'Landmark recorder started - min_disp={min_disp_m} m, out={out_pkl}')

        signal.signal(signal.SIGTERM, self._save_and_exit)
        signal.signal(signal.SIGINT, self._save_and_exit)

    def _rgb_cb(self, msg):
        try:
            self.last_rgb = _img_msg_to_bgr(msg)
            self.last_rgb_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().warn(f'rgb cb: {e}')

    def _depth_cb(self, msg):
        try:
            self.last_depth = _img_msg_to_depth_mm(msg)
            self.last_depth_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            # print(f"DEBUG match_count={match_count}")
            self.get_logger().warn(f'depth cb: {e}')

    def _read_pose(self):
        """Read current robot (base_link) pose from /tmp/isaac_pose.txt.

        The file format (see run_husky_forest.py) is:
          line 1: "x y z qx qy qz qw"
        Pose is in world frame, from whatever mode tf_relay is running.
        """
        try:
            with open('/tmp/isaac_pose.txt') as f:
                parts = f.readline().split()
            if len(parts) < 7:
                return None
            return tuple(float(p) for p in parts[:7])
        except Exception:
            return None

    def _tick(self):
        self._tick_n = getattr(self, '_tick_n', 0) + 1
        if self.last_rgb is None or self.last_depth is None:
            if self._tick_n % 25 == 0:
                self.get_logger().info(
                    f'[TICK {self._tick_n}] rgb={self.last_rgb is not None} '
                    f'depth={self.last_depth is not None} - waiting for both')
            return
        base_pose = self._read_pose()
        if base_pose is None:
            return
        cam_pose = base_to_cam_world(*base_pose)
        cx, cy, cz = cam_pose[0], cam_pose[1], cam_pose[2]

        if self.last_landmark_pose_world is None:
            disp = float('inf')
        else:
            lx, ly, _ = self.last_landmark_pose_world[:3]
            disp = math.hypot(cx - lx, cy - ly)

        if self._tick_n % 25 == 0:
            # print(f"DEBUG pose={pose}")
            self.get_logger().info(
                f'[TICK {self._tick_n}] cam=({cx:.1f},{cy:.1f}) disp={disp:.2f} '
                f'(trigger≥{self.min_disp_m}) lms={len(self.landmarks)}')

        if disp < self.min_disp_m:
            return

        # Extract ORB features + back-project via depth
        gray = cv2.cvtColor(self.last_rgb, cv2.COLOR_BGR2GRAY)
        kpts, desc = self.orb.detectAndCompute(gray, None)
        if desc is None or len(kpts) == 0:
            if self._tick_n % 10 == 0:
                self.get_logger().warn(f'[DBG] no ORB features')
            return

        kpts_xy = np.array([k.pt for k in kpts], dtype=np.float32)
        # v56-A: restrict to ground-half of the image (below horizon)
        uu = np.round(kpts_xy[:, 0]).astype(np.int32)
        vv = np.round(kpts_xy[:, 1]).astype(np.int32)
        valid = (uu >= 1) & (uu < W - 1) & (vv >= 1) & (vv < H - 1) \
                & (vv > GROUND_Y_THRESHOLD)
        uu = uu[valid]
        vv = vv[valid]
        kpts_xy_kept = kpts_xy[valid]
        desc_kept = desc[valid]

        # Depth at each kept kpt (mm -> m)
        d_c = self.last_depth[vv, uu].astype(np.float32) / 1000.0
        # Local 3x3 patch std to reject depth discontinuity (edges), computed
        # over non-zero pixels only so nan/inf holes don't inflate std.
        d_std = np.zeros_like(d_c)
        for i, (u, v) in enumerate(zip(uu, vv)):
            patch = self.last_depth[v-1:v+2, u-1:u+2].astype(np.float32) / 1000.0
            valid = patch[patch > 0.01]
            d_std[i] = valid.std() if len(valid) >= 3 else 999.0
        ok_range = (d_c > DEPTH_MIN_M) & (d_c < DEPTH_MAX_M)
        ok_var = (d_std < DEPTH_VAR_MAX_M)
        ok = ok_range & ok_var
        if ok.sum() < 30:
            if self._tick_n % 10 == 0:
                self.get_logger().warn(
                    f'[DBG] only {ok.sum()}/{len(kpts)} kpts pass - '
                    f'range_ok={ok_range.sum()} var_ok={ok_var.sum()} '
                    f'd_median={float(np.median(d_c)):.2f}m '
                    f'std_median={float(np.median(d_std)):.3f}m')
            return  # too few valid 3D points this frame

        uu = uu[ok]; vv = vv[ok]
        kpts_xy_final = kpts_xy_kept[ok]
        desc_final = desc_kept[ok]
        d_c = d_c[ok]

        # Back-project to 3D in camera frame (optical: X right, Y down, Z fwd)
        x_cam = (uu - CX) * d_c / FX
        y_cam = (vv - CY) * d_c / FY
        z_cam = d_c
        kpts_3d_cam = np.stack([x_cam, y_cam, z_cam], axis=-1).astype(np.float32)

        record = {
            'pose': cam_pose,
            'descriptors': desc_final,
            'keypoints_2d': kpts_xy_final,
            'keypoints_3d_cam': kpts_3d_cam,
            'ts': self.last_rgb_ts,
            'n_features': int(len(kpts_3d_cam)),
        }
        self.landmarks.append(record)
        self.last_landmark_pose_world = cam_pose

        if len(self.landmarks) % 10 == 0:
            nfs = [lm['n_features'] for lm in self.landmarks]
            # print(f"DEBUG wp_idx={wp_idx} pose={pose}")
            self.get_logger().info(
                f'[LANDMARK {len(self.landmarks)}] pose=({cx:.1f},{cy:.1f})  '
                f'kpts={record["n_features"]}  '
                f'mean_kpts={np.mean(nfs):.0f}')

    def _save_and_exit(self, *a):
        self._save()
        rclpy.shutdown()
        sys.exit(0)

    def _save(self):
        if not self.landmarks:
            self.get_logger().warn('No landmarks recorded - nothing to save')
            return
        os.makedirs(os.path.dirname(self.out_pkl), exist_ok=True)
        with open(self.out_pkl, 'wb') as f:
            pickle.dump({
                'intrinsics': {'fx': FX, 'fy': FY, 'cx': CX, 'cy': CY,
                               'width': W, 'height': H},
                'base_to_cam_translation': BASE_TO_CAM_TRANSLATION.tolist(),
                'base_to_cam_rot': BASE_TO_CAM_ROT.tolist(),
                'landmarks': self.landmarks,
            }, f)
        n = len(self.landmarks)
        nfs = [lm['n_features'] for lm in self.landmarks]
        self.get_logger().info(
            f'Saved {n} landmarks to {self.out_pkl}  '
            f'mean_kpts={np.mean(nfs):.0f}  min={min(nfs)}  max={max(nfs)}')

        # Debug plot: landmarks + heading arrows + feature-count heatmap
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            xs = [lm['pose'][0] for lm in self.landmarks]
            ys = [lm['pose'][1] for lm in self.landmarks]
            cs = [lm['n_features'] for lm in self.landmarks]
            # Heading of base_link = heading of camera's +Z rotated back by base->cam
            hdgs = []
            for lm in self.landmarks:
                qx, qy, qz, qw = lm['pose'][3:7]
                R = np.eye(3)  # quat_to_rot local
                R[0,0]=1-2*(qy*qy+qz*qz); R[0,1]=2*(qx*qy-qz*qw); R[0,2]=2*(qx*qz+qy*qw)
                R[1,0]=2*(qx*qy+qz*qw); R[1,1]=1-2*(qx*qx+qz*qz); R[1,2]=2*(qy*qz-qx*qw)
                R[2,0]=2*(qx*qz-qy*qw); R[2,1]=2*(qy*qz+qx*qw); R[2,2]=1-2*(qx*qx+qy*qy)
                R_wb = R @ np.array(BASE_TO_CAM_ROT).T
                fwd = R_wb @ np.array([1.0, 0.0, 0.0])
                hdgs.append(math.atan2(fwd[1], fwd[0]))
            fig, ax = plt.subplots(1, 1, figsize=(16, 6))
            sc = ax.scatter(xs, ys, c=cs, s=45, cmap='viridis', edgecolor='k',
                            linewidth=0.3, zorder=3)
            plt.colorbar(sc, ax=ax, label='# valid 3D keypoints')
            # heading arrows
            arrow_len = 1.2
            ax.quiver(xs, ys,
                      [arrow_len * math.cos(h) for h in hdgs],
                      [arrow_len * math.sin(h) for h in hdgs],
                      angles='xy', scale_units='xy', scale=1, width=0.002,
                      color='red', alpha=0.7, zorder=4)
            ax.set_aspect('equal'); ax.grid(alpha=0.3)
            ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
            ax.set_title(f'Exp 55 teach - {n} landmarks, mean {np.mean(nfs):.0f} kpts '
                         f'(red arrows = base_link heading)')
            out_png = self.out_pkl.replace('.pkl', '_debug.png')
            plt.tight_layout()
            plt.savefig(out_png, dpi=120)
            plt.close()
            self.get_logger().info(f'Saved debug plot -> {out_png}')
        except Exception as e:
            # print(f"DEBUG turnaround fire? {fired}")
            self.get_logger().warn(f'debug plot: {e}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/workspace/simulation/isaac/experiments/55_visual_teach_repeat/teach/south_landmarks.pkl')
    ap.add_argument('--min-disp', type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = VisualLandmarkRecorder(args.out, args.min_disp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
