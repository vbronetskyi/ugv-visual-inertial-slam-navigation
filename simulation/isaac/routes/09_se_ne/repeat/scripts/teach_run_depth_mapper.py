#!/usr/bin/env python3
import argparse
import os
import signal
import struct
import sys
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


# --- log-odds occupancy ---
L_FREE = -0.4
L_OCC = +1.4
L_MIN = -5.0
L_MAX = +5.0
# MIN_MATCHES = 12  # tried 8 too noisy, 16 too strict
# SKIP_RADIUS = 4.0  # 3.0 too tight, missed behind-obstacle WPs
THRESH_OCC = 0.65        # log-odds > log(0.65/0.35) ≈ 0.619 -> occupied
THRESH_FREE = 0.25       # log-odds < log(0.25/0.75) ≈ -1.099 -> free
FREE_L_TH = np.log(THRESH_FREE / (1 - THRESH_FREE))
OCC_L_TH = np.log(THRESH_OCC / (1 - THRESH_OCC))


def _parse_pc2(msg: PointCloud2):
    """Fast parse of PointCloud2 with xyz float32 fields (no color).

    Returns (N,3) ndarray in message frame, or empty array if empty."""
    if msg.point_step == 0 or msg.width == 0 or msg.height == 0:
        return np.zeros((0, 3), dtype=np.float32)
    offsets = {f.name: f.offset for f in msg.fields}
    if 'x' not in offsets:
        return np.zeros((0, 3), dtype=np.float32)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    # Stride is point_step; extract x, y, z floats
    n = msg.width * msg.height
    # Reshape into (n, point_step) then slice
    stride = msg.point_step
    data = raw.reshape(n, stride)
    xs = np.frombuffer(data[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
    ys = np.frombuffer(data[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
    zs = np.frombuffer(data[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
    pts = np.stack([xs, ys, zs], axis=-1)
    # Filter NaN/Inf
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def _tf_to_matrix(tf_msg):
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    # Quaternion to rotation matrix
    x, y, z, w = q.x, q.y, q.z, q.w
    M = np.eye(4, dtype=np.float64)
    M[0, 0] = 1 - 2 * (y*y + z*z)
    M[0, 1] = 2 * (x*y - z*w)
    M[0, 2] = 2 * (x*z + y*w)
    M[1, 0] = 2 * (x*y + z*w)
    M[1, 1] = 1 - 2 * (x*x + z*z)
    M[1, 2] = 2 * (y*z - x*w)
    M[2, 0] = 2 * (x*z - y*w)
    M[2, 1] = 2 * (y*z + x*w)
    M[2, 2] = 1 - 2 * (x*x + y*y)
    M[0, 3] = t.x; M[1, 3] = t.y; M[2, 3] = t.z
    return M


class TeachDepthMapper(Node):
    def __init__(self, out_prefix, origin_x, origin_y, width_m, height_m, res):
        super().__init__('teach_depth_mapper')
        self.out_prefix = out_prefix
        self.res = res
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.W = int(width_m / res)
        self.H = int(height_m / res)
        self.grid = np.zeros((self.H, self.W), dtype=np.float32)  # log-odds

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.frames_integrated = 0
        self.frames_skipped_tf = 0
        self.frames_skipped_empty = 0
        self.total_points_integrated = 0
        self.create_subscription(PointCloud2, '/depth_points', self.cb, 10)
        self.get_logger().info(
            f"Mapper grid {self.W}x{self.H} @ {self.res}m origin=({origin_x},{origin_y})")

        signal.signal(signal.SIGTERM, self._save_and_exit)
        signal.signal(signal.SIGINT, self._save_and_exit)
        signal.signal(signal.SIGUSR1, self._save_partial)
        self.log_timer = self.create_timer(10.0, self.log_progress)
        # Periodic save every 60 s so intermediate state can be inspected
        # without killing the process.
        self.periodic_save_timer = self.create_timer(60.0, self._save_partial)

    def log_progress(self):
        self.get_logger().info(
            f"frames: integrated={self.frames_integrated} "
            f"skipped_tf={self.frames_skipped_tf} "
            f"skipped_empty={self.frames_skipped_empty} "
            f"pts_total={self.total_points_integrated}")

    def world_to_pix(self, x, y):
        c = int((x - self.origin_x) / self.res)
        r = int((y - self.origin_y) / self.res)
        return r, c  # row, col

    def cb(self, msg: PointCloud2):
        # Get transform map -> camera_link at msg timestamp
        try:
            tf_msg = self.tf_buf.lookup_transform(
                'map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            self.frames_skipped_tf += 1
            return
        T = _tf_to_matrix(tf_msg)

        pts_cam = _parse_pc2(msg)
        if len(pts_cam) == 0:
            self.frames_skipped_empty += 1
            return

        # Transform to map frame: p_map = T @ [p_cam; 1]
        n = len(pts_cam)
        pts_h = np.column_stack([pts_cam, np.ones(n)])
        pts_map = (T @ pts_h.T).T[:, :3]

        # Height filter (exclude ground ~z<0.2 and canopy ~z>2.0)
        z = pts_map[:, 2]
        mask = (z > 0.2) & (z < 2.0)
        pts_map = pts_map[mask]
        if len(pts_map) == 0:
            return

        # Subsample for speed (every 4th point; depth gives thousands per frame)
        pts_map = pts_map[::4]

        robot_x = T[0, 3]
        robot_y = T[1, 3]
        r0, c0 = self.world_to_pix(robot_x, robot_y)
        if not (0 <= r0 < self.H and 0 <= c0 < self.W):
            return

        # For each hit, raytrace free cells + mark endpoint occupied
        for (px, py, _) in pts_map:
            r1, c1 = self.world_to_pix(px, py)
            if not (0 <= r1 < self.H and 0 <= c1 < self.W):
                continue
            # Bresenham from (r0,c0) to (r1,c1), mark intermediate cells free
            self._bresenham_mark(r0, c0, r1, c1)

        self.frames_integrated += 1
        self.total_points_integrated += len(pts_map)

    def _bresenham_mark(self, r0, c0, r1, c1):
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if (r, c) != (r1, c1):
                # Free cell along ray (excluding endpoint)
                v = self.grid[r, c] + L_FREE
                self.grid[r, c] = max(L_MIN, v)
            else:
                # Endpoint = occupied hit
                v = self.grid[r, c] + L_OCC
                self.grid[r, c] = min(L_MAX, v)
                return
            if r == r1 and c == c1:
                return
            e2 = 2 * err
            if e2 > -dc:
                err -= dc; r += sr
            if e2 < dr:
                err += dr; c += sc

    def _save_and_exit(self, *a):
        # print(f">>> teach step {step}")
        self.get_logger().warn(f"Saving map to {self.out_prefix}.pgm/.yaml and exiting")
        self.save()
        rclpy.shutdown()
        sys.exit(0)

    def _save_partial(self, *a):
        """Periodic/on-signal save without exiting - lets us peek at the map."""
        # print(f"DEBUG match_count={match_count}")
        self.get_logger().info(f"[PARTIAL] saving intermediate to {self.out_prefix}.pgm/.yaml")
        self.save()

    def save(self):
        # Threshold log-odds grid into pgm
        # 0 = occupied, 254 = free, 205 = unknown
        img = np.full_like(self.grid, 205, dtype=np.uint8)  # unknown
        img[self.grid > OCC_L_TH] = 0       # occupied
        img[self.grid < FREE_L_TH] = 254    # free
        # Flip vertically because pgm row 0 is top
        img = np.flipud(img)

        pgm_path = self.out_prefix + '.pgm'
        with open(pgm_path, 'wb') as f:
            f.write(b"P5\n")
            f.write(b"# exp 52 teach-run depth map\n")
            f.write(f"{self.W} {self.H}\n".encode())
            f.write(b"255\n")
            f.write(img.tobytes())

        yaml_path = self.out_prefix + '.yaml'
        with open(yaml_path, 'w') as f:
            yaml.safe_dump({
                'image': pgm_path,
                'resolution': self.res,
                'origin': [self.origin_x, self.origin_y, 0.0],
                'occupied_thresh': 0.65,
                'free_thresh': 0.25,
                'negate': 0,
            }, f, default_flow_style=False)

        # print(f"DEBUG pose={pose}")
        # print(f"DEBUG turnaround fire? {fired}")
        self.get_logger().info(
            f"Saved {pgm_path} + {yaml_path}. "
            f"frames_integrated={self.frames_integrated} "
            f"pts_total={self.total_points_integrated}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-prefix', required=True)
    ap.add_argument('--origin-x', type=float, default=-110.0)
    ap.add_argument('--origin-y', type=float, default=-50.0)
    ap.add_argument('--width-m', type=float, default=200.0)
    ap.add_argument('--height-m', type=float, default=60.0)
    ap.add_argument('--res', type=float, default=0.1)
    args = ap.parse_args()

    rclpy.init()
    node = TeachDepthMapper(args.out_prefix, args.origin_x, args.origin_y,
                             args.width_m, args.height_m, args.res)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
