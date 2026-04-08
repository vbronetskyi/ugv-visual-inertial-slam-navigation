#!/usr/bin/env python3
"""periodic costmap snapshot logger

subscribes to /global_costmap/costmap (OccupancyGrid) and dumps a snapshot
every N seconds to <output_dir>/costmap_NNNN.npy + costmap_NNNN.yaml

usefull after a run for seeing what the planner actually saw.  run
compare_routes.png scripts read these to plot obstacles vs planned paths
"""
import argparse
import math
import os
import time
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class CostmapSnapshotter(Node):
    def __init__(self, out_dir, period_s, topic):
        super().__init__('costmap_snapshotter')
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.topic = topic
        self.period = period_s
        self.last_save = 0.0
        self.snapshot_id = 0
        self.last_costmap = None
        self.summary_path = os.path.join(out_dir, 'snapshots_summary.csv')
        with open(self.summary_path, 'w') as f:
            f.write('snapshot_id,wall_ts,robot_x,robot_y,n_occupied,n_unknown,n_free\n')
        self.create_subscription(OccupancyGrid, topic, self.cb, 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f"Snapshotting {topic} every {period_s}s -> {out_dir}")

    def cb(self, msg: OccupancyGrid):
        self.last_costmap = msg

    def _read_robot_pose(self):
        # Use same source as rest of pipeline
        try:
            with open('/tmp/isaac_pose.txt') as f:
                parts = f.readline().split()
            return float(parts[0]), float(parts[1])
        except Exception:
            return 0.0, 0.0

    def tick(self):
        now = time.time()
        if self.last_costmap is None:
            return
        if now - self.last_save < self.period:
            return
        self.last_save = now

        cm = self.last_costmap
        w, h = cm.info.width, cm.info.height
        data = np.array(cm.data, dtype=np.int8).reshape(h, w)
        # Nav2 OccupancyGrid convention: -1 unknown, 0 free, 1..100 occupied
        n_occ = int((data >= 50).sum())
        n_unk = int((data == -1).sum())
        n_free = int((data == 0).sum())

        rx, ry = self._read_robot_pose()

        base = os.path.join(self.out_dir, f"costmap_{self.snapshot_id:04d}")
        # Numeric save: .npy raw occupancy (int8, -1/0/1-100), loadable via
        # np.load() directly. One numeric file, no image conversions.
        np.save(base + '.npy', data)

        # JSON metadata per snapshot
        import json
        meta = {
            'snapshot_id': self.snapshot_id,
            'wall_ts': now,
            'width': w, 'height': h,
            'resolution': float(cm.info.resolution),
            'origin_x': float(cm.info.origin.position.x),
            'origin_y': float(cm.info.origin.position.y),
            'robot_xy': [float(rx), float(ry)],
            'n_occupied': n_occ, 'n_unknown': n_unk, 'n_free': n_free,
        }
        with open(base + '.json', 'w') as f:
            json.dump(meta, f, indent=2)

        with open(self.summary_path, 'a') as f:
            f.write(f"{self.snapshot_id},{now:.3f},{rx:.3f},{ry:.3f},"
                    f"{n_occ},{n_unk},{n_free}\n")

        if self.snapshot_id % 10 == 0:
            self.get_logger().info(
                f"snapshot #{self.snapshot_id}: {w}x{h} @ {cm.info.resolution}m  "
                f"occ={n_occ} unk={n_unk} free={n_free}  robot=({rx:.1f},{ry:.1f})")
        self.snapshot_id += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--period', type=float, default=5.0,
                    help='seconds between snapshots')
    ap.add_argument('--topic', type=str, default='/global_costmap/costmap')
    args = ap.parse_args()
    rclpy.init()
    node = CostmapSnapshotter(args.out_dir, args.period, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
