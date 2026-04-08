#!/usr/bin/env python3
"""log every Nav2 /plan as CSV (x,y per pose)

dumps each incoming Path message as plan_XXXX.csv with columns
  seq, wall_ts, x, y
and appends a summary row to plans_summary.csv:
  plan_id, wall_ts, n_poses, path_length_m, robot_x, robot_y, goal_x, goal_y

this is what compute_metrics reads post-hoc to compare planned vs actual
trajectories.  also usefull for debugging when Nav2 is returning weird
single-pose plans that make the BT wait forever
"""
import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path


class PlanLogger(Node):
    def __init__(self, out_dir):
        super().__init__('plan_logger')
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir
        self.plan_id = 0
        self.summary_path = os.path.join(out_dir, 'plans_summary.csv')
        with open(self.summary_path, 'w') as f:
            f.write('plan_id,wall_ts,n_poses,path_length_m,robot_x,robot_y,goal_x,goal_y\n')
        self.create_subscription(Path, '/plan', self.cb, 10)
        self.get_logger().info(f"Logging /plan -> {out_dir}")

    def _read_robot_pose(self):
        try:
            with open('/tmp/isaac_pose.txt') as f:
                parts = f.readline().split()
            return float(parts[0]), float(parts[1])
        except Exception:
            return 0.0, 0.0

    def cb(self, msg: Path):
        if len(msg.poses) < 2:
            return
        now = time.time()
        rx, ry = self._read_robot_pose()
        gx = msg.poses[-1].pose.position.x
        gy = msg.poses[-1].pose.position.y
        # Path length
        length = 0.0
        for i in range(1, len(msg.poses)):
            a = msg.poses[i-1].pose.position
            b = msg.poses[i].pose.position
            length += math.hypot(b.x-a.x, b.y-a.y)

        base = os.path.join(self.out_dir, f"plan_{self.plan_id:04d}.csv")
        with open(base, 'w') as f:
            f.write('seq,wall_ts,x,y\n')
            for i, ps in enumerate(msg.poses):
                f.write(f"{i},{now:.3f},{ps.pose.position.x:.3f},{ps.pose.position.y:.3f}\n")

        with open(self.summary_path, 'a') as f:
            f.write(f"{self.plan_id},{now:.3f},{len(msg.poses)},{length:.2f},"
                    f"{rx:.3f},{ry:.3f},{gx:.3f},{gy:.3f}\n")
        if self.plan_id % 10 == 0:
            self.get_logger().info(
                f"plan #{self.plan_id}: {len(msg.poses)} poses, {length:.1f}m  "
                f"robot=({rx:.1f},{ry:.1f}) goal=({gx:.1f},{gy:.1f})")
        self.plan_id += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    args = ap.parse_args()
    rclpy.init()
    node = PlanLogger(args.out_dir)
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
