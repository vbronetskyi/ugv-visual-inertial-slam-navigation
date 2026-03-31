#!/usr/bin/env python3
"""Send Nav2 waypoints sequentially from VIO trajectory (TUM format).

Reads a TUM trajectory file (ts tx ty tz qx qy qz qw), subsamples to
desired spacing, and sends each as a sequential /navigate_to_pose goal.

For exp 49 continuation: use VIO trajectory from exp 48 as reference path.
Auto-detects current robot position and starts from nearest outbound WP.

Usage:
  python3 send_waypoints.py --trajectory path/to/vio_final_camera.txt --spacing 4.0
"""
import argparse
import csv
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


def _load_vio_trajectory(path, aligned_csv=None):
    """Load VIO trajectory. If aligned CSV available, use world-frame coords;
    else load raw TUM and return camera-frame (caller must align)."""
    if aligned_csv and path.endswith('.csv'):
        wps = []
        with open(path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                wps.append((float(row.get('gt_x', row.get('x', 0))),
                            float(row.get('gt_y', row.get('y', 0)))))
        return wps
    # TUM format: ts tx ty tz qx qy qz qw
    # For VIO output that's in camera frame -> need alignment (not done here)
    raise NotImplementedError("Use pre-aligned CSV from analyze.py")


class WaypointFollower(Node):
    def __init__(self, waypoints, goal_timeout=300.0):
        super().__init__('waypoint_follower')
        self.waypoints = waypoints
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.reached = 0
        self.skipped = 0
        self.GOAL_TIMEOUT = goal_timeout
        self.MAX_ATTEMPTS = 2
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info(f"Nav2 ready. Following {len(waypoints)} waypoints.")
        self.start_time = time.time()

    def _robot_xy(self):
        try:
            with open("/tmp/isaac_pose.txt") as f:
                parts = f.readline().split()
            return float(parts[0]), float(parts[1])
        except Exception:
            return None, None

    def send_goal(self, x, y):
        msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        msg.pose = pose
        future = self.nav_client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        t0 = time.time()
        result_future = handle.get_result_async()
        while rclpy.ok() and (time.time() - t0) < self.GOAL_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.5)
            if result_future.done():
                return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        return False

    def run(self):
        # Auto-start: find closest WP in outbound half (max-x WP = turnaround)
        rx, ry = self._robot_xy()
        turn_idx = max(range(len(self.waypoints)),
                       key=lambda i: self.waypoints[i][0])
        if rx is not None:
            start = min(range(turn_idx + 1),
                        key=lambda i: math.hypot(
                            self.waypoints[i][0]-rx,
                            self.waypoints[i][1]-ry))
            self.get_logger().info(
                f"Robot at ({rx:.1f},{ry:.1f}) - start WP {start}/{turn_idx} (outbound)")
        else:
            start = 0

        for i in range(start, len(self.waypoints)):
            x, y = self.waypoints[i]
            rx, ry = self._robot_xy()
            if rx is not None and math.hypot(x-rx, y-ry) < 1.5:
                self.reached += 1
                self.get_logger().info(f"WP {i}/{len(self.waypoints)-1} already reached")
                continue
            for attempt in range(self.MAX_ATTEMPTS):
                self.get_logger().info(
                    f"WP {i}/{len(self.waypoints)-1}: ({x:.1f},{y:.1f}) attempt {attempt+1}")
                if self.send_goal(x, y):
                    self.reached += 1
                    self.get_logger().info(f"  WP {i} REACHED")
                    break
                self.get_logger().warn(f"  WP {i} failed (attempt {attempt+1})")
            else:
                self.skipped += 1
                self.get_logger().warn(f"  WP {i} SKIPPED")

        total = time.time() - self.start_time
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"RESULT: reached {self.reached}/{len(self.waypoints)} skipped {self.skipped} duration {total:.0f}s")
        self.get_logger().info("=" * 50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trajectory", required=True, help="VIO CSV with gt_x,gt_y cols (pre-aligned)")
    ap.add_argument("--spacing", type=float, default=4.0)
    ap.add_argument("--goal-timeout", type=float, default=300.0)
    args = ap.parse_args()

    # Load pre-aligned CSV (from analyze.py)
    pts = []
    with open(args.trajectory) as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((float(row['gt_x']), float(row['gt_y'])))

    # Subsample to spacing
    wps = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) >= args.spacing:
            wps.append(p)
    if math.hypot(pts[-1][0]-wps[-1][0], pts[-1][1]-wps[-1][1]) > 1.0:
        wps.append(pts[-1])
    print(f"Loaded {len(pts)} poses -> {len(wps)} Nav2 goals @ {args.spacing}m")

    rclpy.init()
    node = WaypointFollower(wps, args.goal_timeout)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
