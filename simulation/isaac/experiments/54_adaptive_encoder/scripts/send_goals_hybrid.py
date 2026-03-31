#!/usr/bin/env python3
"""Hybrid goal sender (v53) with proactive WP projection.

For every /global_costmap/costmap update:
  - Re-checks all *future* waypoints (current..end) against the costmap.
  - Any WP sitting in cost ≥ PROJ_COST_THRESH is BFS-projected to the
    nearest cell with cost < PROJ_COST_THRESH (capped search radius).
  - Maintains two lists: original (frozen) and projected (dynamic).
  - goal sender uses the projected list for Nav2 ComputePathToPose.

This stops the run-3 failure mode (WP lying inside a cone's inflation
bubble - planner returns a path ending 1 m from the cone centre, robot
hull then contacts it). Projection happens as soon as the obstacle
appears in the costmap - not when the robot gets close to it.

If no free cell is found within PROJ_MAX_SEARCH_M of a WP, that WP is
marked as SKIP and the sender moves to the next.
"""
import argparse
import csv
import math
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def _read_robot_pose():
    try:
        with open("/tmp/isaac_pose.txt") as f:
            parts = f.readline().split()
        return float(parts[0]), float(parts[1])
    except Exception:
        return None, None


class HybridGoalSender(Node):
    def __init__(self, waypoints, goal_timeout=300.0, tolerance=1.5):
        super().__init__('hybrid_goal_sender')
        self.original_wps = list(waypoints)
        self.projected_wps = list(waypoints)
        self.skip_flags = [False] * len(waypoints)
        self.n_wps = len(waypoints)
        self.GOAL_TIMEOUT = goal_timeout
        self.TOLERANCE = tolerance
        self.reached = 0
        self.skipped = 0
        self.current_idx = 0
        self.costmap = None
        self.costmap_info = None
        self.n_projections = 0
        self.n_skips_by_proj = 0

        # Projection parameters
        self.PROJ_COST_THRESH = 30   # cells >= this cost are "must avoid"
        self.PROJ_MAX_SEARCH_M = 3.0

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        qos_cm = QoSProfile(depth=1,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.costmap_cb, qos_cm)

        self.get_logger().info("Waiting for /compute_path_to_pose action...")
        self.plan_client.wait_for_server()
        self.get_logger().info(f"Ready. Following {self.n_wps} waypoints.")
        self.start_time = time.time()

    def _make_pose(self, x, y):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        return ps

    def _cell(self, x, y):
        info = self.costmap_info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        return r, c

    def _xy_from_cell(self, r, c):
        info = self.costmap_info
        x = info.origin.position.x + (c + 0.5) * info.resolution
        y = info.origin.position.y + (r + 0.5) * info.resolution
        return x, y

    def _cost_at(self, r, c):
        info = self.costmap_info
        if not (0 <= r < info.height and 0 <= c < info.width):
            return 100  # off-map treated as blocked
        v = int(self.costmap[r, c])
        if v < 0:       # unknown
            return 100
        return v

    def _project_wp(self, x, y):
        """BFS from (x,y) for nearest free cell. Returns (nx, ny, found_bool)."""
        if self.costmap is None:
            return x, y, True
        info = self.costmap_info
        r0, c0 = self._cell(x, y)
        if not (0 <= r0 < info.height and 0 <= c0 < info.width):
            return x, y, True  # off-map - leave alone
        c0v = self._cost_at(r0, c0)
        if c0v < self.PROJ_COST_THRESH:
            return x, y, True  # already OK

        max_cells = int(self.PROJ_MAX_SEARCH_M / info.resolution)
        q = deque([(r0, c0, 0)])
        seen = {(r0, c0)}
        while q:
            r, c, d = q.popleft()
            if d > max_cells:
                continue
            if self._cost_at(r, c) < self.PROJ_COST_THRESH:
                nx, ny = self._xy_from_cell(r, c)
                return nx, ny, True
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) in seen:
                    continue
                seen.add((nr, nc))
                q.append((nr, nc, d + 1))
        return x, y, False  # no free cell within search radius

    def costmap_cb(self, msg):
        self.costmap_info = msg.info
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # Re-project all WPs from current index forward
        n_changed = 0
        n_skipped_now = 0
        for i in range(self.current_idx, self.n_wps):
            ox, oy = self.original_wps[i]
            nx, ny, found = self._project_wp(ox, oy)
            if not found:
                if not self.skip_flags[i]:
                    n_skipped_now += 1
                self.skip_flags[i] = True
                # keep projected = original (will be skipped anyway)
                self.projected_wps[i] = (ox, oy)
            else:
                self.skip_flags[i] = False
                if (nx, ny) != tuple(self.projected_wps[i]):
                    n_changed += 1
                self.projected_wps[i] = (nx, ny)
        if n_changed or n_skipped_now:
            self.n_projections += n_changed
            self.n_skips_by_proj += n_skipped_now
            self.get_logger().info(
                f'[PROACTIVE] costmap update: re-projected {n_changed} WPs, '
                f'+{n_skipped_now} skipped (no free cell); '
                f'totals proj={self.n_projections} skip={self.n_skips_by_proj}')

    def request_plan(self, gx, gy):
        rx, ry = _read_robot_pose()
        if rx is None:
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(gx, gy)
        goal.start = self._make_pose(rx, ry)
        goal.use_start = False
        goal.planner_id = 'GridBased'
        fut = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            return None
        res_fut = h.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=15.0)
        if not res_fut.done():
            return None
        res = res_fut.result()
        if res is None:
            return None
        return res.result.path

    def follow_waypoint(self, i, x, y):
        t0 = time.time()
        last_replan = 0
        REPLAN_PERIOD = 3.0   # v54: faster replan so costmap updates (new obstacles) trigger new plan before robot drives into them at 1 m/s
        while time.time() - t0 < self.GOAL_TIMEOUT:
            # User projected position (may have moved since last tick)
            px, py = self.projected_wps[i]
            if self.skip_flags[i]:
                self.get_logger().warn(
                    f'  WP {i} SKIP - no free cell within '
                    f'{self.PROJ_MAX_SEARCH_M} m of ({self.original_wps[i][0]:.1f},'
                    f'{self.original_wps[i][1]:.1f})')
                return False
            rx, ry = _read_robot_pose()
            if rx is None:
                time.sleep(0.5); continue
            d = math.hypot(px - rx, py - ry)
            if d < self.TOLERANCE:
                self.get_logger().info(f"  WP {i} REACHED (d={d:.1f}m)")
                return True
            if time.time() - last_replan > REPLAN_PERIOD:
                path = self.request_plan(px, py)
                if path is not None and len(path.poses) > 1:
                    self.plan_pub.publish(path)
                    tag = '(projected)' if (px, py) != tuple(self.original_wps[i]) else ''
                    self.get_logger().info(
                        f'  WP {i}: plan {len(path.poses)} poses, d={d:.1f}m {tag}')
                    last_replan = time.time()
                else:
                    self.get_logger().warn(f'  WP {i}: plan failed')
                    last_replan = time.time()
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().warn(f'  WP {i} TIMEOUT (d={d:.1f}m)')
        return False

    def run(self):
        rx0, ry0 = _read_robot_pose()
        turn_idx = max(range(self.n_wps), key=lambda i: self.original_wps[i][0])
        if rx0 is not None:
            start = min(range(turn_idx + 1),
                        key=lambda i: math.hypot(
                            self.original_wps[i][0] - rx0,
                            self.original_wps[i][1] - ry0))
            self.get_logger().info(
                f"Robot at ({rx0:.1f},{ry0:.1f}) - start WP {start}/{turn_idx}")
        else:
            start = 0

        for i in range(start, self.n_wps):
            self.current_idx = i
            x, y = self.projected_wps[i]
            if self.skip_flags[i]:
                self.get_logger().warn(
                    f'WP {i}/{self.n_wps - 1}: SKIP (projection failed)')
                self.skipped += 1
                continue
            rx, ry = _read_robot_pose()
            if rx is not None and math.hypot(x - rx, y - ry) < self.TOLERANCE:
                self.reached += 1
                self.get_logger().info(f"WP {i}/{self.n_wps - 1} already near")
                continue
            tag = '(projected)' if (x, y) != tuple(self.original_wps[i]) else ''
            self.get_logger().info(
                f"WP {i}/{self.n_wps - 1}: ({x:.1f},{y:.1f}) {tag}")
            if self.follow_waypoint(i, x, y):
                self.reached += 1
            else:
                self.skipped += 1

        total = time.time() - self.start_time
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f"RESULT: reached {self.reached}/{self.n_wps} "
            f"skipped {self.skipped} duration {total:.0f}s "
            f"projections={self.n_projections} skip_by_proj={self.n_skips_by_proj}")
        self.get_logger().info('=' * 50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    ap.add_argument('--goal-timeout', type=float, default=300.0)
    ap.add_argument('--tolerance', type=float, default=1.5)
    args = ap.parse_args()

    pts = []
    with open(args.trajectory) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['gt_x']), float(row['gt_y'])))
    wps = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - wps[-1][0], p[1] - wps[-1][1]) >= args.spacing:
            wps.append(p)
    print(f"Loaded {len(pts)} poses -> {len(wps)} goals @ {args.spacing}m")

    rclpy.init()
    node = HybridGoalSender(wps, args.goal_timeout, args.tolerance)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
