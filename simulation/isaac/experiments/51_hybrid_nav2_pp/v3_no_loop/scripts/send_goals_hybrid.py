#!/usr/bin/env python3
"""Hybrid goal sender: request Nav2 plan for each WP, publish on /plan.

For each waypoint:
  1. Request plan from planner_server via ComputePathToPose action
  2. Publish resulting path on /plan (pure pursuit subscribes)
  3. Wait until robot within tolerance of goal
  4. Replan periodically in case robot drifted (depth costmap updates)
"""
import argparse
import csv
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped


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
        self.waypoints = waypoints
        self.GOAL_TIMEOUT = goal_timeout
        self.TOLERANCE = tolerance
        self.reached = 0
        self.skipped = 0

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.get_logger().info("Waiting for /compute_path_to_pose action...")
        self.plan_client.wait_for_server()
        self.get_logger().info(f"Ready. Following {len(waypoints)} waypoints.")
        self.start_time = time.time()

    def _make_goal_pose(self, x, y):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        return ps

    def _make_start_pose(self, rx, ry):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(rx)
        ps.pose.position.y = float(ry)
        ps.pose.orientation.w = 1.0
        return ps

    def request_plan(self, gx, gy):
        """Request a plan to (gx, gy). Use robot's current pose as start."""
        rx, ry = _read_robot_pose()
        if rx is None:
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_goal_pose(gx, gy)
        goal.start = self._make_start_pose(rx, ry)
        goal.use_start = False  # let planner use current TF
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
        """Follow a single waypoint: plan -> publish -> wait for arrival."""
        t0 = time.time()
        last_replan = 0
        REPLAN_PERIOD = 10.0  # re-plan every 10s in case obstacles changed
        while time.time() - t0 < self.GOAL_TIMEOUT:
            rx, ry = _read_robot_pose()
            if rx is None:
                time.sleep(0.5); continue
            d = math.hypot(x - rx, y - ry)
            if d < self.TOLERANCE:
                self.get_logger().info(f"  WP {i} REACHED (d={d:.1f}m)")
                return True
            # Replan periodically or if no path yet
            if time.time() - last_replan > REPLAN_PERIOD:
                path = self.request_plan(x, y)
                if path is not None and len(path.poses) > 1:
                    self.plan_pub.publish(path)
                    self.get_logger().info(
                        f"  WP {i}: plan {len(path.poses)} poses, d={d:.1f}m")
                    last_replan = time.time()
                else:
                    self.get_logger().warn(f"  WP {i}: plan failed")
                    last_replan = time.time()
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().warn(f"  WP {i} TIMEOUT (d={d:.1f}m)")
        return False

    def run(self):
        # Auto-start: closest WP in outbound half
        rx0, ry0 = _read_robot_pose()
        turn_idx = max(range(len(self.waypoints)), key=lambda i: self.waypoints[i][0])
        if rx0 is not None:
            start = min(range(turn_idx + 1),
                key=lambda i: math.hypot(
                    self.waypoints[i][0]-rx0, self.waypoints[i][1]-ry0))
            self.get_logger().info(
                f"Robot at ({rx0:.1f},{ry0:.1f}) - start WP {start}/{turn_idx}")
        else:
            start = 0

        for i in range(start, len(self.waypoints)):
            x, y = self.waypoints[i]
            rx, ry = _read_robot_pose()
            if rx is not None and math.hypot(x-rx, y-ry) < self.TOLERANCE:
                self.reached += 1
                self.get_logger().info(f"WP {i}/{len(self.waypoints)-1} already near")
                continue
            self.get_logger().info(
                f"WP {i}/{len(self.waypoints)-1}: ({x:.1f},{y:.1f})")
            if self.follow_waypoint(i, x, y):
                self.reached += 1
            else:
                self.skipped += 1

        total = time.time() - self.start_time
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"RESULT: reached {self.reached}/{len(self.waypoints)} "
            f"skipped {self.skipped} duration {total:.0f}s")
        self.get_logger().info("=" * 50)


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
        if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) >= args.spacing:
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
