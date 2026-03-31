#!/usr/bin/env python3
"""Hybrid goal sender: request Nav2 plan for each WP, publish on /plan.

v10: home-zone goal collapsing. Once the robot is within HOMEZONE_RADIUS of
the final WP, collapse all remaining waypoints into a single final goal
with enlarged tolerance - avoids the end-zone cascade seen in v9 where
tight-cluster WPs kept retriggering spin.

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
    def __init__(self, waypoints, goal_timeout=300.0, tolerance=1.5,
                 homezone_radius=5.0, homezone_tolerance=None):
        super().__init__('hybrid_goal_sender')
        self.waypoints = waypoints
        self.GOAL_TIMEOUT = goal_timeout
        self.TOLERANCE = tolerance
        self.HOMEZONE_RADIUS = homezone_radius
        self.HOMEZONE_TOL = homezone_tolerance if homezone_tolerance is not None else tolerance
        self.reached = 0
        self.skipped = 0

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.get_logger().info("Waiting for /compute_path_to_pose action...")
        self.plan_client.wait_for_server()
        self.get_logger().info(
            f"Ready. Following {len(waypoints)} WPs, tolerance {tolerance}m, "
            f"home-zone radius {homezone_radius}m.")
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
        rx, ry = _read_robot_pose()
        if rx is None:
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_goal_pose(gx, gy)
        goal.start = self._make_start_pose(rx, ry)
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

    def follow_waypoint(self, i, x, y, tolerance=None):
        if tolerance is None:
            tolerance = self.TOLERANCE
        t0 = time.time()
        last_replan = 0
        REPLAN_PERIOD = 10.0
        while time.time() - t0 < self.GOAL_TIMEOUT:
            rx, ry = _read_robot_pose()
            if rx is None:
                time.sleep(0.5); continue
            d = math.hypot(x - rx, y - ry)
            if d < tolerance:
                self.get_logger().info(f"  WP {i} REACHED (d={d:.1f}m)")
                return True
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

        final_wp = self.waypoints[-1]
        homezone_collapsed = False
        # Require robot to leave home-zone BEFORE we arm the collapse logic.
        # South route is a loop - robot spawns within ~4m of the final WP,
        # so we must wait until it departs on the outbound leg.
        left_homezone = False
        LEAVE_MARGIN = 1.5 * self.HOMEZONE_RADIUS  # hysteresis

        i = start
        while i < len(self.waypoints):
            x, y = self.waypoints[i]
            rx, ry = _read_robot_pose()

            if rx is not None:
                d_home = math.hypot(final_wp[0]-rx, final_wp[1]-ry)
                if not left_homezone and d_home > LEAVE_MARGIN:
                    left_homezone = True
                    self.get_logger().info(
                        f"[HOME-ZONE] robot left home zone (now {d_home:.1f}m from final WP) "
                        f"- collapse logic armed")

            # v10 home-zone collapse: once within HOMEZONE_RADIUS of the final
            # WP (and only after the robot has actually left it once on the
            # outbound leg), skip to the final WP with enlarged tolerance -
            # avoids the end-zone cascade where tight-cluster WPs keep
            # retriggering spin.
            if (left_homezone and not homezone_collapsed and rx is not None and
                    math.hypot(final_wp[0]-rx, final_wp[1]-ry) < self.HOMEZONE_RADIUS):
                skipped_to = len(self.waypoints) - 1
                self.get_logger().info(
                    f"[HOME-ZONE] robot within {self.HOMEZONE_RADIUS}m of final WP - "
                    f"collapsing WPs {i}..{skipped_to-1} into final WP with "
                    f"tolerance {self.HOMEZONE_TOL}m")
                # Mark skipped WPs as reached (we count anything we were about
                # to aim for and are already near as "reached")
                for _ in range(i, skipped_to):
                    self.reached += 1
                i = skipped_to
                homezone_collapsed = True
                # Follow the final WP with the (larger) home-zone tolerance
                x, y = self.waypoints[i]
                self.get_logger().info(
                    f"WP {i}/{len(self.waypoints)-1} (home-zone final): ({x:.1f},{y:.1f})")
                if self.follow_waypoint(i, x, y, tolerance=self.HOMEZONE_TOL):
                    self.reached += 1
                else:
                    self.skipped += 1
                break

            if rx is not None and math.hypot(x-rx, y-ry) < self.TOLERANCE:
                self.reached += 1
                self.get_logger().info(f"WP {i}/{len(self.waypoints)-1} already near")
                i += 1
                continue
            self.get_logger().info(
                f"WP {i}/{len(self.waypoints)-1}: ({x:.1f},{y:.1f})")
            if self.follow_waypoint(i, x, y):
                self.reached += 1
            else:
                self.skipped += 1
            i += 1

        total = time.time() - self.start_time
        self.get_logger().info("=" * 50)
        self.get_logger().info(
            f"RESULT: reached {self.reached}/{len(self.waypoints)} "
            f"skipped {self.skipped} duration {total:.0f}s "
            f"home-zone-collapsed={homezone_collapsed}")
        self.get_logger().info("=" * 50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    ap.add_argument('--goal-timeout', type=float, default=300.0)
    ap.add_argument('--tolerance', type=float, default=1.5)
    ap.add_argument('--homezone-radius', type=float, default=5.0,
                    help='collapse remaining WPs once within this radius of final')
    ap.add_argument('--homezone-tolerance', type=float, default=3.0,
                    help='tolerance used for the collapsed final WP')
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
    node = HybridGoalSender(wps, args.goal_timeout, args.tolerance,
                            args.homezone_radius, args.homezone_tolerance)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
