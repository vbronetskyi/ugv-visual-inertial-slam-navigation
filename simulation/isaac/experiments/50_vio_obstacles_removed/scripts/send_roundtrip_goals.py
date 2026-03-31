#!/usr/bin/env python3
"""Exp 49: Send Nav2 goals for full roundtrip (outbound + turnaround + return).

Reads dense 797-WP roundtrip route, subsamples to ~4m spacing for Nav2 goals.
Unlike send_trajectory_goals.py which only does outbound, this follows the
complete path (including turnaround loop) and returns to spawn.
"""
import argparse
import json
import math
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class RoundtripFollower(Node):
    def __init__(self, route_json, spacing=4.0):
        super().__init__("roundtrip_follower")

        with open(route_json) as f:
            full_route = json.load(f)
        # Format: [[x,y], ...]
        # Subsample to spacing m (keep all WPs that are > spacing from last)
        wps = [full_route[0]]
        for p in full_route[1:]:
            if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) >= spacing:
                wps.append(p)
        if math.hypot(full_route[-1][0]-wps[-1][0], full_route[-1][1]-wps[-1][1]) > 1.0:
            wps.append(full_route[-1])
        self.waypoints = wps

        self.get_logger().info(
            f"Roundtrip: {len(wps)} goals @ {spacing}m from {len(full_route)} dense WPs"
        )
        # Identify turnaround
        max_x_idx = max(range(len(wps)), key=lambda i: wps[i][0])
        self.get_logger().info(
            f"  Turnaround at goal {max_x_idx}: ({wps[max_x_idx][0]:.1f},{wps[max_x_idx][1]:.1f})"
        )
        self.get_logger().info(
            f"  Start: ({wps[0][0]:.1f},{wps[0][1]:.1f}) End: ({wps[-1][0]:.1f},{wps[-1][1]:.1f})"
        )

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.reached = 0
        self.skipped = 0
        self.start_time = time.time()

        self.MAX_ATTEMPTS = 2
        self.GOAL_TIMEOUT = 300.0  # slow sim compensation

        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready.")

    def _get_robot_xy(self):
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
                status = result_future.result().status
                return status == GoalStatus.STATUS_SUCCEEDED
        # Timeout - cancel
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=3.0)
        return False

    def run(self):
        # Skip to closest WP in outbound half (handles warmup drift on outbound)
        # Route is symmetric: outbound 0-45, return 46-90. Robot after warmup
        # is still on outbound, so restrict search to outbound half only.
        rx0, ry0 = self._get_robot_xy()
        turn_idx = max(range(len(self.waypoints)),
                       key=lambda i: self.waypoints[i][0])
        if rx0 is not None:
            # Search only outbound half
            candidates = range(0, turn_idx + 1)
            start = min(candidates,
                        key=lambda i: math.hypot(self.waypoints[i][0]-rx0,
                                                 self.waypoints[i][1]-ry0))
            self.get_logger().info(
                f"Robot at ({rx0:.1f},{ry0:.1f}) - start WP {start}/{turn_idx} (outbound half)"
            )
        else:
            start = 0

        for i in range(start, len(self.waypoints)):
            x, y = self.waypoints[i]
            rx, ry = self._get_robot_xy()
            if rx is None:
                self.get_logger().warn("No GT pose, skipping WP")
                continue
            # Skip if too close already
            if math.hypot(x-rx, y-ry) < 1.5:
                self.get_logger().info(f"WP {i}/{len(self.waypoints)-1} already reached")
                self.reached += 1
                continue

            for attempt in range(self.MAX_ATTEMPTS):
                self.get_logger().info(
                    f"WP {i}/{len(self.waypoints)-1}: ({x:.1f},{y:.1f}) attempt {attempt+1}"
                )
                ok = self.send_goal(x, y)
                if ok:
                    self.reached += 1
                    self.get_logger().info(f"  WP {i} REACHED")
                    break
                else:
                    self.get_logger().warn(f"  WP {i} failed (attempt {attempt+1})")
            else:
                self.skipped += 1
                self.get_logger().warn(f"  WP {i} SKIPPED")

        total_t = time.time() - self.start_time
        self.get_logger().info("=" * 50)
        self.get_logger().info("RESULT:")
        self.get_logger().info(f"  Reached: {self.reached}/{len(self.waypoints)}")
        self.get_logger().info(f"  Skipped: {self.skipped}")
        self.get_logger().info(f"  Duration: {total_t:.0f}s")
        self.get_logger().info("=" * 50)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--route", required=True, help="Full dense route JSON")
    parser.add_argument("--spacing", type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init()
    node = RoundtripFollower(args.route, args.spacing)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
