#!/usr/bin/env python3
"""
Send teach-repeat waypoints to Nav2 as navigation goals.

Reads world-frame waypoints from JSON, sends them one by one to Nav2.
Nav2 handles obstacle avoidance via depth costmap.

Usage:
    ros2 run --prefix 'python3' send_tr_waypoints.py --waypoints config/tr_waypoints.json
"""
import argparse
import json
import math
import time
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class TRWaypointSender(Node):
    def __init__(self, waypoints_file):
        super().__init__("tr_waypoint_sender")

        with open(waypoints_file) as f:
            self.waypoints = json.load(f)
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints from {waypoints_file}"
        )

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready.")

        self.current_wp = 0
        self.total_reached = 0
        self.total_failed = 0
        self.start_time = time.time()

        # Waypoint skip: if robot is already past a waypoint, skip it
        self.SKIP_DISTANCE = 3.0   # meters
        self.GOAL_TIMEOUT = 30.0   # seconds per waypoint
        self.MAX_RETRIES = 2

    def run(self):
        """Send all waypoints sequentially."""
        for i, wp in enumerate(self.waypoints):
            self.current_wp = i
            self.get_logger().info(
                f"WP {i}/{len(self.waypoints)-1}: "
                f"({wp['x']:.1f}, {wp['y']:.1f})"
            )

            success = self._send_goal(wp)

            if success:
                self.total_reached += 1
                self.get_logger().info(f"  WP {i} REACHED")
            else:
                self.total_failed += 1
                self.get_logger().warn(f"  WP {i} FAILED - skipping")

        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"RESULT:\n"
            f"  Waypoints: {self.total_reached}/{len(self.waypoints)} reached\n"
            f"  Failed: {self.total_failed}\n"
            f"  Duration: {elapsed:.0f}s\n"
            f"{'='*50}"
        )

    def _send_goal(self, wp):
        """Send single waypoint goal and wait."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        goal.pose.pose.position.z = 0.0

        yaw = float(wp.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        for attempt in range(self.MAX_RETRIES):
            future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.result() is None:
                self.get_logger().warn(f"  Goal rejected (attempt {attempt+1})")
                continue

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"  Goal not accepted (attempt {attempt+1})")
                continue

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=self.GOAL_TIMEOUT
            )

            if result_future.result() is not None:
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return True
                else:
                    self.get_logger().warn(
                        f"  Goal status={status} (attempt {attempt+1})"
                    )
            else:
                # Timeout - cancel and retry
                self.get_logger().warn(
                    f"  Timeout after {self.GOAL_TIMEOUT}s (attempt {attempt+1})"
                )
                goal_handle.cancel_goal_async()
                time.sleep(1.0)

        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--waypoints",
        default="/workspace/simulation/isaac/experiments/40_tr_nav2_hybrid/config/tr_waypoints.json",
    )
    args = parser.parse_args()

    rclpy.init()
    sender = TRWaypointSender(args.waypoints)
    try:
        sender.run()
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
