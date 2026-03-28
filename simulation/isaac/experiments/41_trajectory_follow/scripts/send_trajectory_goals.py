#!/usr/bin/env python3
"""
Follow recorded teach-run trajectory via Nav2 sequential goals.
On failure: backup, clear costmap, skip to next waypoint.
"""
import argparse
import json
import math
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from action_msgs.msg import GoalStatus


class TrajectoryFollower(Node):
    def __init__(self, waypoints_file):
        super().__init__("trajectory_follower")

        with open(waypoints_file) as f:
            all_anchors = json.load(f)
        max_x = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
        outbound = all_anchors[: max_x + 1]

        self.waypoints = []
        last = None
        for a in outbound:
            if last is None or math.hypot(a["x"] - last[0], a["y"] - last[1]) > 4.0:
                self.waypoints.append(a)
                last = (a["x"], a["y"])
        if math.hypot(outbound[-1]["x"] - last[0], outbound[-1]["y"] - last[1]) > 1.0:
            self.waypoints.append(outbound[-1])

        self.get_logger().info(f"Trajectory: {len(self.waypoints)} waypoints, "
                               f"({self.waypoints[0]['x']:.0f},{self.waypoints[0]['y']:.0f}) -> "
                               f"({self.waypoints[-1]['x']:.0f},{self.waypoints[-1]['y']:.0f})")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Waiting for Nav2...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 ready.")

        self.reached = 0
        self.skipped = 0
        self.start_time = time.time()

        self.MAX_ATTEMPTS = 2
        self.GOAL_TIMEOUT = 15.0
        self.BACKUP_DURATION = 1.5
        self.BACKUP_SPEED = -0.2

    def _get_robot_xy(self):
        """Read current GT position from pose file."""
        try:
            with open("/tmp/isaac_pose.txt") as f:
                parts = f.readline().split()
            return float(parts[0]), float(parts[1])
        except Exception:
            return None, None

    def run(self):
        i = 0
        while i < len(self.waypoints):
            wp = self.waypoints[i]

            # Skip waypoints behind the robot or very close
            rx, ry = self._get_robot_xy()
            if rx is not None:
                dx = wp["x"] - rx
                dist = math.hypot(wp["x"] - rx, wp["y"] - ry)
                if dx < -2.0:
                    self.get_logger().info(f"  WP {i} behind robot ({wp['x']:.0f} < {rx:.0f}), skip")
                    self.skipped += 1
                    i += 1
                    continue
                if dist < 2.5:
                    self.get_logger().info(f"  WP {i} within {dist:.1f}m, counting as reached")
                    self.reached += 1
                    i += 1
                    continue

            self.get_logger().info(
                f"WP {i}/{len(self.waypoints)-1}: ({wp['x']:.1f}, {wp['y']:.1f})"
            )

            success = False
            for attempt in range(self.MAX_ATTEMPTS):
                result = self._send_goal(wp)
                if result:
                    self.reached += 1
                    self.get_logger().info(f"  WP {i} REACHED")
                    success = True
                    break

                self.get_logger().warn(
                    f"  WP {i} failed (attempt {attempt + 1}/{self.MAX_ATTEMPTS})"
                )

                if attempt < self.MAX_ATTEMPTS - 1:
                    self._backup()
                    self._clear_local_only()
                    time.sleep(0.3)

            if not success:
                self.skipped += 1
                self.get_logger().warn(f"  WP {i} SKIPPED after {self.MAX_ATTEMPTS} attempts")

            i += 1

        elapsed = time.time() - self.start_time
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"RESULT:\n"
            f"  Reached: {self.reached}/{len(self.waypoints)}\n"
            f"  Skipped: {self.skipped}\n"
            f"  Duration: {elapsed:.0f}s\n"
            f"{'='*50}"
        )

    def _send_goal(self, wp):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp["x"])
        goal.pose.pose.position.y = float(wp["y"])
        yaw = float(wp.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return False
        gh = future.result()
        if not gh.accepted:
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.GOAL_TIMEOUT
        )

        if result_future.result() is not None:
            return result_future.result().status == GoalStatus.STATUS_SUCCEEDED
        else:
            gh.cancel_goal_async()
            time.sleep(0.5)
            return False

    def _backup(self):
        self.get_logger().info("  backing up...")
        twist = Twist()
        twist.linear.x = self.BACKUP_SPEED
        t0 = time.time()
        while time.time() - t0 < self.BACKUP_DURATION:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)

    def _clear_local_only(self):
        """Clear local costmap only - keep global obstacle memory."""
        try:
            subprocess.run(
                ["ros2", "service", "call",
                 "/local_costmap/clear_entirely_local_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
        except Exception:
            pass

    def _clear_costmaps(self):
        try:
            subprocess.run(
                ["ros2", "service", "call",
                 "/global_costmap/clear_entirely_global_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
            subprocess.run(
                ["ros2", "service", "call",
                 "/local_costmap/clear_entirely_local_costmap",
                 "nav2_msgs/srv/ClearEntireCostmap", "{}"],
                timeout=3, capture_output=True,
            )
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--anchors",
        default="/workspace/simulation/isaac/route_memory/road/anchors.json",
    )
    parser.add_argument(
        "--direction", default="outbound", choices=["outbound", "return"],
    )
    args = parser.parse_args()

    rclpy.init()
    follower = TrajectoryFollower(args.anchors)
    if args.direction == "return":
        follower.waypoints = list(reversed(follower.waypoints))
        follower.get_logger().info(
            f"RETURN mode: {len(follower.waypoints)} waypoints reversed"
        )
    try:
        follower.run()
    except KeyboardInterrupt:
        pass
    finally:
        follower.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
