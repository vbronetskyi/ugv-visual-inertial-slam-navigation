#!/usr/bin/env python3
"""Hybrid: pure pursuit follower for Nav2-planned path.

Subscribes to Nav2 /plan topic, follows the path using smooth pure pursuit.
Publishes /cmd_vel. No Nav2 controller needed - only planner.

Rationale (from exp 49 analysis):
  Nav2 DWB/MPPI emit oscillating angular velocity (±0.6 rad/s every 2-3 cycles)
  which breaks ORB feature tracking on curves. Pure pursuit gives smooth
  proportional commands that preserve VIO accuracy (exp 48: 0.49m ATE).

Nav2 stays responsible for:
  - Global planning (obstacle-aware via depth costmap)
  - Replanning when obstacles appear

Pure pursuit takes over:
  - Path following with smooth motion
  - Angular velocity proportional to heading error (no oscillation)
"""
import argparse
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import tf2_ros


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuitFollower(Node):
    def __init__(self, lookahead=2.0, max_vel=0.25, goal_tol=0.5):
        super().__init__('pure_pursuit_follower')
        self.LOOKAHEAD = lookahead
        self.MAX_VEL = max_vel          # cmd (actual = MAX_VEL * 3.4 for Husky)
        self.GAIN_ANG = 1.2
        self.MAX_ANG = 0.8
        self.GOAL_TOL = goal_tol

        self.path = None
        self.path_idx = 0
        self.path_done = False

        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz control
        self.log_counter = 0

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        self.path = msg
        self.path_idx = 0
        self.path_done = False
        self.get_logger().info(f'New path: {len(msg.poses)} poses, '
            f'end=({msg.poses[-1].pose.position.x:.1f},'
            f'{msg.poses[-1].pose.position.y:.1f})')

    def tick(self):
        if self.path is None or self.path_done:
            self.cmd_pub.publish(Twist())
            return

        try:
            t = self.tf_buf.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        ryaw = _yaw_from_quat(t.transform.rotation)

        # Track final path point distance for logging (but don't stop on it)
        fin = self.path.poses[-1].pose.position
        d_fin = math.hypot(fin.x - rx, fin.y - ry)

        # Find lookahead point: first path pose at least LOOKAHEAD away
        # from robot, starting from current path_idx
        lookahead_idx = None
        closest_idx = self.path_idx
        closest_d = float('inf')
        for i in range(self.path_idx, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y
            d = math.hypot(px - rx, py - ry)
            if d < closest_d:
                closest_d = d
                closest_idx = i
            if d >= self.LOOKAHEAD:
                lookahead_idx = i
                break

        if lookahead_idx is None:
            lookahead_idx = len(self.path.poses) - 1

        # Advance path_idx to closest (don't go backwards)
        self.path_idx = max(self.path_idx, closest_idx)

        tgt = self.path.poses[lookahead_idx].pose.position
        angle_to_tgt = math.atan2(tgt.y - ry, tgt.x - rx)
        err = angle_to_tgt - ryaw
        err = math.atan2(math.sin(err), math.cos(err))

        cmd = Twist()
        # Speed scales down with heading error (slow when turning sharp)
        cmd.linear.x = self.MAX_VEL * max(0.3, 1.0 - abs(err) / 1.57)
        cmd.angular.z = max(-self.MAX_ANG, min(self.MAX_ANG, self.GAIN_ANG * err))
        self.cmd_pub.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 20 == 0:  # 2 Hz log
            self.get_logger().info(
                f'pos=({rx:.1f},{ry:.1f}) tgt=({tgt.x:.1f},{tgt.y:.1f}) '
                f'err={math.degrees(err):.0f}° d_fin={d_fin:.1f}m '
                f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lookahead', type=float, default=2.0)
    ap.add_argument('--max-vel', type=float, default=0.25)
    ap.add_argument('--goal-tolerance', type=float, default=0.5)
    args = ap.parse_args()

    rclpy.init()
    node = PurePursuitFollower(args.lookahead, args.max_vel, args.goal_tolerance)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
