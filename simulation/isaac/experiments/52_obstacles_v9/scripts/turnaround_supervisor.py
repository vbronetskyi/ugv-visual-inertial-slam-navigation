#!/usr/bin/env python3
"""Exp 52 turnaround supervisor - removes obstacles at turnaround in real time.

Polls /tmp/isaac_pose.txt. When robot has reached the max_x of the route
(turnaround zone) and is starting to move back (x decreasing), writes
/tmp/isaac_remove_obstacles.txt - Isaac's run_husky_forest.py reads this
flag and calls remove_obstacles(stage), deleting cones + tent from the
simulation.

Nav2 has no static map swap to do - the static map was blank all along.
Nav2's obstacle_layer clears the cleared cones from its costmap naturally
once the depth camera no longer sees them (clearing=True in yaml).
"""
import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node


def _read_pose():
    try:
        with open("/tmp/isaac_pose.txt") as f:
            parts = f.readline().split()
        return float(parts[0]), float(parts[1])
    except Exception:
        return None, None


class TurnaroundSupervisor(Node):
    def __init__(self, turnaround_x_threshold, past_turn_margin):
        super().__init__('turnaround_supervisor')
        self.THR = turnaround_x_threshold
        self.MARGIN = past_turn_margin
        self.max_x_seen = -1e9
        self.fired = False
        self.get_logger().info(
            f"Supervisor up. Will fire when GT x passes {self.THR} and "
            f"then decreases by {self.MARGIN} m.")
        self.timer = self.create_timer(0.5, self.tick)

    def fire(self):
        self.fired = True
        flag = "/tmp/isaac_remove_obstacles.txt"
        with open(flag, "w") as f:
            f.write(str(time.time()))
        self.get_logger().warn(
            f"[FIRE] wrote {flag} - Isaac drops cones+tent. "
            "Nav2 obstacle_layer will self-clear via clearing=True once "
            "depth camera no longer sees them.")

    def tick(self):
        if self.fired:
            return
        rx, ry = _read_pose()
        if rx is None:
            return
        if rx > self.max_x_seen:
            self.max_x_seen = rx
        if self.max_x_seen >= self.THR and rx < self.max_x_seen - self.MARGIN:
            self.get_logger().warn(
                f"TURNAROUND DETECTED: max_x={self.max_x_seen:.1f} current x={rx:.1f} "
                f"(past by {self.max_x_seen - rx:.1f}m). Firing.")
            self.fire()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turnaround-x', type=float, default=60.0,
                    help='x threshold indicating robot near turnaround')
    ap.add_argument('--past-margin', type=float, default=2.0,
                    help='once x decreases by this much after max_x, fire')
    args = ap.parse_args()

    rclpy.init()
    node = TurnaroundSupervisor(args.turnaround_x, args.past_margin)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
