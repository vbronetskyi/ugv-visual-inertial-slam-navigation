#!/usr/bin/env python3
"""turnaround supervisor, removes obstacles at turnaround time

the obstacle drop: during repeat runs we want the outbound leg to face
obstacles (cones + tent) so the pipeline proves it can detour, but the
return leg should be clean so we dont hit the same obstacles on the way back
this supervisor watches the robot pose and signals Isaac to delete obstacles
once the robot hits the turnaround zone

how it works: polls /tmp/isaac_pose.txt.  when robot has reached within
TRIGGER_RADIUS_M of the turnaround point and starts heading back (displacement
from turnaround > RESET_RADIUS_M), writes /tmp/isaac_remove_obstacles.txt
run_husky_forest.py reads this flag and calls remove_obstacles(stage)

Nav2 has no static map swap to do.  the static map was blank all along
Nav2's obstacle_layer clears the cleared cones from its costmap naturally
once the depth camera no longer sees them (clearing=True in yaml)

changelog:
  exp 52  - first version, triggered on x-coordinate only
  exp 55  - trigger on euclidean distance to turnaround_xy (routes 04+ are diagonal)
  exp 59  - added RESET_RADIUS_M to debounce (was firing repeatedly near turnaround)
  exp 62  - final-WP-reach signal for precise end approach

dropped exp 58 idea: use send_goals_hybrid's WP index instead of pose to detect
turnaround.  worked in simulation but coupled the supervisor to the WP list,
so moving to pose-based decoupling was cleaner
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
    """Exp 72: fires when robot returns near the FINAL point (5 m radius)
    after having travelled at least 30 m away. Keeps obstacles present on
    both outbound AND return legs; removes them only as robot approaches
    its finish so that the very last approach to start is unobstructed.
    """
    def __init__(self, final_x, final_y, near_radius=5.0, far_radius=30.0):
        super().__init__('turnaround_supervisor')
        self.FX, self.FY = final_x, final_y
        self.NEAR = near_radius
        self.FAR = far_radius
        self.been_far = False
        self.fired = False
        self.get_logger().info(
            f"Supervisor up. Will fire when robot reaches <{self.NEAR}m of "
            f"final ({self.FX:.1f},{self.FY:.1f}) after being >{self.FAR}m away.")
        self.timer = self.create_timer(0.5, self.tick)

    def fire(self):
        self.fired = True
        flag = "/tmp/isaac_remove_obstacles.txt"
        with open(flag, "w") as f:
            f.write(str(time.time()))
        self.get_logger().warn(
            f"[FIRE] wrote {flag} - Isaac drops cones+tent (robot is {self.NEAR} m from finish).")

    def tick(self):
        if self.fired:
            return
        rx, ry = _read_pose()
        if rx is None:
            return
        import math as _m
        d = _m.hypot(rx - self.FX, ry - self.FY)
        if d > self.FAR:
            self.been_far = True
        if self.been_far and d < self.NEAR:
            self.get_logger().warn(
                f"NEAR FINISH: robot at ({rx:.1f},{ry:.1f}), {d:.1f}m from "
                f"final. Firing obstacle removal.")
            self.fire()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turnaround-x', type=float, default=60.0,
                    help='(unused in exp 72 - kept for CLI backward-compat)')
    ap.add_argument('--past-margin', type=float, default=2.0,
                    help='(unused in exp 72)')
    ap.add_argument('--final-x', type=float, default=-90.88,
                    help='final (= start) x coordinate to trigger removal near')
    ap.add_argument('--final-y', type=float, default=-5.55,
                    help='final (= start) y coordinate to trigger removal near')
    ap.add_argument('--near-radius', type=float, default=5.0,
                    help='fire when within this m of final after being >30 m away')
    args = ap.parse_args()

    rclpy.init()
    node = TurnaroundSupervisor(args.final_x, args.final_y, near_radius=args.near_radius)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
