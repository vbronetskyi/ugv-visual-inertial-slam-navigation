#!/usr/bin/env python3
"""
two-phase waypoint sender for Nav2 - always forward, never back.

Adaptive speed based on drift from mapped trajectory.
Waypoint skipping when robot passes waypoint along forward axis.
No return-to-path - robot always moves forward.

usage:
  source /opt/ros/jazzy/setup.bash
  python3 send_nav2_goal.py --route road
"""
import json
import math
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


PHASE_FILE = "/tmp/nav2_phase.json"

# drift-based speed (always forward, never stop)
DRIFT_LOW = 2.0        # m - full speed
DRIFT_MED = 4.0        # m - slow
DRIFT_HIGH = 7.0       # m - crawl but still forward
SPEED_FULL = 0.8
SPEED_SLOW = 0.4
SPEED_CRAWL = 0.2

# waypoint skipping: skip if robot passed waypoint along X
WAYPOINT_SKIP_MARGIN = 3.0  # m past waypoint X -> skip it

CHECK_INTERVAL = 2.0   # seconds


def write_phase(phase):
    with open(PHASE_FILE, 'w') as f:
        json.dump({"phase": phase, "timestamp": time.time()}, f)


def make_pose(x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


class Nav2TwoPhase(Node):
    def __init__(self, route):
        super().__init__('nav2_goal_sender')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route = route
        self.outbound_poses = []
        self.return_poses = []
        self.outbound_wps = []
        self.current_poses = []
        self.current_idx = 0
        self.phase = "outbound"
        self._retries = 0
        self.current_drift = 0.0

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None

    def closest_dist_to_path(self, rx, ry):
        min_dist = float('inf')
        for wx, wy in self.outbound_wps:
            d = math.hypot(rx - wx, ry - wy)
            if d < min_dist:
                min_dist = d
        return min_dist

    def get_target_speed(self, drift):
        if drift < DRIFT_LOW:
            return SPEED_FULL
        elif drift < DRIFT_MED:
            return SPEED_SLOW
        elif drift < DRIFT_HIGH:
            return SPEED_CRAWL
        else:
            return SPEED_CRAWL  # always forward

    def start(self):
        with open('/tmp/slam_routes.json') as f:
            routes = json.load(f)

        wps = routes[self.route]
        dists = [math.hypot(p[0] - 72, p[1] + 5) for p in wps]
        turn_idx = dists.index(min(dists))

        outbound_wps_raw = wps[1:turn_idx + 1]
        self.outbound_wps = outbound_wps_raw
        for i, (x, y) in enumerate(outbound_wps_raw):
            if i < len(outbound_wps_raw) - 1:
                dx = outbound_wps_raw[i + 1][0] - x
                dy = outbound_wps_raw[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = math.pi
            self.outbound_poses.append(make_pose(x, y, yaw))

        return_wps = list(reversed(wps[0:turn_idx + 1]))
        for i, (x, y) in enumerate(return_wps):
            if i < len(return_wps) - 1:
                dx = return_wps[i + 1][0] - x
                dy = return_wps[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            self.return_poses.append(make_pose(x, y, yaw))

        self.get_logger().info(
            f'route "{self.route}": {len(self.outbound_poses)} outbound, '
            f'{len(self.return_poses)} return waypoints')

        self.current_poses = self.outbound_poses
        self.current_idx = 0
        self.phase = "outbound"

        self.get_logger().info('waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('=== PHASE 1: OUTBOUND (obstacles present) ===')

        # drift + skip monitor
        self.monitor_timer = self.create_timer(CHECK_INTERVAL, self.monitor_tick)
        self.send_next_goal()

    def monitor_tick(self):
        """Monitor drift and skip passed waypoints."""
        if not self.outbound_wps:
            return

        rx, ry = self.get_robot_pose()
        if rx is None:
            return

        # measure drift
        self.current_drift = self.closest_dist_to_path(rx, ry)
        speed = self.get_target_speed(self.current_drift)

        # skip waypoints that robot has passed (forward axis = +X for outbound)
        if self.phase == "outbound" and self.current_idx < len(self.outbound_wps):
            wx, _ = self.outbound_wps[self.current_idx]
            if rx > wx + WAYPOINT_SKIP_MARGIN:
                old_idx = self.current_idx
                # skip all waypoints behind robot
                while (self.current_idx < len(self.outbound_wps) and
                       rx > self.outbound_wps[self.current_idx][0] + WAYPOINT_SKIP_MARGIN):
                    self.current_idx += 1
                if self.current_idx != old_idx:
                    self.get_logger().info(
                        f'Skipped wp {old_idx+1}->{self.current_idx+1} '
                        f'(robot x={rx:.0f} passed wp x={wx:.0f})')
                    self.send_next_goal()

    def send_next_goal(self):
        if self.current_idx >= len(self.current_poses):
            if self.phase == "outbound":
                self.get_logger().info('=== OUTBOUND COMPLETE ===')
                self.get_logger().info('signaling obstacle removal...')
                write_phase("removing")
                time.sleep(5.0)
                self.get_logger().info('=== PHASE 2: RETURN (obstacles removed) ===')
                self.phase = "return"
                self.current_poses = self.return_poses
                self.current_idx = 0
                self._retries = 0
                self.send_next_goal()
            else:
                self.get_logger().info('=== ALL WAYPOINTS REACHED - NAVIGATION COMPLETE ===')
                write_phase("done")
            return

        pose = self.current_poses[self.current_idx]
        self.get_logger().info(
            f'[{self.phase}][{self.current_idx + 1}/{len(self.current_poses)}] goal: '
            f'({pose.pose.position.x:.1f},{pose.pose.position.y:.1f})'
            f' drift={self.current_drift:.1f}m')

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'goal {self.current_idx + 1} rejected, skipping...')
            self.current_idx += 1
            self._retries = 0
            self.send_next_goal()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        pos = fb.current_pose.pose.position
        self.get_logger().info(
            f'[{self.phase}][{self.current_idx + 1}/{len(self.current_poses)}] '
            f'pos=({pos.x:.1f},{pos.y:.1f}) remaining={fb.distance_remaining:.1f}m '
            f'drift={self.current_drift:.1f}m',
            throttle_duration_sec=5.0)

    def result_cb(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(
                f'[{self.phase}] waypoint {self.current_idx + 1} reached!')
            self.current_idx += 1
            self._retries = 0
            self.send_next_goal()
        else:
            self._retries += 1
            if self._retries >= 5:
                self.get_logger().warn(
                    f'[{self.phase}] waypoint {self.current_idx + 1} failed {self._retries}x, skipping')
                self.current_idx += 1
                self._retries = 0
                self.send_next_goal()
            else:
                self.get_logger().info(
                    f'[{self.phase}] waypoint {self.current_idx + 1} failed '
                    f'(err={result.error_code}), retry {self._retries}/5...')
                time.sleep(2.0)
                self.send_next_goal()


def main():
    route = 'road'
    for i, arg in enumerate(sys.argv):
        if arg == '--route' and i + 1 < len(sys.argv):
            route = sys.argv[i + 1]

    rclpy.init(args=sys.argv)
    node = Nav2TwoPhase(route)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
