#!/usr/bin/env python3
"""
Two-phase waypoint sender for Nav2 with SLAM frame support.

Aggressive but safe waypoint skipping:
  - Forward skip: robot passed waypoint along X
  - Timeout skip: stuck too long (outbound only)
  - Failure skip: Nav2 can't reach after N retries
  - Max consecutive skip limit prevents cascade

usage:
  python3 send_nav2_goal.py --route road [--slam-frame]
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

# skip parameters
SKIP_FORWARD_MARGIN = 3.0    # m - skip if robot X > wp X + this
SKIP_TIMEOUT = 20.0           # s - skip if stuck this long (outbound only)
MAX_NAV2_RETRIES = 3          # failures before skip
MAX_CONSECUTIVE_SKIPS = 3     # prevent cascade: max skips in a row
CHECK_INTERVAL = 2.0


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
    def __init__(self, route, slam_frame=False):
        super().__init__('nav2_goal_sender')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.route = route
        self.slam_frame = slam_frame
        self.outbound_poses = []
        self.return_poses = []
        self.outbound_wps = []
        self.current_poses = []
        self.current_idx = 0
        self.phase = "outbound"
        self._retries = 0
        self.current_drift = 0.0
        self.wp_start_time = time.time()
        self.current_goal_handle = None
        self.consecutive_skips = 0

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

    def start(self):
        if self.slam_frame:
            self._load_slam_frame_waypoints()
        else:
            self._load_world_frame_waypoints()

        self.current_poses = self.outbound_poses
        self.current_idx = 0
        self.phase = "outbound"

        self.get_logger().info('waiting for Nav2 action server...')
        self.client.wait_for_server()
        self.get_logger().info('=== PHASE 1: OUTBOUND (obstacles present) ===')

        self.monitor_timer = self.create_timer(CHECK_INTERVAL, self.monitor_tick)
        self.send_next_goal()

    def _load_slam_frame_waypoints(self):
        with open('/tmp/slam_waypoints.json') as f:
            data = json.load(f)
        outbound_raw = data['outbound']
        return_raw = data['return']
        self.outbound_wps = outbound_raw

        for i, (x, y) in enumerate(outbound_raw):
            if i < len(outbound_raw) - 1:
                dx = outbound_raw[i + 1][0] - x
                dy = outbound_raw[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = math.pi
            self.outbound_poses.append(make_pose(x, y, yaw))

        for i, (x, y) in enumerate(return_raw):
            if i < len(return_raw) - 1:
                dx = return_raw[i + 1][0] - x
                dy = return_raw[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0
            self.return_poses.append(make_pose(x, y, yaw))

        self.get_logger().info(
            f'SLAM-FRAME route: {len(self.outbound_poses)} outbound, '
            f'{len(self.return_poses)} return waypoints')

    def _load_world_frame_waypoints(self):
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
            f'WORLD route "{self.route}": {len(self.outbound_poses)} outbound, '
            f'{len(self.return_poses)} return waypoints')

    def monitor_tick(self):
        if not self.outbound_wps:
            return
        rx, ry = self.get_robot_pose()
        if rx is None:
            return

        self.current_drift = self.closest_dist_to_path(rx, ry)

        if self.current_idx >= len(self.current_poses):
            return

        # --- Forward skip (outbound only): robot passed waypoint along X ---
        if self.phase == "outbound" and self.current_idx < len(self.outbound_wps):
            wx = self.outbound_wps[self.current_idx][0]
            if rx > wx + SKIP_FORWARD_MARGIN:
                if self.consecutive_skips < MAX_CONSECUTIVE_SKIPS:
                    old_idx = self.current_idx
                    while (self.current_idx < len(self.outbound_wps) and
                           rx > self.outbound_wps[self.current_idx][0] + SKIP_FORWARD_MARGIN and
                           self.consecutive_skips < MAX_CONSECUTIVE_SKIPS):
                        self.current_idx += 1
                        self.consecutive_skips += 1
                    if self.current_idx != old_idx:
                        self.get_logger().info(
                            f'FORWARD-SKIP wp {old_idx+1}->{self.current_idx+1} '
                            f'(robot x={rx:.0f} > wp x={wx:.0f}, '
                            f'consec={self.consecutive_skips})')
                        self._cancel_and_send_next()
                        return

        # --- Timeout skip (outbound only): stuck too long ---
        if self.phase == "outbound":
            elapsed = time.time() - self.wp_start_time
            if elapsed > SKIP_TIMEOUT:
                if self.consecutive_skips < MAX_CONSECUTIVE_SKIPS:
                    self.get_logger().warn(
                        f'TIMEOUT-SKIP wp {self.current_idx+1} after {elapsed:.0f}s '
                        f'(consec={self.consecutive_skips+1})')
                    self.current_idx += 1
                    self.consecutive_skips += 1
                    self._retries = 0
                    self._cancel_and_send_next()
                else:
                    # Too many consecutive skips - reset and try current wp
                    self.get_logger().warn(
                        f'MAX CONSECUTIVE SKIPS reached, retrying wp {self.current_idx+1}')
                    self.consecutive_skips = 0
                    self.wp_start_time = time.time()

    def _cancel_and_send_next(self):
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_idx >= len(self.current_poses):
            if self.phase == "outbound":
                self.get_logger().info('=== OUTBOUND COMPLETE ===')
                write_phase("removing")
                time.sleep(5.0)
                self.get_logger().info('=== PHASE 2: RETURN (obstacles removed) ===')
                self.phase = "return"
                self.current_poses = self.return_poses
                self.current_idx = 0
                self._retries = 0
                self.consecutive_skips = 0
                self.send_next_goal()
            else:
                self.get_logger().info('=== ALL WAYPOINTS REACHED - NAVIGATION COMPLETE ===')
                write_phase("done")
            return

        # Skip waypoints too far from robot (Nav2 can't plan to them)
        rx, ry = self.get_robot_pose()
        if rx is not None:
            MAX_GOAL_DIST = 15.0  # don't send goals further than this
            skipped = 0
            while self.current_idx < len(self.current_poses):
                wp = self.current_poses[self.current_idx]
                dist = math.hypot(rx - wp.pose.position.x, ry - wp.pose.position.y)
                if dist <= MAX_GOAL_DIST:
                    break
                self.current_idx += 1
                skipped += 1
            if skipped > 0:
                self.get_logger().info(
                    f'[{self.phase}] skipped {skipped} far waypoints (>{MAX_GOAL_DIST:.0f}m)')
            if self.current_idx >= len(self.current_poses):
                # All remaining waypoints too far - finish phase
                if self.phase == "outbound":
                    self.get_logger().info('=== OUTBOUND COMPLETE (remaining wps too far) ===')
                    write_phase("removing")
                    time.sleep(5.0)
                    self.get_logger().info('=== PHASE 2: RETURN (obstacles removed) ===')
                    self.phase = "return"
                    self.current_poses = self.return_poses
                    self.current_idx = 0
                    self._retries = 0
                    self.consecutive_skips = 0
                    self.send_next_goal()
                else:
                    self.get_logger().info('=== NAVIGATION COMPLETE (remaining wps too far) ===')
                    write_phase("done")
                return

        self.wp_start_time = time.time()
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
            self.get_logger().warn(f'goal {self.current_idx + 1} rejected, skipping')
            self.current_idx += 1
            self.consecutive_skips += 1
            self._retries = 0
            self.send_next_goal()
            return
        self.current_goal_handle = goal_handle
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
        self.current_goal_handle = None
        result = future.result().result
        if result.error_code == 0:
            # Verify robot is actually near the waypoint (Nav2 reports "reached"
            # for goals outside costmap with distance_remaining=0)
            rx, ry = self.get_robot_pose()
            if rx is not None and self.current_idx < len(self.current_poses):
                wp = self.current_poses[self.current_idx]
                dist = math.hypot(rx - wp.pose.position.x, ry - wp.pose.position.y)
                if dist > 10.0:
                    # Not actually near the goal - Nav2 false positive
                    # Don't skip - retry same goal (robot needs to drive there)
                    self._retries += 1
                    if self._retries >= MAX_NAV2_RETRIES:
                        self.get_logger().warn(
                            f'[{self.phase}] wp {self.current_idx+1} false positive '
                            f'{self._retries}x (dist={dist:.0f}m), skipping')
                        self.current_idx += 1
                        self.consecutive_skips += 1
                        self._retries = 0
                    else:
                        self.get_logger().warn(
                            f'[{self.phase}] wp {self.current_idx+1} false positive '
                            f'(dist={dist:.0f}m), retry {self._retries}/{MAX_NAV2_RETRIES}')
                    self.send_next_goal()
                    return
            self.get_logger().info(
                f'[{self.phase}] waypoint {self.current_idx + 1} reached!')
            self.current_idx += 1
            self._retries = 0
            self.consecutive_skips = 0
            self.send_next_goal()
        else:
            self._retries += 1
            if self._retries >= MAX_NAV2_RETRIES:
                self.get_logger().warn(
                    f'[{self.phase}] wp {self.current_idx + 1} FAILED {self._retries}x, skipping')
                self.current_idx += 1
                self.consecutive_skips += 1
                self._retries = 0
                self.send_next_goal()
            else:
                self.get_logger().info(
                    f'[{self.phase}] wp {self.current_idx + 1} failed '
                    f'(err={result.error_code}), retry {self._retries}/{MAX_NAV2_RETRIES}')
                time.sleep(1.0)
                self.send_next_goal()


def main():
    route = 'road'
    slam_frame = '--slam-frame' in sys.argv

    for i, arg in enumerate(sys.argv):
        if arg == '--route' and i + 1 < len(sys.argv):
            route = sys.argv[i + 1]

    rclpy.init(args=[a for a in sys.argv if a != '--slam-frame'])
    node = Nav2TwoPhase(route, slam_frame=slam_frame)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
