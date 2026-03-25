#!/usr/bin/env python3
"""
Two-phase waypoint sender for Nav2.

Supports two modes:
  --slam-frame: load SLAM-frame waypoints from /tmp/slam_waypoints.json
  default: load world-frame waypoints from /tmp/slam_routes.json

Adaptive speed based on drift from mapped trajectory.
Waypoint skipping when robot passes waypoint along forward axis.

usage:
  source /opt/ros/jazzy/setup.bash
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
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


PHASE_FILE = "/tmp/nav2_phase.json"

# drift-based speed
DRIFT_LOW = 2.0
DRIFT_MED = 4.0
DRIFT_HIGH = 7.0
SPEED_FULL = 0.8
SPEED_SLOW = 0.4
SPEED_CRAWL = 0.2

WAYPOINT_SKIP_MARGIN = 3.0
CHECK_INTERVAL = 2.0
WAYPOINT_TIMEOUT = 20.0  # seconds - skip if robot stuck on a waypoint too long
# Lateral corridor: if robot drifts > MAX_LATERAL_DEV from the waypoint
# line, send an intermediate goal back onto the path.
MAX_LATERAL_DEV = 3.0
RETURN_TO_PATH_DIST = 1.5
# Forward regression watchdog (exp 26): if robot's max-x-reached doesn't
# increase for REGRESSION_TIMEOUT seconds, we're in a recovery loop.
REGRESSION_TIMEOUT = 30.0
FORWARD_SKIP_DISTANCE = 5.0


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

        self._wp_start_time = 0.0
        # track robot position at goal send to detect "stuck" false-positive cascade
        self._goal_send_rx = None
        self._goal_send_ry = None
        self._goal_send_time = 0.0
        # lateral corridor state: 'NAVIGATING' or 'RETURNING_TO_PATH'
        self._corridor_state = 'NAVIGATING'
        self._return_target = None
        # Forward-regression watchdog state
        self._max_x_reached = -1e9
        self._max_x_time = time.time()
        self._regression_escape_count = 0
        # Service clients for clearing costmaps (used by regression watchdog)
        self._clear_local_cli = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global_cli = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

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

    def _current_wps(self):
        """Return the list of (x,y) waypoints for the current phase."""
        if self.phase == "outbound":
            return self.outbound_wps
        # return phase: reverse of outbound as pairs
        return [(p.pose.position.x, p.pose.position.y) for p in self.return_poses]

    def get_lateral_deviation_fullpath(self, rx, ry):
        """Find closest segment across the ENTIRE path (not just current_idx window).

        Returns (min_dist, closest_seg_idx, closest_point_xy). Exp 26 showed
        that window-based search misses the drift case where watchdog
        advances current_idx past the robot's actual SLAM position.
        """
        wps = self._current_wps()
        if len(wps) < 2:
            return 0.0, 0, (rx, ry)
        best_d = float('inf')
        best_idx = 0
        best_pt = (wps[0][0], wps[0][1])
        for i in range(len(wps) - 1):
            wx1, wy1 = wps[i]
            wx2, wy2 = wps[i + 1]
            dx = wx2 - wx1
            dy = wy2 - wy1
            length2 = dx * dx + dy * dy
            if length2 < 0.01:
                d = math.hypot(rx - wx1, ry - wy1)
                cx, cy = wx1, wy1
            else:
                t = max(0.0, min(1.0, ((rx - wx1) * dx + (ry - wy1) * dy) / length2))
                cx = wx1 + t * dx
                cy = wy1 + t * dy
                d = math.hypot(rx - cx, ry - cy)
            if d < best_d:
                best_d = d
                best_idx = i
                best_pt = (cx, cy)
        return best_d, best_idx, best_pt

    def get_lateral_deviation(self, rx, ry):
        """Backward-compatible lateral dev - returns just the distance."""
        dist, _, _ = self.get_lateral_deviation_fullpath(rx, ry)
        return dist

    def snap_current_idx_to_robot(self, rx, ry):
        """Set current_idx to the closest waypoint that is ahead of the robot.

        Avoids the exp 26 bug where watchdog advanced current_idx far ahead
        of robot's actual SLAM position, breaking lateral check and goal
        tracking.
        """
        wps = self._current_wps()
        best_idx = self.current_idx
        best_d = float('inf')
        direction = 1 if self.phase == "outbound" else -1
        for i, (wx, wy) in enumerate(wps):
            # must be approximately ahead (or very close)
            forward = (wx - rx) * direction
            if forward > -2.0:
                d = math.hypot(rx - wx, ry - wy)
                if d < best_d:
                    best_d = d
                    best_idx = i
        return best_idx

    def _clear_costmaps_async(self):
        """Fire-and-forget clear costmap service calls."""
        req = ClearEntireCostmap.Request()
        if self._clear_local_cli.service_is_ready():
            self._clear_local_cli.call_async(req)
        if self._clear_global_cli.service_is_ready():
            self._clear_global_cli.call_async(req)

    def _handle_regression(self, rx, ry):
        """Forward-regression recovery: clear costmaps, snap idx, skip ahead.

        Called from monitor_tick when robot's max x hasn't advanced for
        REGRESSION_TIMEOUT seconds (recovery loop).
        """
        self._regression_escape_count += 1
        stuck_dur = time.time() - self._max_x_time
        self.get_logger().warn(
            f'FORWARD REGRESSION #{self._regression_escape_count}: '
            f'no progress for {stuck_dur:.0f}s, max_x={self._max_x_reached:.1f}, '
            f'robot_x={rx:.1f} - breaking out')

        # 1. Clear costmaps (global + local)
        self._clear_costmaps_async()

        # 2. Snap current_idx to the robot's actual SLAM position (exp 27 fix)
        old_idx = self.current_idx
        snapped_idx = self.snap_current_idx_to_robot(rx, ry)
        if snapped_idx != self.current_idx:
            self.get_logger().warn(
                f'  snapped wp {old_idx+1} -> {snapped_idx+1} '
                f'(closest to robot at ({rx:.1f},{ry:.1f}))')
            self.current_idx = snapped_idx

        # 3. Skip waypoints: find first wp at least FORWARD_SKIP_DISTANCE ahead
        wps = self._current_wps()
        snap_idx = self.current_idx
        target_x = rx + FORWARD_SKIP_DISTANCE
        while self.current_idx < len(wps) and wps[self.current_idx][0] < target_x:
            self.current_idx += 1
        if self.current_idx >= len(wps):
            self.current_idx = len(wps) - 1
        self.get_logger().warn(
            f'  advanced wp {snap_idx+1} -> {self.current_idx+1} '
            f'(target_x={target_x:.1f})')

        # 3. Reset watchdog timer and corridor state
        self._max_x_reached = rx
        self._max_x_time = time.time()
        self._wp_start_time = time.time()
        self._retries = 0
        self._corridor_state = 'NAVIGATING'
        self._return_target = None

        # 4. Send goal to new target wp
        self.send_next_goal()

    def get_return_point(self, rx, ry):
        """Closest point on the path to the robot - for return-to-path goal."""
        wps = self._current_wps()
        if len(wps) < 2:
            return rx, ry
        start = max(0, self.current_idx - 2)
        end = min(len(wps) - 1, self.current_idx + 3)
        best_d = float('inf')
        best_pt = (wps[min(self.current_idx, len(wps)-1)][0],
                   wps[min(self.current_idx, len(wps)-1)][1])
        for i in range(start, end):
            wx1, wy1 = wps[i]
            wx2, wy2 = wps[i + 1]
            dx = wx2 - wx1
            dy = wy2 - wy1
            length2 = dx * dx + dy * dy
            if length2 < 0.01:
                continue
            t = max(0.0, min(1.0, ((rx - wx1) * dx + (ry - wy1) * dy) / length2))
            cx = wx1 + t * dx
            cy = wy1 + t * dy
            d = math.hypot(rx - cx, ry - cy)
            if d < best_d:
                best_d = d
                best_pt = (cx, cy)
        return best_pt

    def get_target_speed(self, drift):
        if drift < DRIFT_LOW:
            return SPEED_FULL
        elif drift < DRIFT_MED:
            return SPEED_SLOW
        elif drift < DRIFT_HIGH:
            return SPEED_CRAWL
        else:
            return SPEED_CRAWL

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

        self._wp_start_time = time.time()
        self.monitor_timer = self.create_timer(CHECK_INTERVAL, self.monitor_tick)
        self.send_next_goal()

    def _sanitize_waypoints(self, waypoints, max_lateral_dev=4.0, window=10):
        """Clamp Y deviation using sliding window mean.

        SLAM mapping run can have heavy lateral drift at scale. Global mean
        doesn't work when route has natural bends (e.g. road curves). Instead,
        clamp each waypoint Y to the local window mean ± max_lateral_dev.
        """
        if not waypoints:
            return waypoints
        clamped = []
        n_clamped = 0
        for i, (x, y) in enumerate(waypoints):
            start = max(0, i - window)
            end = min(len(waypoints), i + window + 1)
            local_mean_y = sum(wp[1] for wp in waypoints[start:end]) / (end - start)
            clamped_y = max(local_mean_y - max_lateral_dev,
                            min(local_mean_y + max_lateral_dev, y))
            if abs(clamped_y - y) > 0.01:
                n_clamped += 1
            clamped.append([x, round(clamped_y, 3)])
        self.get_logger().info(
            f'Sanitized waypoints (window={window}, max_dev={max_lateral_dev}m): '
            f'clamped {n_clamped}/{len(waypoints)}')
        return clamped

    def _load_slam_frame_waypoints(self):
        """Load waypoints in SLAM coordinate frame."""
        with open('/tmp/slam_waypoints.json') as f:
            data = json.load(f)

        outbound_raw = self._sanitize_waypoints(data['outbound'], max_lateral_dev=5.0)
        return_raw = self._sanitize_waypoints(data['return'], max_lateral_dev=5.0)

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
        """Load waypoints in world frame (legacy)."""
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

        # Forward-regression watchdog (exp 26)
        if self.phase == "outbound":
            if rx > self._max_x_reached:
                self._max_x_reached = rx
                self._max_x_time = time.time()
            elif time.time() - self._max_x_time > REGRESSION_TIMEOUT:
                self._handle_regression(rx, ry)
                return

        # Lateral corridor enforcement (full-path closest segment search)
        lat_dev, closest_seg_idx, closest_pt = self.get_lateral_deviation_fullpath(rx, ry)
        if self._corridor_state == 'NAVIGATING':
            if lat_dev > MAX_LATERAL_DEV:
                ret_x, ret_y = closest_pt
                self.get_logger().warn(
                    f'LATERAL DEV {lat_dev:.1f}m > {MAX_LATERAL_DEV}m, '
                    f'closest_seg={closest_seg_idx+1}, '
                    f'returning to ({ret_x:.1f},{ret_y:.1f})')
                # Snap current_idx to closest segment so the subsequent
                # navigation flow picks up from the right place
                self.current_idx = closest_seg_idx
                self._corridor_state = 'RETURNING_TO_PATH'
                self._return_target = (ret_x, ret_y)
                yaw = math.atan2(ret_y - ry, ret_x - rx)
                pose = make_pose(ret_x, ret_y, yaw)
                goal = NavigateToPose.Goal()
                goal.pose = pose
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                self._goal_send_rx = rx
                self._goal_send_ry = ry
                self._goal_send_time = time.time()
                self._wp_start_time = time.time()
                future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
                future.add_done_callback(self.goal_response_cb)
                return
        elif self._corridor_state == 'RETURNING_TO_PATH':
            if lat_dev < RETURN_TO_PATH_DIST:
                self.get_logger().info(
                    f'Back on path (lat_dev={lat_dev:.1f}m), resuming navigation')
                self._corridor_state = 'NAVIGATING'
                self._return_target = None
                self.send_next_goal()
                return

        # skip waypoints robot has passed (forward = +X in both frames)
        if self.phase == "outbound" and self.current_idx < len(self.outbound_wps):
            wx = self.outbound_wps[self.current_idx][0]
            if rx > wx + WAYPOINT_SKIP_MARGIN:
                old_idx = self.current_idx
                while (self.current_idx < len(self.outbound_wps) and
                       rx > self.outbound_wps[self.current_idx][0] + WAYPOINT_SKIP_MARGIN):
                    self.current_idx += 1
                if self.current_idx != old_idx:
                    self.get_logger().info(
                        f'Skipped wp {old_idx+1}->{self.current_idx+1} '
                        f'(robot x={rx:.0f} passed wp x={wx:.0f})')
                    self._wp_start_time = time.time()
                    self.send_next_goal()
                    return

        # Timeout: if robot stuck on a waypoint too long, skip to next
        if time.time() - self._wp_start_time > WAYPOINT_TIMEOUT:
            if self.current_idx + 1 < len(self.current_poses):
                next_wp = self.current_poses[self.current_idx + 1]
                nx = next_wp.pose.position.x
                # only skip forward (prevent oscillation on return)
                forward_ok = (nx > rx - 1.0) if self.phase == "outbound" else (nx < rx + 1.0)
                if forward_ok:
                    self.get_logger().warn(
                        f'[{self.phase}] wp {self.current_idx+1} TIMEOUT '
                        f'after {WAYPOINT_TIMEOUT:.0f}s - skipping')
                    self.current_idx += 1
                    self._wp_start_time = time.time()
                    self._retries = 0
                    # Cancel stale corridor state - timeout means whatever we
                    # were doing (incl. returning to path) failed, start fresh
                    self._corridor_state = 'NAVIGATING'
                    self._return_target = None
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

        # Remember position at goal send time to detect no-progress false positives
        rx, ry = self.get_robot_pose()
        self._goal_send_rx = rx
        self._goal_send_ry = ry
        self._goal_send_time = time.time()

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

    def _find_next_reachable_wp(self, rx, ry, start_idx):
        """Find next waypoint forward from robot, within lateral reach."""
        wps = self.outbound_wps if self.phase == "outbound" else None
        if wps is None:
            return start_idx + 1
        for i in range(start_idx + 1, len(self.current_poses)):
            wp = self.current_poses[i]
            wx, wy = wp.pose.position.x, wp.pose.position.y
            # waypoint must be ahead (or near robot X)
            if wx > rx - 2.0:
                lateral = abs(ry - wy)
                if lateral < 8.0:
                    return i
        return len(self.current_poses)

    def result_cb(self, future):
        result = future.result().result
        if result.error_code == 0:
            # Verify reached: Nav2 false-positives goals by returning success
            # without moving (e.g. controller abort when stuck).
            rx, ry = self.get_robot_pose()
            if rx is not None and self.current_idx < len(self.current_poses):
                wp = self.current_poses[self.current_idx]
                wx, wy = wp.pose.position.x, wp.pose.position.y
                real_dist = math.hypot(rx - wx, ry - wy)
                # Did robot actually move since goal send?
                moved = 0.0
                if self._goal_send_rx is not None:
                    moved = math.hypot(rx - self._goal_send_rx,
                                       ry - self._goal_send_ry)
                goal_age = time.time() - self._goal_send_time

                # Case 1: robot didn't move AND result came quickly -> Nav2 aborted
                # (e.g. stuck, path unreachable). Retry this same wp after delay.
                if moved < 0.5 and goal_age < 2.0 and real_dist > 2.5:
                    self._retries += 1
                    if self._retries >= 10:
                        self.get_logger().warn(
                            f'[{self.phase}] wp {self.current_idx+1} STUCK '
                            f'(moved={moved:.1f}m, dist={real_dist:.1f}m), '
                            f'skipping after {self._retries} no-progress retries')
                        self.current_idx += 1
                        self._retries = 0
                        self._wp_start_time = time.time()
                        time.sleep(1.0)
                        self.send_next_goal()
                    else:
                        self.get_logger().info(
                            f'[{self.phase}] wp {self.current_idx+1} no-progress '
                            f'retry {self._retries}/10 (moved={moved:.1f}m dist={real_dist:.1f}m)')
                        time.sleep(2.0)
                        self.send_next_goal()
                    return

                # Case 2: robot moved but result reported success far from wp.
                # Trust that the robot is in the right general area; advance.
                if real_dist > 5.0:
                    self.get_logger().warn(
                        f'[{self.phase}] wp {self.current_idx+1} FALSE POSITIVE '
                        f'(dist={real_dist:.1f}m, moved={moved:.1f}m), advancing')

            if self._corridor_state == 'RETURNING_TO_PATH':
                # return goal done - don't advance waypoint index
                self.get_logger().info(
                    f'[{self.phase}] return-to-path goal result received')
                # monitor_tick will transition state when lat_dev small enough
                return
            self.get_logger().info(
                f'[{self.phase}] waypoint {self.current_idx + 1} reached!')
            self.current_idx += 1
            self._retries = 0
            self._wp_start_time = time.time()
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
