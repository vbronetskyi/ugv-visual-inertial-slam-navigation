#!/usr/bin/env python3
"""Stock-Nav2 baseline client v2 - via FollowWaypoints action.

The canonical Nav2 way to 'visit a list of waypoints with skip-on-failure'
is the WaypointFollower action (stop_on_failure: false). Each WP is
delegated to NavigateToPose internally; if one fails, the follower logs it
and moves to the next.

Compared to v1 (NavigateThroughPoses single-trajectory) this survives
individual WP failures instead of aborting the whole route.
"""
import argparse, csv, math, time, sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import FollowWaypoints
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


def subsample(traj_path, spacing):
    pts = []
    with open(traj_path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['gt_x']), float(row['gt_y'])))
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing:
            out.append(p)
    return out


def robot_pose_from_tmp():
    try:
        with open('/tmp/isaac_pose.txt') as f:
            t = f.readline().strip().split()
            return float(t[0]), float(t[1])
    except Exception:
        return None


def skip_initial_reached(wps, robot_xy, tol=3.5):
    # FIXME: hardcoded spawn, read from routes.json
    """Drop leading WPs already within `tol` of robot pose. Stock Nav2
    goal_checker declares them reached but BT gets stuck in wait
    recovery with trivial single-pose plans."""
    if robot_xy is None:
        return wps
    rx, ry = robot_xy
    i = 0
    while i < len(wps) and math.hypot(wps[i][0] - rx, wps[i][1] - ry) < tol:
        i += 1
    return wps[i:]


def pose_from_xy(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    return p


class WaypointClient(Node):
    # WP-projection skip logic (canonical stock-Nav2 extension):
    # before sending, peek global costmap at every WP.  If cell cost ≥
    # LETHAL_INFLATED, try nearest free cell within PROJ_RADIUS_M; if
    # nothing found, drop that WP altogether (so robot doesn't stall
    # at an unreachable goal in forest inflation).
    LETHAL_INFLATED = 70
    PROJ_RADIUS_M = 2.0

    def __init__(self, wps):
        super().__init__('waypoint_follower_client')
        self.wps = wps
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.last_idx = -1
        self.start = time.time()
        self.costmap = None
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cm_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._on_costmap, qos)

    def _on_costmap(self, msg):
        self.costmap = msg

    def _wait_costmap(self, timeout=30.0):
        t0 = time.time()
        while rclpy.ok() and self.costmap is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.costmap is not None

    def _cost_at(self, x, y):
        if self.costmap is None:
            return 0
        info = self.costmap.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        c = int((x - ox) / res); r = int((y - oy) / res)
        if not (0 <= r < info.height and 0 <= c < info.width):
            return 100
        return self.costmap.data[r * info.width + c] & 0xff

    def _project(self, x, y):
        """Find nearest free cell within PROJ_RADIUS_M. Returns (x,y,ok)."""
        if self.costmap is None:
            return x, y, True
        info = self.costmap.info
        res = info.resolution
        max_cells = int(self.PROJ_RADIUS_M / res) + 1
        best = None
        best_d = None
        for dr in range(-max_cells, max_cells + 1):
            for dc in range(-max_cells, max_cells + 1):
                if dr * dr + dc * dc > max_cells * max_cells:
                    continue
                nx, ny = x + dc * res, y + dr * res
                if self._cost_at(nx, ny) < 50:
                    d = math.hypot(nx - x, ny - y)
                    if best_d is None or d < best_d:
                        best = (nx, ny); best_d = d
        if best is None:
            return x, y, False
        return best[0], best[1], True

    def _prepare_poses(self):
        """Apply cost-lookahead projection + skip to each WP."""
        if not self._wait_costmap():
            self.get_logger().warn('no global_costmap received; skipping projection')
            projected = [(x, y, True) for (x, y) in self.wps]
        else:
            projected = []
            projected_n = skipped_n = 0
            for i, (x, y) in enumerate(self.wps):
                c = self._cost_at(x, y)
                if c < self.LETHAL_INFLATED:
                    projected.append((x, y, True))
                    continue
                px, py, ok = self._project(x, y)
                if ok:
                    projected.append((px, py, True))
                    projected_n += 1
                else:
                    projected.append((x, y, False))  # drop
                    skipped_n += 1
            self.get_logger().info(
                f'WP projection: projected {projected_n}, dropped {skipped_n} '
                f'of {len(self.wps)} WPs')
        poses = []
        self.wps_final = []
        for i, (x, y, keep) in enumerate(projected):
            if not keep:
                continue
            yaw = 0.0
            if i + 1 < len(projected):
                nx, ny, _ = projected[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            poses.append(pose_from_xy(x, y, yaw))
            self.wps_final.append((x, y))
        return poses

    def run(self):
        self.get_logger().info(f'Waiting for /follow_waypoints server...')
        if not self.client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('FollowWaypoints server did not come up')
            return 1
        poses = self._prepare_poses()
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        # print(f"DEBUG state={state} pose={pose}")
        self.get_logger().info(
            f'Sending {len(poses)} WPs to stock Nav2 FollowWaypoints action')
        fut = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('goal rejected')
            return 1
        self.get_logger().info('goal accepted, awaiting result...')
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result()
        dur = int(time.time() - self.start)
        if result is None:
            self.get_logger().warn(f'RESULT: no-result duration {dur}s')
            return 1
        missed = list(getattr(result.result, 'missed_waypoints', []))
        n_total = len(self.wps)
        n_skipped = len(missed)
        n_reached = n_total - n_skipped
        self.get_logger().info(
            f'RESULT: reached {n_reached}/{n_total} '
            f'skipped {n_skipped} duration {dur}s '
            f'missed_idx={missed}')
        return 0

    def _on_feedback(self, msg):
        fb = msg.feedback
        idx = int(getattr(fb, 'current_waypoint', -1))
        dur = int(time.time() - self.start)
        if idx != self.last_idx:
            self.last_idx = idx
            wps_list = self.wps_final if hasattr(self, 'wps_final') else self.wps
            x, y = wps_list[idx] if 0 <= idx < len(wps_list) else (0.0, 0.0)
            self.get_logger().info(
                f'  WP progress  idx={idx}/{len(wps_list)-1}  '
                f'target=({x:.1f},{y:.1f})  t={dur}s')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    args = ap.parse_args()

    wps = subsample(args.trajectory, args.spacing)
    print(f'Subsampled to {len(wps)} WPs at {args.spacing}m spacing')
    robot_xy = robot_pose_from_tmp()
    wps_after = skip_initial_reached(wps, robot_xy, tol=3.5)
    # print(f"DEBUG: ran {len(ran)} waypoints")
    print(f'Skipped {len(wps) - len(wps_after)} leading WP(s) already within 3.5 m of robot '
          f'pose {robot_xy} - stock Nav2 gets stuck on trivial plans.')
    wps = wps_after

    rclpy.init()
    node = WaypointClient(wps)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)
