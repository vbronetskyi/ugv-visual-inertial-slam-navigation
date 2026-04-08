#!/usr/bin/env python3
"""hybrid goal sender (v53) - custom Nav2 client with proactive WP projection

the problem: stock Nav2 planner happily routes you to a pose sitting 1 m
inside an inflated obstacle's cost halo, and the controller then scrapes
the obstacle tangentially on approach.  happens every time the planner
doesnt know about a dropped obstacle yet

what this does: for every /global_costmap/costmap update, re-checks all
future waypoints against the live costmap.  any WP in cost >= PROJ_COST_THRESH
gets BFS-projected to the nearest cell with cost < PROJ_COST_THRESH (capped
at PROJ_MAX_SEARCH_M).  sends the PROJECTED WP to Nav2 not the original
two lists kept: original (frozen) and projected (dynamic)

if no free cell found within search radius - WP gets SKIP'd and we move to the next

changelog:
  exp 52: crude version, projected only when robot was within 3 m of WP.  too
          reactive, by then the robot was already commited to approach angle
  exp 53: proactive projection on every costmap update, full future window
  exp 58: added look-ahead skip for WPs inside infl zone with no free cell
  exp 59: detour ring (insert detour WPs around blocked regions).  main version
  exp 60: tried 10 cm precision finisher for final WP.  got 10 cm but some
          routes timed out.  see final-WP policy around line 400
  exp 64: current - exp 59 plus GT open-loop finisher via supervisor signal

dropped exp 58 variant: tangential shift instead of BFS projection.  idea was
to move WPs 2 m along the path tangent out of inflation zone.  in practice
this put WPs right next to the obstacle edge, robot drove within 1 m of cones
- looks terrible and ocassionally clipped them.  BFS to nearest free cell is
the way
"""
import argparse
import csv
import math
import time
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import tf2_ros


class HybridGoalSender(Node):
    def __init__(self, waypoints, goal_timeout=300.0, tolerance=1.5):
        super().__init__('hybrid_goal_sender')
        self.original_wps = list(waypoints)
        self.projected_wps = list(waypoints)
        self.skip_flags = [False] * len(waypoints)
        self.n_wps = len(waypoints)
        self.GOAL_TIMEOUT = goal_timeout
        self.TOLERANCE = tolerance
        self.reached = 0
        self.skipped = 0
        self.current_idx = 0
        self.costmap = None
        self.costmap_info = None
        self.n_projections = 0
        self.n_skips_by_proj = 0

        # Projection parameters
        self.PROJ_COST_THRESH = 30
        self.PROJ_MAX_SEARCH_M = 3.0
        self.PROJ_MAX_SHIFT_M = 1.0

        # v59: WP look-ahead skip + detour
        self.LOOKAHEAD_SKIP_COST = 60
        self.DETOUR_CLEARANCE_M = 5.0          # ring radius around WP; 5 m guarantees clearance of teach-map tree inflation
        self.DETOUR_MAX_COST = 30              # detour candidate cell must have cost < this
        self.LOOKAHEAD_N = 3

        # No hardcoded obstacle list - Nav2 obstacle_layer gets live
        # observations from the depth camera (/depth_points) and updates the
        # costmap online. Robot detours are planned from that costmap alone.
        self.KNOWN_CONES = []
        self.KNOWN_TENT = None
        # Minimum allowed clearance from any known obstacle - WP center
        # must be ≥ this many metres from obstacle edge.
        # robot_radius 0.7 + 0.2 margin = 0.9 m from obstacle edge
        self.KNOWN_CLEARANCE_M = 0.9

        # v59-fix: use map->base_link tf (SLAM pose, consistent with
        # pure_pursuit_path_follower) instead of /tmp/isaac_pose.txt (Isaac
        # GT). Mixing GT with SLAM-frame WPs breaks REACH once drift > TOL.
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        qos_cm = QoSProfile(depth=1,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.costmap_cb, qos_cm)

        self.get_logger().info("Waiting for /compute_path_to_pose action...")
        self.plan_client.wait_for_server()
        self.get_logger().info(f"Ready. Following {self.n_wps} waypoints.")
        self.start_time = time.time()

    def _read_robot_pose(self):
        """Read robot pose from map->base_link tf (SLAM frame, matches
        pure_pursuit + Nav2 planner). Returns (None, None) if tf not ready.
        """
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link',
                                             rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None

    def _make_pose(self, x, y):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        return ps

    def _cell(self, x, y):
        info = self.costmap_info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        return r, c

    def _xy_from_cell(self, r, c):
        info = self.costmap_info
        x = info.origin.position.x + (c + 0.5) * info.resolution
        y = info.origin.position.y + (r + 0.5) * info.resolution
        return x, y

    def _cost_at(self, r, c):
        info = self.costmap_info
        if not (0 <= r < info.height and 0 <= c < info.width):
            return 100  # off-map treated as blocked
        v = int(self.costmap[r, c])
        if v < 0:       # unknown
            return 100
        return v

    def _wp_too_close_to_known(self, x, y):
        """Return True if (x,y) is within KNOWN_CLEARANCE_M of any known
        obstacle (cone radius ≈ 0.3 m, tent rectangle footprint).
        Costmap-independent: catches collisions even before depth sees
        the obstacle.
        """
        # Cones - radius 0.3 m
        for cx, cy in self.KNOWN_CONES:
            if math.hypot(x - cx, y - cy) < 0.3 + self.KNOWN_CLEARANCE_M:
                return True, ('cone', cx, cy)
        # Tent - rectangle; distance from point to nearest edge
        if self.KNOWN_TENT is not None:
            tc = self.KNOWN_TENT['center']
            hx, hy = self.KNOWN_TENT['half_x'], self.KNOWN_TENT['half_y']
            dx = max(0.0, abs(x - tc[0]) - hx)
            dy = max(0.0, abs(y - tc[1]) - hy)
            if math.hypot(dx, dy) < self.KNOWN_CLEARANCE_M:
                return True, ('tent', tc[0], tc[1])
        return False, None

    def _lookahead_cost(self, x, y):
        """Peak cost at and immediately around (x,y) - used to decide skip."""
        if self.costmap is None:
            return 0
        r0, c0 = self._cell(x, y)
        H, W = self.costmap.shape
        if not (0 <= r0 < H and 0 <= c0 < W):
            return 0
        # 3x3 window around cell
        peak = 0
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                r, c = r0 + dr, c0 + dc
                if 0 <= r < H and 0 <= c < W:
                    v = self._cost_at(r, c)
                    if v > peak:
                        peak = v
        return peak

    def _find_detour(self, x, y):
        """Find a safe detour point around (x,y).  Returns (cx,cy) or
        (None,None).

        Tries multiple radii (4 / 5 / 6 / 7 m), samples 24 candidates per
        ring, accepts a candidate only if (a) it is NOT within known
        obstacle clearance, and (b) its costmap cost < DETOUR_MAX_COST.
        Returns the first accepted candidate with lowest cost.
        """
        best = None
        best_cost = 999
        n_samples = 24
        for radius in (4.0, 5.0, 6.0, 7.0):
            for k in range(n_samples):
                ang = 2 * math.pi * k / n_samples
                cx = x + radius * math.cos(ang)
                cy = y + radius * math.sin(ang)
                too_close, _ = self._wp_too_close_to_known(cx, cy)
                if too_close:
                    continue
                cost = self._lookahead_cost(cx, cy) if self.costmap is not None else 0
                if cost < self.DETOUR_MAX_COST and cost < best_cost:
                    best_cost = cost
                    best = (cx, cy)
            if best is not None:
                return best
        return None, None

    def _project_wp(self, x, y):
        """BFS from (x,y) for nearest free cell. Returns (nx, ny, found_bool)."""
        if self.costmap is None:
            return x, y, True
        info = self.costmap_info
        r0, c0 = self._cell(x, y)
        if not (0 <= r0 < info.height and 0 <= c0 < info.width):
            return x, y, True  # off-map - leave alone
        c0v = self._cost_at(r0, c0)
        if c0v < self.PROJ_COST_THRESH:
            return x, y, True  # already OK

        max_cells = int(self.PROJ_MAX_SEARCH_M / info.resolution)
        q = deque([(r0, c0, 0)])
        seen = {(r0, c0)}
        while q:
            r, c, d = q.popleft()
            if d > max_cells:
                continue
            if self._cost_at(r, c) < self.PROJ_COST_THRESH:
                nx, ny = self._xy_from_cell(r, c)
                # v56-B: if projection shift exceeds cap, leave WP as-is so
                # robot attempts original path (keeps closer to teach trajectory)
                shift = math.hypot(nx - x, ny - y)
                if shift > self.PROJ_MAX_SHIFT_M:
                    return x, y, True   # "found but keep original"
                return nx, ny, True
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) in seen:
                    continue
                seen.add((nr, nc))
                q.append((nr, nc, d + 1))
        return x, y, False  # no free cell within search radius

    def costmap_cb(self, msg):
        self.costmap_info = msg.info
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # Re-project all WPs from current index forward
        n_changed = 0
        n_skipped_now = 0
        for i in range(self.current_idx, self.n_wps):
            ox, oy = self.original_wps[i]
            nx, ny, found = self._project_wp(ox, oy)
            if not found:
                if not self.skip_flags[i]:
                    n_skipped_now += 1
                self.skip_flags[i] = True
                # keep projected = original (will be skipped anyway)
                self.projected_wps[i] = (ox, oy)
            else:
                self.skip_flags[i] = False
                if (nx, ny) != tuple(self.projected_wps[i]):
                    n_changed += 1
                self.projected_wps[i] = (nx, ny)
        if n_changed or n_skipped_now:
            self.n_projections += n_changed
            self.n_skips_by_proj += n_skipped_now
            self.get_logger().info(
                f'[PROACTIVE] costmap update: re-projected {n_changed} WPs, '
                f'+{n_skipped_now} skipped (no free cell); '
                f'totals proj={self.n_projections} skip={self.n_skips_by_proj}')

    def request_plan(self, gx, gy):
        rx, ry = self._read_robot_pose()
        if rx is None:
            return None
        goal = ComputePathToPose.Goal()
        goal.goal = self._make_pose(gx, gy)
        goal.start = self._make_pose(rx, ry)
        goal.use_start = False
        goal.planner_id = 'GridBased'
        fut = self.plan_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            return None
        res_fut = h.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=15.0)
        if not res_fut.done():
            return None
        res = res_fut.result()
        if res is None:
            return None
        return res.result.path

    def follow_waypoint(self, i, x, y):
        t0 = time.time()
        last_replan = 0
        REPLAN_PERIOD = 5.0        # v55: faster replan at 0.8 m/s
        n_plan_fails = 0
        # Final-WP policy: don't give up on the last few WPs - they are needed
        # for route closure. The last 5 WPs get 2× timeout and no plan-fail SKIP.
        is_final_wp = (i >= self.n_wps - 5)
        MAX_PLAN_FAILS = 9999 if is_final_wp else 5
        wp_timeout = self.GOAL_TIMEOUT * 2 if is_final_wp else self.GOAL_TIMEOUT
        while time.time() - t0 < wp_timeout:
            # User projected position (may have moved since last tick)
            px, py = self.projected_wps[i]
            if self.skip_flags[i]:
                self.get_logger().warn(
                    f'  WP {i} SKIP - no free cell within '
                    f'{self.PROJ_MAX_SEARCH_M} m of ({self.original_wps[i][0]:.1f},'
                    f'{self.original_wps[i][1]:.1f})')
                return False
            rx, ry = self._read_robot_pose()
            if rx is None:
                time.sleep(0.5); continue
            d = math.hypot(px - rx, py - ry)
            if d < self.TOLERANCE:
                # print(f"DEBUG pose={pose}")
                self.get_logger().info(f"  WP {i} REACHED (d={d:.1f}m)")
                return True
            # v59 continuous lookahead: only abort if VERY close to
            # unsafe target (d<3m) AND known obstacle proximity.  Costmap
            # cost alone can spike from teach-map tree inflation - don't
            # abort on that.
            if d < 3.0:
                too_close, _ = self._wp_too_close_to_known(px, py)
                if too_close:
                    self.get_logger().warn(
                        f'  WP {i} LATE-DETECTED near known obstacle '
                        f'(d={d:.1f}m) - abandoning')
                    return False
            if time.time() - last_replan > REPLAN_PERIOD:
                path = self.request_plan(px, py)
                if path is not None and len(path.poses) > 1:
                    self.plan_pub.publish(path)
                    tag = '(projected)' if (px, py) != tuple(self.original_wps[i]) else ''
                    self.get_logger().info(
                        f'  WP {i}: plan {len(path.poses)} poses, d={d:.1f}m {tag}')
                    last_replan = time.time()
                    n_plan_fails = 0
                else:
                    n_plan_fails += 1
                    self.get_logger().warn(
                        f'  WP {i}: plan failed ({n_plan_fails}/{MAX_PLAN_FAILS})')
                    last_replan = time.time()
                    if n_plan_fails >= MAX_PLAN_FAILS:
                        self.get_logger().warn(
                            f'  WP {i} SKIP (plan failed {MAX_PLAN_FAILS}× - '
                            f'target unreachable, moving on)')
                        return False
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().warn(f'  WP {i} TIMEOUT (d={d:.1f}m)')
        return False

    def run(self):
        # Wait for map->base_link tf (listener needs a few spin cycles).
        for _ in range(40):
            rclpy.spin_once(self, timeout_sec=0.25)
            rx0, ry0 = self._read_robot_pose()
            if rx0 is not None:
                break
        turn_idx = max(range(self.n_wps), key=lambda i: self.original_wps[i][0])
        if rx0 is not None:
            start = min(range(turn_idx + 1),
                        key=lambda i: math.hypot(
                            self.original_wps[i][0] - rx0,
                            self.original_wps[i][1] - ry0))
            self.get_logger().info(
                f"Robot at ({rx0:.1f},{ry0:.1f}) - start WP {start}/{turn_idx}")
        else:
            start = 0

        for i in range(start, self.n_wps):
            self.current_idx = i
            x, y = self.projected_wps[i]
            if self.skip_flags[i]:
                # print(f"DEBUG matches={matches}")
                self.get_logger().warn(
                    f'WP {i}/{self.n_wps - 1}: SKIP (projection failed)')
                self.skipped += 1
                continue

            # v59 LOOK-AHEAD + KNOWN-OBSTACLE CHECK:
            # (a) hardcoded check against known cone/tent positions - no
            #     costmap latency, fires regardless of depth visibility;
            # (b) if costmap is available, also check cell cost.
            # Final WPs bypass this guard (ghost inflation after FIRE is
            # spurious - obstacles were removed; follow_waypoint keeps
            # replanning until it succeeds or the 2× timeout elapses).
            is_final_wp = (i >= self.n_wps - 5)
            unsafe_reason = None
            if not is_final_wp:
                too_close, what = self._wp_too_close_to_known(x, y)
                if too_close:
                    unsafe_reason = f'near_{what[0]}@({what[1]:.1f},{what[2]:.1f})'
                elif self.costmap is not None:
                    wp_cost = self._lookahead_cost(x, y)
                    if wp_cost >= self.LOOKAHEAD_SKIP_COST:
                        unsafe_reason = f'costmap_cost={wp_cost}'
            if unsafe_reason is not None:
                dx, dy = self._find_detour(x, y)
                if dx is not None:
                    # print(f"DEBUG matches={matches}")
                    self.get_logger().warn(
                        f'WP {i}/{self.n_wps - 1}: unsafe ({unsafe_reason}) '
                        f'-> DETOUR to ({dx:.1f},{dy:.1f})')
                    if self.follow_waypoint(i, dx, dy):
                        self.reached += 1
                    else:
                        self.skipped += 1
                    continue
                else:
                    self.get_logger().warn(
                        f'WP {i}/{self.n_wps - 1}: unsafe ({unsafe_reason}), '
                        f'no detour found - SKIP')
                    self.skipped += 1
                    continue

            rx, ry = self._read_robot_pose()
            if rx is not None and math.hypot(x - rx, y - ry) < self.TOLERANCE:
                self.reached += 1
                self.get_logger().info(f"WP {i}/{self.n_wps - 1} already near")
                continue
            tag = '(projected)' if (x, y) != tuple(self.original_wps[i]) else ''
            self.get_logger().info(
                f"WP {i}/{self.n_wps - 1}: ({x:.1f},{y:.1f}) {tag}")
            if self.follow_waypoint(i, x, y):
                self.reached += 1
                continue
            # WP failed - try detour fallback
            dx, dy = self._find_detour(x, y)
            if dx is not None:
                self.get_logger().warn(
                    f'WP {i}/{self.n_wps - 1}: failed -> DETOUR fallback '
                    f'to ({dx:.1f},{dy:.1f})')
                if self.follow_waypoint(i, dx, dy):
                    self.reached += 1
                    continue
            self.skipped += 1

        total = time.time() - self.start_time
        self.get_logger().info('=' * 50)
        self.get_logger().info(
            f"RESULT: reached {self.reached}/{self.n_wps} "
            f"skipped {self.skipped} duration {total:.0f}s "
            f"projections={self.n_projections} skip_by_proj={self.n_skips_by_proj}")
        # print("DEBUG: entering main loop")
        self.get_logger().info('=' * 50)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trajectory', required=True)
    ap.add_argument('--spacing', type=float, default=4.0)
    ap.add_argument('--goal-timeout', type=float, default=300.0)
    ap.add_argument('--tolerance', type=float, default=1.5)
    args = ap.parse_args()

    pts = []
    with open(args.trajectory) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['gt_x']), float(row['gt_y'])))
    wps = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - wps[-1][0], p[1] - wps[-1][1]) >= args.spacing:
            wps.append(p)
    print(f"Loaded {len(pts)} poses -> {len(wps)} goals @ {args.spacing}m")

    rclpy.init()
    node = HybridGoalSender(wps, args.goal_timeout, args.tolerance)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
