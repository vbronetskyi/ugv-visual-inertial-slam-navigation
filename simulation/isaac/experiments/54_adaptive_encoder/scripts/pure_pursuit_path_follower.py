#!/usr/bin/env python3
"""Pure pursuit follower with (v9) anti-spin + (v53) proximity speed limiter.

Proximity speed limiter:
    Subscribes to /global_costmap/costmap. Before every cmd publish,
    samples costs in a forward arc (0.3–1.5 m ahead of robot, ±0.3 m
    lateral). Caps cmd.linear.x based on the MAX cost seen:

        cost < 30   -> full speed (MAX_VEL)
        30 ≤ cost < 70  -> 0.15 m/s (slowdown)
        70 ≤ cost < 99  -> 0.08 m/s (crawl - near inflation edge)
        cost ≥ 99 or unknown(-1)  -> 0.03 m/s (near stop)

    Inflation radius is 1.2 m, so cost 30 roughly starts 0.7–0.9 m from
    the occupied center - robot begins slowing BEFORE contact range.
    This gives SLAM time to converge and planner time to re-plan, and
    removes the run 4 failure mode (driving into tent while SLAM
    drifts).

v9 anti-spin layer is preserved unchanged.
"""
import argparse
import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PurePursuitFollower(Node):
    def __init__(self, lookahead=2.0, max_vel=0.25, goal_tol=0.5):
        super().__init__('pure_pursuit_follower')
        self.LOOKAHEAD = lookahead
        self.MAX_VEL = max_vel
        self.GAIN_ANG = 1.2
        self.MAX_ANG = 0.8
        self.GOAL_TOL = goal_tol

        # v9 anti-spin
        self.SPIN_W_THRESH = 0.5
        self.SPIN_V_THRESH = 0.05
        self.SPIN_LIMIT_S = 5.0
        self.SPIN_COOLDOWN_S = 3.0
        self.PROGRESS_WINDOW_S = 5.0
        self.MIN_PROGRESS_M = 0.5

        # v54 wedge-recovery - robot wedged against obstacle (physical contact).
        # If we've been emitting cmd.v > 0 for WEDGE_WINDOW_S but GT displacement
        # is < WEDGE_MIN_DISP_M, reverse for WEDGE_BACKUP_S to get clearance,
        # then PP resumes.
        self.WEDGE_WINDOW_S = 4.0
        self.WEDGE_MIN_DISP_M = 0.15
        self.WEDGE_BACKUP_S = 2.5
        self.WEDGE_BACKUP_V = -0.25
        self.wedge_backup_until = 0.0
        self.wedge_activations = 0

        # v54 proximity speed profile - real Husky cruise at ~1 m/s,
        # slow to 0.5 m/s near obstacles, 0.2 m/s at true contact range.
        # Faster cruise ⇒ shorter mission time ⇒ less IMU bias walk ⇒
        # tighter VIO. Simpler 2-tier ruleset instead of 3.
        self.PROX_SAMPLE_DIST = [0.4, 0.9, 1.4]     # 0.4-1.4 m ahead (longer for 1 m/s stopping)
        self.PROX_SAMPLE_LAT = [-0.15, 0.0, 0.15]
        self.PROX_COST_SLOW = 50
        self.PROX_COST_LETHAL = 99
        self.V_SLOW = 0.5
        self.V_LETHAL = 0.2

        self.path = None
        self.path_idx = 0
        self.path_done = False
        self.costmap = None
        self.costmap_info = None

        self.spin_accum_t = 0.0
        self.cooldown_until = 0.0
        self.pos_history = []

        self.create_subscription(Path, '/plan', self.path_cb, 10)
        qos_cm = QoSProfile(depth=1,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self.costmap_cb, qos_cm)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.timer = self.create_timer(0.1, self.tick)
        self.log_counter = 0
        self.anti_spin_activations = 0
        self.prox_activations = 0

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        self.path = msg
        self.path_idx = 0
        self.path_done = False
        self.get_logger().info(f'New path: {len(msg.poses)} poses, '
            f'end=({msg.poses[-1].pose.position.x:.1f},'
            f'{msg.poses[-1].pose.position.y:.1f})')

    def costmap_cb(self, msg):
        self.costmap_info = msg.info
        self.costmap = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _costmap_cell(self, x, y):
        if self.costmap is None:
            return 0
        info = self.costmap_info
        c = int((x - info.origin.position.x) / info.resolution)
        r = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= r < info.height and 0 <= c < info.width):
            return 0
        return int(self.costmap[r, c])

    def _max_cost_ahead(self, rx, ry, ryaw):
        """Sample along the current plan poses (not a straight heading arc).

        Rationale: the planner already routes around obstacles by going
        through cells with cost < inscribed. Sampling the plan reports
        "how costly is the path the robot is about to execute" rather
        than "how costly is the space straight ahead". With heading arc
        sampling, every teach-map tree near the route triggers LETHAL
        even when the plan curves cleanly around it.

        Fallback: if no plan yet, sample the heading arc as before.
        """
        if self.costmap is None:
            return 0
        if self.path is None or len(self.path.poses) < 2:
            return self._max_cost_heading_arc(rx, ry, ryaw)
        max_cost = 0
        # Walk plan poses from current lookahead index, accumulating arc length
        start = max(0, min(self.path_idx, len(self.path.poses) - 1))
        arc = 0.0
        last_x, last_y = rx, ry
        for i in range(start, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y
            arc += math.hypot(px - last_x, py - last_y)
            last_x, last_y = px, py
            if arc < self.PROX_SAMPLE_DIST[0]:
                continue
            if arc > self.PROX_SAMPLE_DIST[-1]:
                break
            c = self._costmap_cell(px, py)
            if c < 0:
                c = self.PROX_COST_LETHAL
            if c > max_cost:
                max_cost = c
        return max_cost

    def _max_cost_heading_arc(self, rx, ry, ryaw):
        cos_y, sin_y = math.cos(ryaw), math.sin(ryaw)
        cos_p, sin_p = math.cos(ryaw + math.pi / 2), math.sin(ryaw + math.pi / 2)
        max_cost = 0
        for df in self.PROX_SAMPLE_DIST:
            for dl in self.PROX_SAMPLE_LAT:
                x = rx + df * cos_y + dl * cos_p
                y = ry + df * sin_y + dl * sin_p
                c = self._costmap_cell(x, y)
                if c < 0:
                    c = self.PROX_COST_LETHAL
                if c > max_cost:
                    max_cost = c
        return max_cost

    def _check_progress(self, t_now):
        cutoff = t_now - self.PROGRESS_WINDOW_S
        self.pos_history = [(tt, xx, yy) for tt, xx, yy in self.pos_history if tt > cutoff]
        if len(self.pos_history) < 2:
            return float('inf')
        x0, y0 = self.pos_history[0][1], self.pos_history[0][2]
        xn, yn = self.pos_history[-1][1], self.pos_history[-1][2]
        return math.hypot(xn - x0, yn - y0)

    def tick(self):
        if self.path is None or self.path_done:
            self.cmd_pub.publish(Twist())
            return

        try:
            t = self.tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        ryaw = _yaw_from_quat(t.transform.rotation)

        t_now = time.time()
        self.pos_history.append((t_now, rx, ry))

        fin = self.path.poses[-1].pose.position
        d_fin = math.hypot(fin.x - rx, fin.y - ry)

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
        self.path_idx = max(self.path_idx, closest_idx)

        tgt = self.path.poses[lookahead_idx].pose.position
        angle_to_tgt = math.atan2(tgt.y - ry, tgt.x - rx)
        err = angle_to_tgt - ryaw
        err = math.atan2(math.sin(err), math.cos(err))

        cmd = Twist()
        cmd.linear.x = self.MAX_VEL * max(0.3, 1.0 - abs(err) / 1.57)
        cmd.angular.z = max(-self.MAX_ANG, min(self.MAX_ANG, self.GAIN_ANG * err))

        # v54 proximity speed limiter - plan-based (main) + emergency fallback.
        # Main: sample along the current plan (which already avoids obstacles).
        # Emergency: check the cell directly under the robot + 0.5 m ahead in
        # heading. If costmap says lethal there, we're physically inside an
        # inflation bubble - stale plan, obstacle just appeared, or drift
        # steered us off path. Slow hard regardless of plan-sample result.
        plan_cost = self._max_cost_ahead(rx, ry, ryaw)
        # emergency under-robot check: robot cell + one cell 0.5 m forward
        emerg_cost = max(self._costmap_cell(rx, ry),
                         self._costmap_cell(rx + 0.5 * math.cos(ryaw),
                                            ry + 0.5 * math.sin(ryaw)))
        if emerg_cost < 0:
            emerg_cost = self.PROX_COST_LETHAL
        prox_cost = max(plan_cost, emerg_cost)
        v_cap = self.MAX_VEL
        prox_label = ''
        if emerg_cost >= 80:
            v_cap = self.V_LETHAL
            prox_label = 'EMERG'
        elif prox_cost >= self.PROX_COST_LETHAL:
            v_cap = self.V_LETHAL
            prox_label = 'LETHAL'
        elif prox_cost >= self.PROX_COST_SLOW:
            v_cap = self.V_SLOW
            prox_label = 'SLOW'
        if v_cap < cmd.linear.x:
            self.prox_activations += 1
            cmd.linear.x = v_cap

        # v9 anti-spin
        is_spinning = (abs(cmd.angular.z) >= self.SPIN_W_THRESH
                       and abs(cmd.linear.x) <= self.SPIN_V_THRESH * 2)
        if is_spinning:
            self.spin_accum_t += 0.1
        else:
            self.spin_accum_t = max(0.0, self.spin_accum_t - 0.2)

        # v54 wedge-recovery - physically wedged? Reverse briefly.
        if t_now < self.wedge_backup_until:
            cmd.angular.z = 0.0
            cmd.linear.x = self.WEDGE_BACKUP_V
            prox_label = 'WEDGE'
        else:
            # Window check: in the last WEDGE_WINDOW_S, was cmd forward-like?
            # Use pos_history: if robot drift < WEDGE_MIN_DISP_M while MAX_VEL
            # was capped to something > 0, trigger backup.
            wedge_disp = self._check_progress(t_now)  # uses same window
            if (wedge_disp < self.WEDGE_MIN_DISP_M
                    and cmd.linear.x > 0.05
                    and len(self.pos_history) > 30):
                # confirm cmd has been forward over window
                self.wedge_backup_until = t_now + self.WEDGE_BACKUP_S
                self.wedge_activations += 1
                self.get_logger().warn(
                    f'[WEDGE #{self.wedge_activations}] disp={wedge_disp:.2f}m '
                    f'<{self.WEDGE_MIN_DISP_M}m over window -> reversing '
                    f'{self.WEDGE_BACKUP_V}m/s for {self.WEDGE_BACKUP_S}s')
                cmd.angular.z = 0.0
                cmd.linear.x = self.WEDGE_BACKUP_V
                prox_label = 'WEDGE'
                # clear pos history so we don't immediately retrigger
                self.pos_history = [(t_now, rx, ry)]

        if t_now < self.cooldown_until:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.15
        elif self.spin_accum_t >= self.SPIN_LIMIT_S:
            progress = self._check_progress(t_now)
            if progress < self.MIN_PROGRESS_M:
                self.cooldown_until = t_now + self.SPIN_COOLDOWN_S
                self.spin_accum_t = 0.0
                self.anti_spin_activations += 1
                self.get_logger().warn(
                    f'[ANTI-SPIN #{self.anti_spin_activations}] spin>'
                    f'{self.SPIN_LIMIT_S}s, progress={progress:.2f}m < '
                    f'{self.MIN_PROGRESS_M}m. Cooldown {self.SPIN_COOLDOWN_S}s.')
                cmd.angular.z = 0.0
                cmd.linear.x = 0.15

        self.cmd_pub.publish(cmd)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f'pos=({rx:.1f},{ry:.1f}) tgt=({tgt.x:.1f},{tgt.y:.1f}) '
                f'err={math.degrees(err):.0f}° d_fin={d_fin:.1f}m '
                f'v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} '
                f'spin_t={self.spin_accum_t:.1f}s prox=[{prox_label}:{prox_cost}] '
                f'prox_hits={self.prox_activations}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lookahead', type=float, default=2.0)
    ap.add_argument('--max-vel', type=float, default=1.0)
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
