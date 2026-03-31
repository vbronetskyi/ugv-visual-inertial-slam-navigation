#!/usr/bin/env python3
"""Pure pursuit follower with spin-in-place cascade protection (v9).

Subscribes to Nav2 /plan, publishes /cmd_vel. Pure pursuit control law
with a new safety layer:

- Monitors "spinning" state: high |w|, low |v|, low progress over time.
- If spinning for > SPIN_LIMIT_S without translating meaningfully,
  stop emitting rotate commands for a cooldown period. Without the
  feedback from the rotate, localization gets a chance to stabilise.

Rationale: in exp 51 v8, VIO err past 150 m grew > Nav2 goal-tolerance;
Nav2 kept replanning; new path's lookahead point ended up "behind" robot;
PP read that as 180° heading err -> saturated w=0.8 rad/s. Robot spun in
place, adding VIO uncertainty, cascading to failure.
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
        self.MAX_VEL = max_vel
        self.GAIN_ANG = 1.2
        self.MAX_ANG = 0.8
        self.GOAL_TOL = goal_tol

        # v9 anti-spin parameters
        self.SPIN_W_THRESH = 0.5        # |w| above this counts as "spinning"
        self.SPIN_V_THRESH = 0.05       # |v| below this counts as "in place"
        self.SPIN_LIMIT_S = 5.0         # after this long -> declare spin-in-place
        self.SPIN_COOLDOWN_S = 3.0      # freeze rotation for this long
        self.PROGRESS_WINDOW_S = 5.0    # check position progress over this window
        self.MIN_PROGRESS_M = 0.5       # need at least this m displacement in window

        self.path = None
        self.path_idx = 0
        self.path_done = False

        # Anti-spin state
        self.spin_accum_t = 0.0
        self.cooldown_until = 0.0
        self.pos_history = []  # list of (t, x, y)

        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.timer = self.create_timer(0.1, self.tick)
        self.log_counter = 0
        self.anti_spin_activations = 0

    def path_cb(self, msg):
        if len(msg.poses) < 2:
            return
        self.path = msg
        self.path_idx = 0
        self.path_done = False
        self.get_logger().info(f'New path: {len(msg.poses)} poses, '
            f'end=({msg.poses[-1].pose.position.x:.1f},'
            f'{msg.poses[-1].pose.position.y:.1f})')

    def _check_progress(self, t_now):
        """Return displacement (m) over recent PROGRESS_WINDOW_S."""
        # Trim history
        cutoff = t_now - self.PROGRESS_WINDOW_S
        self.pos_history = [(tt, xx, yy) for tt, xx, yy in self.pos_history if tt > cutoff]
        if len(self.pos_history) < 2:
            return float('inf')  # not enough data -> assume progress OK
        x0, y0 = self.pos_history[0][1], self.pos_history[0][2]
        xn, yn = self.pos_history[-1][1], self.pos_history[-1][2]
        return math.hypot(xn - x0, yn - y0)

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

        import time
        t_now = time.time()
        self.pos_history.append((t_now, rx, ry))

        fin = self.path.poses[-1].pose.position
        d_fin = math.hypot(fin.x - rx, fin.y - ry)

        # Find lookahead point
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

        # v9 anti-spin layer
        # Detect "actively spinning" state: high w, low v from our own cmd
        is_spinning = (abs(cmd.angular.z) >= self.SPIN_W_THRESH
                       and abs(cmd.linear.x) <= self.SPIN_V_THRESH * 2)
        if is_spinning:
            self.spin_accum_t += 0.1  # 10 Hz tick
        else:
            self.spin_accum_t = max(0.0, self.spin_accum_t - 0.2)

        if t_now < self.cooldown_until:
            # Inside cooldown - suppress rotation, just creep forward slowly.
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
                f'spin_t={self.spin_accum_t:.1f}s')


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
