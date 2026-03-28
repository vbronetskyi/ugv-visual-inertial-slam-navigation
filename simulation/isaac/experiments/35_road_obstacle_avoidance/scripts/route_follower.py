#!/usr/bin/env python3
"""
Route-following controller for teach-and-repeat navigation.
Uses anchor localizer for position + dead-reckoning between matches.
"""
import numpy as np


class RouteFollower:
    def __init__(self, anchors, lookahead_distance=8.0):
        self.anchors = anchors
        self.lookahead_dist = lookahead_distance

        self.MAX_LINEAR_VEL = 0.8
        self.MIN_LINEAR_VEL = 0.2
        self.MAX_ANGULAR_VEL = 0.5
        self.GOAL_TOLERANCE = 2.0

        self.KP_ANGULAR = 1.5
        self.SLOWDOWN_ANGLE = 0.5

        self.current_anchor_idx = 0
        self.finished = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_initialized = False

    def initialize_from_anchor(self, anchor_idx):
        anchor = self.anchors[anchor_idx]
        self.odom_x = anchor["x"]
        self.odom_y = anchor["y"]
        self.odom_yaw = anchor["yaw"]
        self.current_anchor_idx = anchor_idx
        self.odom_initialized = True

    def update_odometry(self, linear_vel, angular_vel, dt):
        if not self.odom_initialized:
            return
        self.odom_x += linear_vel * dt * np.cos(self.odom_yaw)
        self.odom_y += linear_vel * dt * np.sin(self.odom_yaw)
        self.odom_yaw += angular_vel * dt
        self.odom_yaw = np.arctan2(np.sin(self.odom_yaw), np.cos(self.odom_yaw))

    def correct_from_anchor(self, anchor_idx, confidence):
        if confidence < 0.2:
            return

        anchor = self.anchors[anchor_idx]
        alpha = min(confidence, 0.8)

        self.odom_x = (1 - alpha) * self.odom_x + alpha * anchor["x"]
        self.odom_y = (1 - alpha) * self.odom_y + alpha * anchor["y"]

        yaw_diff = anchor["yaw"] - self.odom_yaw
        yaw_diff = np.arctan2(np.sin(yaw_diff), np.cos(yaw_diff))
        self.odom_yaw += alpha * 0.5 * yaw_diff

        self.current_anchor_idx = anchor_idx

    def get_adaptive_lookahead(self):
        cur = self.anchors[self.current_anchor_idx]
        max_yaw_change = 0.0
        for i in range(self.current_anchor_idx + 1,
                       min(self.current_anchor_idx + 5, len(self.anchors))):
            yd = abs(self.anchors[i]["yaw"] - cur["yaw"])
            yd = min(yd, 2 * np.pi - yd)
            max_yaw_change = max(max_yaw_change, yd)
        if max_yaw_change > 1.0:
            return 3.0
        elif max_yaw_change > 0.5:
            return 5.0
        return self.lookahead_dist

    def get_lookahead_point(self, lookahead=None):
        if lookahead is None:
            lookahead = self.lookahead_dist
        current_s = self.anchors[self.current_anchor_idx]["s"]
        target_s = current_s + lookahead
        for anchor in self.anchors[self.current_anchor_idx :]:
            if anchor["s"] >= target_s:
                return anchor["x"], anchor["y"], anchor["id"]
        last = self.anchors[-1]
        return last["x"], last["y"], last["id"]

    def compute_cmd_vel(self):
        if self.finished or not self.odom_initialized:
            return 0.0, 0.0

        last = self.anchors[-1]
        dist_to_end = np.hypot(self.odom_x - last["x"], self.odom_y - last["y"])
        if dist_to_end < self.GOAL_TOLERANCE:
            self.finished = True
            return 0.0, 0.0

        lookahead = self.get_adaptive_lookahead()
        tx, ty, _ = self.get_lookahead_point(lookahead)
        dx = tx - self.odom_x
        dy = ty - self.odom_y
        dist = np.hypot(dx, dy)
        if dist < 0.1:
            return 0.0, 0.0

        target_angle = np.arctan2(dy, dx)
        heading_error = target_angle - self.odom_yaw
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        angular_vel = np.clip(
            self.KP_ANGULAR * heading_error,
            -self.MAX_ANGULAR_VEL,
            self.MAX_ANGULAR_VEL,
        )

        if abs(heading_error) > self.SLOWDOWN_ANGLE:
            linear_vel = self.MIN_LINEAR_VEL
        else:
            factor = 1.0 - (abs(heading_error) / self.SLOWDOWN_ANGLE) * 0.7
            linear_vel = max(self.MAX_LINEAR_VEL * factor, self.MIN_LINEAR_VEL)

        return linear_vel, angular_vel

    def get_stats(self):
        la = self.get_adaptive_lookahead()
        tx, ty, tid = self.get_lookahead_point(la)
        return {
            "anchor_idx": self.current_anchor_idx,
            "odom_x": self.odom_x,
            "odom_y": self.odom_y,
            "odom_yaw": self.odom_yaw,
            "target_anchor": tid,
            "progress_m": self.anchors[self.current_anchor_idx]["s"],
            "total_m": self.anchors[-1]["s"],
            "finished": self.finished,
        }
