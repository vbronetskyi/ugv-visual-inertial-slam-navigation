#!/usr/bin/env python3
"""
Feasible-gap navigation constrained by route heading.

Finds real passable gaps between obstacles considering Husky width,
then picks the best gap aligned with the route direction.
"""
import math
import numpy as np
import cv2


class GapNavigator:
    def __init__(self, robot_width=0.67, safety_margin=0.3):
        self.ROBOT_WIDTH = robot_width
        self.SAFETY_MARGIN = safety_margin
        self.REQUIRED_WIDTH = robot_width + 2 * safety_margin

        self.DEPTH_WIDTH = 640
        self.DEPTH_HEIGHT = 480
        self.CAMERA_FOV = 1.2

        self.MIN_VALID_DEPTH = 0.8  # ignore branches/leaves closer than bumper
        self.MAX_RANGE = 5.0
        self.OBSTACLE_THRESHOLD = 1.5  # closer threshold - forest has 2-3m gaps
        self.CRITICAL_DIST = 0.8

        self.ROUTE_CONE_HALF = 0.6   # ±34° normal
        self.ROUTE_CONE_RECOVERY = 1.2  # ±69° recovery - full FOV

        self.W_ROUTE = 0.4
        self.W_CLEARANCE = 0.25
        self.W_WIDTH = 0.2
        self.W_STABILITY = 0.15

        self.MAX_LINEAR = 0.7
        self.MIN_LINEAR = 0.1
        self.MAX_ANGULAR = 0.5
        self.SLOW_SAFE_LINEAR = 0.25
        self.SLOW_SAFE_ANGULAR = 0.3

        self.current_gap = None
        self.gap_hold_frames = 0
        self.MIN_HOLD_FRAMES = 3
        self.SWITCH_THRESHOLD = 0.3

        self.mode = "ROUTE_TRACKING"
        self.prev_gaps = []
        self.HISTORY_SIZE = 5

        self.col_to_angle = np.linspace(
            -self.CAMERA_FOV / 2, self.CAMERA_FOV / 2, self.DEPTH_WIDTH
        )

    def depth_to_obstacle_profile(self, depth_image):
        if depth_image is None:
            return np.ones(self.DEPTH_WIDTH, dtype=bool), \
                   np.full(self.DEPTH_WIDTH, self.MAX_RANGE)

        y1, y2 = self.DEPTH_HEIGHT // 3, self.DEPTH_HEIGHT * 2 // 3
        strip = depth_image[y1:y2, :]

        profile = np.full(self.DEPTH_WIDTH, self.MAX_RANGE)
        for col in range(self.DEPTH_WIDTH):
            column = strip[:, col]
            valid = column[
                (column > self.MIN_VALID_DEPTH)
                & (column < self.MAX_RANGE)
                & np.isfinite(column)
            ]
            if len(valid) > 3:
                profile[col] = float(np.percentile(valid, 10))

        free_mask = profile > self.OBSTACLE_THRESHOLD
        return free_mask, profile

    def inflate_obstacles(self, free_mask):
        rad_per_px = self.CAMERA_FOV / self.DEPTH_WIDTH
        m_per_px = self.OBSTACLE_THRESHOLD * math.tan(rad_per_px)
        half_px = max(int(self.REQUIRED_WIDTH / 2 / max(m_per_px, 0.001)), 5)
        kernel = np.ones(2 * half_px + 1, dtype=np.uint8).reshape(1, -1)
        inflated = cv2.erode(
            free_mask.astype(np.uint8), kernel, iterations=1
        ).astype(bool).flatten()
        return inflated

    def find_gaps(self, inflated, profile):
        gaps = []
        in_gap = False
        start = 0
        for col in range(len(inflated)):
            if inflated[col] and not in_gap:
                start = col
                in_gap = True
            elif not inflated[col] and in_gap:
                g = self._make_gap(start, col - 1, profile)
                if g:
                    gaps.append(g)
                in_gap = False
        if in_gap:
            g = self._make_gap(start, len(inflated) - 1, profile)
            if g:
                gaps.append(g)
        return gaps

    def _make_gap(self, left, right, profile):
        center = (left + right) // 2
        ca = float(self.col_to_angle[center])
        aw = float(self.col_to_angle[right] - self.col_to_angle[left])
        if aw < 0.1:
            return None
        seg = profile[left : right + 1]
        valid = seg[(seg > self.MIN_VALID_DEPTH) & np.isfinite(seg)]
        cl = float(np.min(valid)) if len(valid) > 3 else self.MAX_RANGE
        md = float(np.median(valid)) if len(valid) > 3 else self.MAX_RANGE
        return {
            "left_col": left, "right_col": right, "center_col": center,
            "center_angle": ca, "angular_width": aw,
            "clearance": cl, "median_depth": md,
        }

    def score_gaps(self, gaps, desired):
        cone = self.ROUTE_CONE_RECOVERY if self.mode == "STUCK" else self.ROUTE_CONE_HALF
        scored = []
        for g in gaps:
            ad = abs(g["center_angle"] - desired)
            if ad > cone:
                continue
            rs = max(0.0, 1.0 - ad / cone)
            cs = min(g["clearance"] / self.MAX_RANGE, 1.0)
            ws = min(g["angular_width"] / 0.5, 1.0)
            ss = self._stability(g)
            total = self.W_ROUTE * rs + self.W_CLEARANCE * cs + \
                    self.W_WIDTH * ws + self.W_STABILITY * ss
            scored.append({**g, "route_score": rs, "clearance_score": cs,
                           "width_score": ws, "stability_score": ss,
                           "total_score": total})
        scored.sort(key=lambda g: g["total_score"], reverse=True)
        return scored

    def _stability(self, gap):
        if not self.prev_gaps:
            return 0.5
        matches = sum(
            1 for pg_list in self.prev_gaps
            for pg in pg_list
            if abs(pg["center_angle"] - gap["center_angle"]) < 0.15
        )
        return min(matches / max(len(self.prev_gaps), 1), 1.0)

    def select_gap(self, scored):
        if not scored:
            return None
        best = scored[0]
        if self.current_gap is not None and self.gap_hold_frames < self.MIN_HOLD_FRAMES:
            for sg in scored:
                if abs(sg["center_angle"] - self.current_gap["center_angle"]) < 0.15:
                    self.gap_hold_frames += 1
                    return sg
        if self.current_gap is not None:
            cur = None
            for sg in scored:
                if abs(sg["center_angle"] - self.current_gap["center_angle"]) < 0.15:
                    cur = sg
                    break
            if cur and best["total_score"] - cur["total_score"] < self.SWITCH_THRESHOLD:
                self.gap_hold_frames += 1
                return cur
        self.current_gap = best
        self.gap_hold_frames = 0
        return best

    def _determine_mode(self, gaps, scored, profile):
        blocked = sum(1 for d in profile if d < self.OBSTACLE_THRESHOLD)
        ratio = blocked / len(profile)
        c = self.DEPTH_WIDTH // 4
        center_min = float(np.min(profile[self.DEPTH_WIDTH // 2 - c:
                                          self.DEPTH_WIDTH // 2 + c]))
        if not scored and not gaps:
            self.mode = "STUCK"
        elif not scored and gaps:
            # Gaps exist but outside route cone - widen cone
            self.mode = "STUCK"  # will use RECOVERY cone on next call
        elif ratio > 0.6 or center_min < self.OBSTACLE_THRESHOLD:
            self.mode = "GAP_MODE"
        else:
            self.mode = "ROUTE_TRACKING"

    def compute_cmd_vel(self, depth_image, desired_heading_error):
        free_mask, profile = self.depth_to_obstacle_profile(depth_image)
        inflated = self.inflate_obstacles(free_mask)
        gaps = self.find_gaps(inflated, profile)
        scored = self.score_gaps(gaps, desired_heading_error)
        self._determine_mode(gaps, scored, profile)

        self.prev_gaps.append(gaps)
        if len(self.prev_gaps) > self.HISTORY_SIZE:
            self.prev_gaps.pop(0)

        blocked_ratio = sum(
            1 for d in profile if d < self.OBSTACLE_THRESHOLD
        ) / len(profile)
        debug = {
            "mode": self.mode, "total_gaps": len(gaps),
            "valid_gaps": len(scored), "blocked_ratio": blocked_ratio,
        }

        if self.mode == "STUCK":
            debug["action"] = "no_feasible_gap"
            return 0.0, self.SLOW_SAFE_ANGULAR * 0.5, self.mode, debug

        selected = self.select_gap(scored)
        if selected is None:
            return 0.0, 0.0, self.mode, debug

        debug["sel_ang"] = selected["center_angle"]
        debug["sel_score"] = selected["total_score"]
        debug["sel_width"] = selected["angular_width"]

        if self.mode == "ROUTE_TRACKING":
            ang = np.clip(desired_heading_error * 1.5,
                          -self.MAX_ANGULAR, self.MAX_ANGULAR)
            cd = float(np.median(profile[
                self.DEPTH_WIDTH // 3: self.DEPTH_WIDTH * 2 // 3]))
            lin = self.MAX_LINEAR * max(min(cd / self.MAX_RANGE, 1.0), 0.4)
        else:  # GAP_MODE
            ang = np.clip(selected["center_angle"] * 2.0,
                          -self.SLOW_SAFE_ANGULAR, self.SLOW_SAFE_ANGULAR)
            cf = min(selected["clearance"] / self.OBSTACLE_THRESHOLD, 1.0)
            lin = self.SLOW_SAFE_LINEAR * max(cf, 0.3)
            if abs(ang) > self.SLOW_SAFE_ANGULAR * 0.6:
                lin *= 0.5

        return float(lin), float(ang), self.mode, debug
