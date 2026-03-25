#!/usr/bin/env python3
"""
Depth-primary navigation with route guidance.
Like VFH (Vector Field Histogram) with route preference.

1. Depth image -> free/blocked sectors
2. Route anchor -> desired heading
3. Pick best free sector closest to desired heading
4. Generate cmd_vel
"""
import math
import numpy as np


class DepthNavigator:
    def __init__(self):
        self.NUM_SECTORS = 15
        self.CAMERA_FOV = 1.2  # ~69° D435
        self.MIN_DIST = 0.3
        self.MAX_RANGE = 5.0
        self.OBSTACLE_THRESHOLD = 2.0
        self.CRITICAL_THRESHOLD = 0.8

        self.MAX_LINEAR = 0.8
        self.MIN_LINEAR = 0.15
        self.MAX_ANGULAR = 0.6
        self.ROUTE_WEIGHT = 0.7
        self.FREE_SPACE_WEIGHT = 0.3

        self.sector_angles = np.linspace(
            -self.CAMERA_FOV / 2, self.CAMERA_FOV / 2, self.NUM_SECTORS
        )
        sw = 640 // self.NUM_SECTORS
        self.sector_ranges = [(i * sw, (i + 1) * sw) for i in range(self.NUM_SECTORS)]

    def analyze_depth(self, depth_image):
        if depth_image is None:
            return np.full(self.NUM_SECTORS, self.MAX_RANGE)

        y1, y2 = 480 // 3, 480 * 2 // 3
        strip = depth_image[y1:y2, :]
        dists = np.full(self.NUM_SECTORS, self.MAX_RANGE)

        for i, (x1, x2) in enumerate(self.sector_ranges):
            s = strip[:, x1:x2]
            valid = s[(s > self.MIN_DIST) & (s < self.MAX_RANGE) & np.isfinite(s)]
            if len(valid) > 5:
                dists[i] = float(np.percentile(valid, 10))
        return dists

    def compute_cmd_vel(self, depth_image, desired_heading_error):
        dists = self.analyze_depth(depth_image)

        # Critical zone check
        c = self.NUM_SECTORS // 2
        center_min = min(dists[c - 1], dists[c], dists[c + 1])
        if center_min < self.CRITICAL_THRESHOLD:
            best = int(np.argmax(dists))
            turn = 1.0 if best < c else -1.0
            return 0.0, turn * self.MAX_ANGULAR

        # Desirability per sector
        desirability = np.zeros(self.NUM_SECTORS)
        for i in range(self.NUM_SECTORS):
            free = min(dists[i] / self.MAX_RANGE, 1.0)
            if dists[i] < self.OBSTACLE_THRESHOLD:
                free = 0.0

            angle_diff = abs(self.sector_angles[i] - desired_heading_error)
            route = max(0.0, 1.0 - angle_diff / self.CAMERA_FOV)

            desirability[i] = self.FREE_SPACE_WEIGHT * free + self.ROUTE_WEIGHT * route
            if free > 0.5 and route > 0.5:
                desirability[i] *= 1.3

        best = int(np.argmax(desirability))
        best_angle = self.sector_angles[best]

        ang = np.clip(best_angle * 2.0, -self.MAX_ANGULAR, self.MAX_ANGULAR)

        fwd = dists[c]
        if fwd > self.OBSTACLE_THRESHOLD * 1.5:
            lin = self.MAX_LINEAR
        else:
            lin = max(self.MAX_LINEAR * fwd / (self.OBSTACLE_THRESHOLD * 1.5),
                      self.MIN_LINEAR)

        if abs(ang) > self.MAX_ANGULAR * 0.5:
            lin *= 0.5

        return float(lin), float(ang)

    def debug_info(self, dists, desired):
        blocked = int(np.sum(dists < self.OBSTACLE_THRESHOLD))
        best = int(np.argmax(dists))
        return (f"blk={blocked}/{self.NUM_SECTORS} "
                f"min={np.min(dists):.1f}m "
                f"best={math.degrees(self.sector_angles[best]):+.0f}deg "
                f"want={math.degrees(desired):+.0f}deg")
