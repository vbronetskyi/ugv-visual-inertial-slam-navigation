#!/usr/bin/env python3
"""
Small ego-centric grid (20m x 20m, 0.2m resolution = 100x100 cells)
that accumulates depth obstacle detections over time.

As robot approaches obstacles, hits accumulate from multiple frames
and viewpoints. This gives:
1. Earlier detection (5-6m vs 2-3m from single frame)
2. Cone groups merge into continuous obstacles (not 3 separate objects)
3. Stable obstacle representation (not flickering per frame)

Grid is ego-centric - shifts with robot movement each frame.
Old observations decay so grid doesn't fill up.
"""
import math
import numpy as np


class LocalObstacleGrid:
    def __init__(self, size=20.0, resolution=0.2):
        self.size = size
        self.resolution = resolution
        self.grid_size = int(size / resolution)  # 100
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.float32)

        self.center = self.grid_size // 2  # robot at center

        # Tuning
        self.DECAY_RATE = 0.997       # per-frame decay: half-life ~4s at 60fps
        self.HIT_VALUE = 1.5          # stronger hits per observation
        self.OBSTACLE_HITS = 8.0      # higher threshold with slower decay
        self.MAX_DEPTH_FOR_GRID = 5.0 # 5m detection (was 4m)
        self.MIN_DEPTH = 0.3

        # Camera
        self.CAMERA_FOV = 1.2  # rad (~69 degrees, D435)
        self.DEPTH_WIDTH = 640
        self.DEPTH_HEIGHT = 480

        self.prev_x = None
        self.prev_y = None

    def update(self, robot_x, robot_y, robot_yaw, depth_image):
        """
        Each frame:
        1. Shift grid to keep robot at center
        2. Decay old observations
        3. Project current depth into grid
        """
        # 1. Shift grid based on robot movement
        if self.prev_x is not None:
            dx_world = robot_x - self.prev_x
            dy_world = robot_y - self.prev_y

            shift_x = int(dx_world / self.resolution)
            shift_y = int(dy_world / self.resolution)

            if abs(shift_x) > 0 or abs(shift_y) > 0:
                if shift_x != 0:
                    self.grid = np.roll(self.grid, -shift_x, axis=1)
                    if shift_x > 0:
                        self.grid[:, -shift_x:] = 0
                    else:
                        self.grid[:, :(-shift_x)] = 0

                if shift_y != 0:
                    self.grid = np.roll(self.grid, -shift_y, axis=0)
                    if shift_y > 0:
                        self.grid[-shift_y:, :] = 0
                    else:
                        self.grid[:(-shift_y), :] = 0

        self.prev_x = robot_x
        self.prev_y = robot_y

        # 2. Decay
        self.grid *= self.DECAY_RATE

        # 3. Project depth
        if depth_image is not None:
            self._project_depth(robot_yaw, depth_image)

    def _project_depth(self, robot_yaw, depth_image):
        """Project blocked depth pixels into grid cells."""
        h, w = depth_image.shape[:2]

        y_start = h // 3
        y_end = h * 2 // 3
        strip = depth_image[y_start:y_end, :]

        col_angles = np.linspace(
            -self.CAMERA_FOV / 2, self.CAMERA_FOV / 2, w
        )

        # Process every 4th column for speed
        for col in range(0, w, 4):
            column = strip[:, col]
            valid = column[
                (column > self.MIN_DEPTH)
                & (column < self.MAX_DEPTH_FOR_GRID)
                & np.isfinite(column)
            ]

            if len(valid) < 3:
                continue

            depth = float(np.percentile(valid, 10))

            angle = robot_yaw + col_angles[col]
            dx = depth * math.cos(angle)
            dy = depth * math.sin(angle)

            gx = self.center + int(dx / self.resolution)
            gy = self.center + int(dy / self.resolution)

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.grid[gy, gx] += self.HIT_VALUE

                # Mark neighboring cells (obstacle has physical extent)
                for ngx, ngy in [(gx+1, gy), (gx-1, gy),
                                 (gx, gy+1), (gx, gy-1)]:
                    if 0 <= ngx < self.grid_size and 0 <= ngy < self.grid_size:
                        self.grid[ngy, ngx] += self.HIT_VALUE * 0.3

    def get_obstacle_profile(self, robot_yaw, num_sectors=30, max_range=8.0):
        """
        Ray-march from robot center outward in each sector direction.
        Return distance to first confirmed obstacle per sector.

        This replaces the single-frame depth profile in gap_navigator.
        """
        sector_angles = np.linspace(
            robot_yaw - self.CAMERA_FOV / 2,
            robot_yaw + self.CAMERA_FOV / 2,
            num_sectors,
        )

        profile = np.full(num_sectors, max_range)

        for i, angle in enumerate(sector_angles):
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            dist = self.resolution * 2
            while dist < max_range:
                gx = self.center + int(dist * cos_a / self.resolution)
                gy = self.center + int(dist * sin_a / self.resolution)

                if not (0 <= gx < self.grid_size and 0 <= gy < self.grid_size):
                    break

                if self.grid[gy, gx] >= self.OBSTACLE_HITS:
                    profile[i] = dist
                    break

                dist += self.resolution

        return profile

    def get_grid_image(self):
        """For debug visualization."""
        normalized = np.clip(self.grid / self.OBSTACLE_HITS, 0, 1)
        return (normalized * 255).astype(np.uint8)

    def get_stats(self):
        """Debug stats."""
        total_cells = self.grid_size * self.grid_size
        obstacle_cells = int(np.sum(self.grid >= self.OBSTACLE_HITS))
        max_hits = float(np.max(self.grid))
        return {
            "obstacle_cells": obstacle_cells,
            "obstacle_pct": round(obstacle_cells / total_cells * 100, 1),
            "max_hits": round(max_hits, 1),
        }
