#!/usr/bin/env python3
"""
Simple A* local planner on the ego-centric obstacle grid.
Plans a short path (5-10m) from robot to lookahead anchor.

Grid from LocalObstacleGrid: 100x100, 0.2m resolution, robot at center.
Obstacles are accumulated depth hits (higher = more confirmed).

Output: next waypoint direction (angle + distance) for the controller.
"""
import math
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt


class GridPlanner:
    def __init__(self, grid_size=100, resolution=0.2):
        self.grid_size = grid_size
        self.resolution = resolution
        self.center = grid_size // 2

        # Cost parameters
        self.OBSTACLE_COST_SCALE = 50.0
        self.OBSTACLE_HITS_THRESHOLD = 2.0
        self.INFLATION_RADIUS_CELLS = 4  # 0.8m (robot half-width + margin)
        self.ROBOT_RADIUS_CELLS = 2      # 0.4m ~ half robot width

        # 8-connected neighbors: (dx, dy, move_cost)
        self.NEIGHBORS = [
            (-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414),
            (0, -1, 1.0),                   (0, 1, 1.0),
            (1, -1, 1.414),  (1, 0, 1.0),  (1, 1, 1.414),
        ]

        # Cache
        self._last_costmap = None

    def build_costmap(self, obstacle_grid):
        """Convert raw obstacle grid (hit counts) into navigation costmap."""
        costmap = np.ones((self.grid_size, self.grid_size), dtype=np.float32)

        obstacle_mask = obstacle_grid >= self.OBSTACLE_HITS_THRESHOLD
        costmap[obstacle_mask] = 999.0

        dist_to_obs = distance_transform_edt(~obstacle_mask)

        inflation_cost = np.where(
            dist_to_obs < self.INFLATION_RADIUS_CELLS,
            self.OBSTACLE_COST_SCALE * (1.0 - dist_to_obs / self.INFLATION_RADIUS_CELLS),
            0.0,
        )
        costmap += inflation_cost

        costmap[dist_to_obs < self.ROBOT_RADIUS_CELLS] = 999.0

        self._last_costmap = costmap
        return costmap

    def world_to_grid(self, dx, dy):
        """World-relative offset (from robot) to grid coords."""
        gx = self.center + int(dx / self.resolution)
        gy = self.center + int(dy / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Grid coords to world-relative offset."""
        dx = (gx - self.center) * self.resolution
        dy = (gy - self.center) * self.resolution
        return dx, dy

    def plan(self, obstacle_grid, target_dx, target_dy):
        """
        Plan path from robot (grid center) to target point.

        Args:
            obstacle_grid: 2D array from LocalObstacleGrid.grid
            target_dx, target_dy: target relative to robot (meters, world frame)

        Returns:
            next_dx, next_dy: direction to move (waypoint on path, meters)
            path_found: bool
            path_length: float (meters)
        """
        costmap = self.build_costmap(obstacle_grid)

        start = (self.center, self.center)

        goal_gx, goal_gy = self.world_to_grid(target_dx, target_dy)
        goal_gx = max(1, min(self.grid_size - 2, goal_gx))
        goal_gy = max(1, min(self.grid_size - 2, goal_gy))
        goal = (goal_gx, goal_gy)

        if costmap[goal[1], goal[0]] >= 999.0:
            goal = self._find_nearest_free(costmap, goal)
            if goal is None:
                return 0.0, 0.0, False, 0.0

        path = self._astar(costmap, start, goal)

        if path is None or len(path) < 2:
            return 0.0, 0.0, False, 0.0

        # Lookahead on path: 15 cells = 3m
        lookahead_idx = min(15, len(path) - 1)
        wp = path[lookahead_idx]

        next_dx, next_dy = self.grid_to_world(wp[0], wp[1])
        path_length = len(path) * self.resolution

        return next_dx, next_dy, True, path_length

    def _astar(self, costmap, start, goal):
        """Standard A* on grid with cost-weighted movement."""
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        max_iterations = 8000
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            cx, cy = current

            for dx, dy, move_cost in self.NEIGHBORS:
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue

                cell_cost = costmap[ny, nx]
                if cell_cost >= 999.0:
                    continue

                tentative_g = g_score[current] + move_cost * cell_cost
                neighbor = (nx, ny)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))
                    came_from[neighbor] = current

        return None

    def _heuristic(self, a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _find_nearest_free(self, costmap, goal):
        gx, gy = goal
        for r in range(1, 20):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if (0 <= nx < self.grid_size and 0 <= ny < self.grid_size
                            and costmap[ny, nx] < 999.0):
                        return (nx, ny)
        return None

    def get_debug_info(self):
        """Return costmap stats for logging."""
        if self._last_costmap is None:
            return {"impassable": 0, "inflated": 0}
        cm = self._last_costmap
        return {
            "impassable": int(np.sum(cm >= 999.0)),
            "inflated": int(np.sum((cm > 1.5) & (cm < 999.0))),
        }
