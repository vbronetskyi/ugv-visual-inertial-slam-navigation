#!/usr/bin/env python3
"""
Optimized A* local planner - wraps GridPlanner with caching.

Key optimizations vs running GridPlanner every frame:
1. Replan every N frames (10), use cached path between replans
2. Reduced A* iterations (3000)
3. Early goal proximity termination
4. Timing instrumentation

scipy distance_transform is already fast (~1ms). The real overhead was
replanning every 3 frames. With caching, we plan 6× per second instead of 20×.
"""
import math
import time as _time
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt


class FastGridPlanner:
    def __init__(self, grid_size=100, resolution=0.2):
        self.grid_size = grid_size
        self.resolution = resolution
        self.center = grid_size // 2

        # Costmap params
        self.OBSTACLE_COST = 999.0
        self.INFLATION_COST_SCALE = 50.0
        self.INFLATION_RADIUS = 4   # cells
        self.ROBOT_RADIUS = 2       # cells
        self.HITS_THRESHOLD = 8.0

        # Cache
        self.cached_path = None
        self.cached_goal = None
        self.replan_counter = 0
        self.REPLAN_INTERVAL = 10

        # A*
        self.MAX_ITERATIONS = 3000
        self.NEIGHBORS = [
            (-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414),
            (0, -1, 1.0),                   (0, 1, 1.0),
            (1, -1, 1.414),  (1, 0, 1.0),  (1, 1, 1.414),
        ]

        # Debug
        self._last_impassable = 0
        self._last_plan_ms = 0.0
        self._replanned = False

    def _build_costmap(self, obstacle_grid):
        costmap = np.ones((self.grid_size, self.grid_size), dtype=np.float32)
        obs_mask = obstacle_grid >= self.HITS_THRESHOLD
        costmap[obs_mask] = self.OBSTACLE_COST

        dist = distance_transform_edt(~obs_mask)
        inflation = np.where(
            dist < self.INFLATION_RADIUS,
            self.INFLATION_COST_SCALE * (1.0 - dist / self.INFLATION_RADIUS),
            0.0,
        )
        costmap += inflation
        costmap[dist < self.ROBOT_RADIUS] = self.OBSTACLE_COST

        self._last_impassable = int(np.sum(costmap >= self.OBSTACLE_COST))
        return costmap

    def plan(self, obstacle_grid, target_dx, target_dy, force_replan=False):
        self.replan_counter += 1
        self._replanned = False

        goal_gx = self.center + int(target_dx / self.resolution)
        goal_gy = self.center + int(target_dy / self.resolution)
        goal_gx = max(1, min(self.grid_size - 2, goal_gx))
        goal_gy = max(1, min(self.grid_size - 2, goal_gy))
        goal = (goal_gx, goal_gy)

        need = (
            force_replan
            or self.cached_path is None
            or self.replan_counter >= self.REPLAN_INTERVAL
            or self.cached_goal is None
            or abs(goal[0] - self.cached_goal[0]) > 5
            or abs(goal[1] - self.cached_goal[1]) > 5
        )

        if need:
            t0 = _time.monotonic()
            costmap = self._build_costmap(obstacle_grid)
            start = (self.center, self.center)

            if costmap[goal[1], goal[0]] >= self.OBSTACLE_COST:
                goal = self._nearest_free(costmap, goal)
                if goal is None:
                    self.cached_path = None
                    self._last_plan_ms = (_time.monotonic() - t0) * 1000
                    return 0.0, 0.0, False, 0.0

            path = self._astar(costmap, start, goal)
            self.cached_path = path
            self.cached_goal = goal
            self.replan_counter = 0
            self._replanned = True
            self._last_plan_ms = (_time.monotonic() - t0) * 1000

        if self.cached_path is None or len(self.cached_path) < 2:
            return 0.0, 0.0, False, 0.0

        li = min(15, len(self.cached_path) - 1)
        wp = self.cached_path[li]
        wp_dx = (wp[0] - self.center) * self.resolution
        wp_dy = (wp[1] - self.center) * self.resolution
        plen = len(self.cached_path) * self.resolution
        return wp_dx, wp_dy, True, plen

    def _astar(self, costmap, start, goal):
        heap = [(0.0, start)]
        came = {}
        gs = {start: 0.0}
        it = 0
        while heap and it < self.MAX_ITERATIONS:
            it += 1
            _, cur = heapq.heappop(heap)
            if cur == goal or (abs(cur[0]-goal[0]) <= 1 and abs(cur[1]-goal[1]) <= 1):
                return self._recon(came, cur)
            cx, cy = cur
            for dy, dx, mc in self.NEIGHBORS:
                ny, nx = cy + dy, cx + dx
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue
                cc = costmap[ny, nx]
                if cc >= self.OBSTACLE_COST:
                    continue
                tg = gs[cur] + mc * cc
                nb = (nx, ny)
                if nb not in gs or tg < gs[nb]:
                    gs[nb] = tg
                    heapq.heappush(heap, (tg + self._h(nb, goal), nb))
                    came[nb] = cur
        return None

    def _h(self, a, b):
        return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

    def _recon(self, cf, c):
        p = [c]
        while c in cf:
            c = cf[c]
            p.append(c)
        p.reverse()
        return p

    def _nearest_free(self, cm, g):
        gx, gy = g
        for r in range(1, 15):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = gx+dx, gy+dy
                    if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                        if cm[ny, nx] < self.OBSTACLE_COST:
                            return (nx, ny)
        return None

    def get_debug_info(self):
        return {
            "impassable": self._last_impassable,
            "plan_ms": round(self._last_plan_ms, 1),
            "replanned": self._replanned,
        }
