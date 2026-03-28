#!/usr/bin/env python3
"""
Hybrid navigator: fast gap nav on open road, A* planner near obstacles.

CRUISE mode: gap navigator (0.55 m/s, reactive)
PLAN mode: A* on obstacle grid (0.3-0.5 m/s, routes around obstacles)

Transition: grid obstacle count + center depth trigger.
Grid accumulates in background during CRUISE -> instant costmap for PLAN.
"""
import math
import numpy as np
from gap_navigator import GapNavigator
from fast_grid_planner import FastGridPlanner
from local_obstacle_grid import LocalObstacleGrid


class HybridNavigator:
    def __init__(self):
        self.gap_nav = GapNavigator()
        self.planner = FastGridPlanner(grid_size=100, resolution=0.2)
        self.grid = LocalObstacleGrid(size=20.0, resolution=0.2)

        self.mode = "CRUISE"

        # Mode transition - tuned for DECAY_RATE=0.997, OBSTACLE_HITS=8.0
        # Roadside steady-state: ~50-70 cells. Cones add ~20-40 more.
        self.PLAN_TRIGGER_CELLS = 100
        self.CRUISE_RESTORE_CELLS = 60
        self.CENTER_OBSTACLE_DIST = 2.5  # only very close center obstacles

        # Speed
        self.CRUISE_SPEED = 0.55
        self.PLAN_SPEED_HIGH = 0.45
        self.PLAN_SPEED_LOW = 0.2

        # Steering smoothing
        self.prev_steer = 0.0
        self.STEER_SMOOTHING = 0.3

    def update(self, robot_x, robot_y, robot_yaw, depth_image,
               desired_heading_error, target_dx, target_dy):
        """
        Main function. Returns (linear_vel, angular_vel, mode, debug).
        """
        # Always update grid
        self.grid.update(robot_x, robot_y, robot_yaw, depth_image)
        gs = self.grid.get_stats()

        center_dist = self._check_center_obstacle(depth_image)

        # Mode transitions
        old_mode = self.mode
        if self.mode == "CRUISE":
            if (gs["obstacle_cells"] > self.PLAN_TRIGGER_CELLS
                    or center_dist < self.CENTER_OBSTACLE_DIST):
                self.mode = "PLAN"
                self.planner.cached_path = None
        elif self.mode == "PLAN":
            # Restore CRUISE when grid clears (obstacles behind us)
            if gs["obstacle_cells"] < self.CRUISE_RESTORE_CELLS:
                self.mode = "CRUISE"

        if old_mode != self.mode:
            print(f"  MODE: {old_mode} -> {self.mode} "
                  f"(grid={gs['obstacle_cells']}, center={center_dist:.1f}m)")

        # Execute
        if self.mode == "CRUISE":
            lin, ang, _, debug = self.gap_nav.compute_cmd_vel(
                depth_image, desired_heading_error)
            if debug.get("blocked_ratio", 0) < 0.1:
                lin = self.CRUISE_SPEED
            debug["mode"] = "CRUISE"
            debug["grid_obs"] = gs["obstacle_cells"]
            debug["grid_pct"] = gs["obstacle_pct"]
            debug["center_dist"] = round(center_dist, 1)
        else:
            lin, ang, debug = self._plan_mode(
                robot_yaw, target_dx, target_dy, gs)

        return float(lin), float(ang), self.mode, debug

    def _plan_mode(self, robot_yaw, target_dx, target_dy, gs):
        wp_dx, wp_dy, path_ok, path_len = self.planner.plan(
            self.grid.grid, target_dx, target_dy)

        pdi = self.planner.get_debug_info()

        if path_ok:
            steer = math.atan2(wp_dy, wp_dx)
            herr = steer - robot_yaw
            herr = math.atan2(math.sin(herr), math.cos(herr))

            smoothed = (self.STEER_SMOOTHING * herr
                        + (1 - self.STEER_SMOOTHING) * self.prev_steer)
            self.prev_steer = smoothed

            ang = float(np.clip(smoothed * 1.5, -0.5, 0.5))

            ha = abs(herr)
            if ha < 0.3:
                lin = self.PLAN_SPEED_HIGH
            elif ha < 0.7:
                lin = (self.PLAN_SPEED_HIGH + self.PLAN_SPEED_LOW) / 2
            else:
                lin = self.PLAN_SPEED_LOW

            debug = {
                "mode": "PLAN", "path_len": round(path_len, 1),
                "plan_ms": pdi["plan_ms"], "impassable": pdi["impassable"],
                "grid_obs": gs["obstacle_cells"], "grid_pct": gs["obstacle_pct"],
                "blocked_ratio": 0, "total_gaps": 0, "valid_gaps": 0,
            }
        else:
            lin = 0.0
            ang = 0.3
            debug = {
                "mode": "NO_PATH", "path_len": 0,
                "plan_ms": pdi["plan_ms"], "impassable": pdi["impassable"],
                "grid_obs": gs["obstacle_cells"], "grid_pct": gs["obstacle_pct"],
                "blocked_ratio": 0, "total_gaps": 0, "valid_gaps": 0,
            }

        return lin, ang, debug

    def _check_center_obstacle(self, depth_image):
        if depth_image is None:
            return 99.0
        h, w = depth_image.shape[:2]
        # Narrow center: 10% of width (cols 288-352) - avoids roadside trees
        c10 = w // 20
        strip = depth_image[h // 3: h * 2 // 3, w // 2 - c10: w // 2 + c10]
        valid = strip[(strip > 0.3) & (strip < 10.0) & np.isfinite(strip)]
        if len(valid) < 10:
            return 99.0
        return float(np.percentile(valid, 5))

    def get_stats(self):
        return {"mode": self.mode, "grid": self.grid.get_stats()}
