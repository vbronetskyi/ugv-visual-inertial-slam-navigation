# navigation experiments - analysis and results

## approach 1: custom pure pursuit + depth dodge (run_husky_nav_v1.py)

### goal
navigate husky along SLAM route waypoints using ORB-SLAM3 for localization,
with reactive depth-based obstacle avoidance.

### architecture
```
slam_routes.json -> pure pursuit -> steer -> wheel velocities
                        up
              SLAM pose (ORB-SLAM3 via /tmp/slam_pose.txt)
                        up
              depth dodge: if obstacle < 3.5m -> override steering
              if blocked > 2s -> plan 5-point detour around obstacle
```

### what worked
- pure pursuit on road waypoints - smooth, stays on road
- SLAM localization - ATE ~1m on road route, GT and SLAM tracks overlap
- depth-based obstacle detection - correctly identifies cones/tent
- stop-plan-execute detour - robot stops, checks which side is clear, plans
  smooth 5-point bypass with 3m lateral offset

![route following](../final/09_nav_v1_route_following.png)

### what failed
- SLAM lost tracking during detours - when robot turns to bypass
  obstacle, camera sees new view, ORB-SLAM3 loses feature matching.
  position jumps 60+ meters. added jump filter (reject >5m jumps) but
  SLAM freezes on stale pose -> robot navigates on wrong position
- **PhysX collision trap** - once robot touches obstacle, skid-steer
  can't reverse on terrain (wheels spin but no traction backwards)
- **depth can't distinguish trees from obstacles** - both are "close
  depth readings". asymmetry check (left vs right) helps for isolated
  obstacles but fails in forest

![detour at tent](../final/10_nav_v1_detour_tent.png)

### results
- road route: robot reached x=35 of 72 (130m / 170m = 76%)
- 3 cone groups: bypassed first (x=-50), stuck at second (x=15)
- tent: detour triggered but SLAM lost -> froze at stale position
- 26/35 waypoints visited, 2 skipped

### conclusion
custom navigation is fragile. SLAM breaks on detours. depth-based
obstacle detection has too many false positives from trees. need
proper navigation stack with costmap and local planner.

**files:** `scripts/run_husky_nav_v1.py`, `scripts/nav_planner.py`

---

## approach 2: Nav2 + MPPI + depth costmap (run_husky_nav2.py)

### goal
replace custom navigation with ROS2 Nav2 stack:
- ORB-SLAM3 saved-map localization (Atlas from mapping run)
- Nav2 global planner (NavFn) on static SLAM map
- depth PointCloud2 -> local rolling costmap for obstacle detection
- MPPI controller for path following + obstacle avoidance

### architecture
```
Isaac Sim -> /camera/depth, /odom, /tf, /clock -> ROS2
         <- /cmd_vel <- Nav2

Nav2 stack:
  map_server -> static SLAM map (from mapping run, no cones/tent)
  NavFn -> global path (between trees, around static obstacles)
  depth -> PointCloud2 -> local costmap (detects new obstacles)
  MPPI -> follows global path, avoids obstacles in local costmap

TF tree:
  map -> odom -> world -> base_link
  (static)  (static)  (Isaac Sim)
```

### setup
1. installed ROS2 Jazzy + Nav2 (34 packages)
2. built occupancy grid from SLAM mapping run (4557 depth frames
   projected to 2D grid using camera trajectory)
3. Isaac Sim publishes RGB-D + odom + tf + clock via OmniGraph
4. Nav2 launch: map_server, controller_server (MPPI), planner_server
   (NavFn), velocity_smoother, bt_navigator
5. sequential waypoint sender: navigate_to_pose one by one

### SLAM occupancy map

built from depth images of mapping run. projects depth pixels to 2D
occupancy grid using SLAM camera poses. ground filtering: only upper
40% of depth image (avoids marking road as occupied).

![slam map](../final/13_slam_occupancy_map.png)

840x400 cells, 0.25m resolution, 254 occupied cells (trees/rocks)

### experiment 1: empty global costmap + MPPI

no static map, NavFn plans straight lines. MPPI handles obstacles
through local costmap only.

result: MPPI sucessfully bypassed cones at x=-50 (went south to
y=-7), but MPPI horizon (2.8s) too short to navigate back to road
after bypass. robot hit pine tree at (-25, -6) that was off-road.

![cone bypass](../final/11_nav2_mppi_cone_bypass.png)

### experiment 2: static SLAM map in global costmap

NavFn plans along the road (between trees). MPPI follows the path.
cones and tent not in static map - MPPI should bypass them via local
costmap.

**result:** NavFn plans correct path along road. MPPI follows it
smoothly at 0.6 m/s. but at cones (x=-50), NavFn path goes straight
through them (not in static map). MPPI horizon too short to go around
-> "failed to make progress" -> controller abort -> waypoints skipped.

![global plan stuck](../final/12_nav2_global_plan_stuck.png)

### experiment 3: static + live depth in global costmap

added depth obstacle_layer to global costmap on top of static map.
NavFn should replan around new obstacles detected by depth.

result: depth obstacles from trees flood global costmap. inflation
(even 0.25m) closes gaps between trees. NavFn can't find path even on
clear road. robot stuck immediately.

### key findings

| experiment | global costmap | cones bypass | tent bypass | distance |
|-----------|---------------|-------------|-------------|----------|
| empty global | - | yes (y=-7) | no (tree hit) | 70m |
| static SLAM map | trees only | no (MPPI stuck) | - | 45m |
| static + live depth | trees + depth | no (path blocked) | - | 20m |

**core problem:** MPPI horizon (56 steps × 0.05s = 2.8s ≈ 2.8m at 1m/s)
is too short to navigate around obstacles that block the global path.
NavFn replanning with live depth doesn't work because depth also sees
trees that are already in static map -> double obstacles -> no passage.

### next steps

try Regulated Pure Pursuit (RPP) controller instead of MPPI:
- RPP follows global path more tightly
- when obstacle detected in local costmap, RPP slows/stops
- Nav2 BT triggers recovery: backup + global replan
- with proper BT (recovery + replan), robot should back up from
  obstacle and NavFn replans on updated costmap

alternative: custom hybrid approach:
- follow global path normally
- when local costmap shows obstacle on path -> switch to local planner
  that plans around obstacle in local costmap only
- after clearing obstacle -> rejoin global path

**files:** `scripts/run_husky_nav2.py`, `config/nav2_husky_params.yaml`,
`config/nav2_husky_launch.py`, `scripts/send_nav2_goal.py`,
`scripts/build_occupancy_map.py`
