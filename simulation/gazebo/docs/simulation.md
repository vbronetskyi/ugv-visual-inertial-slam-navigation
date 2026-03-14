# Simulation: Launch and Configuration

Full autonomous navigation pipeline for a UGV in a simulated outdoor environment (220 x 150 m).

## Technology Stack

| Component | Version | Purpose |
|-----------|---------|---------|
| Ubuntu | 24.04 LTS | Operating system |
| ROS 2 | Jazzy Jalisco | Middleware |
| Gazebo | Harmonic 8.10 | Physics simulation |
| Nav2 | Jazzy | Autonomous navigation |
| RTAB-Map | 0.22.1 | Visual SLAM (RGB-D + IMU) |
| SLAM Toolbox | Jazzy | 2D LiDAR SLAM (alternative) |
| ros_gz_bridge | 1.0.18 | Gz <-> ROS 2 bridge |
| Flask | - | Web UI |

## Architecture

```
+------------------+    +-------------------+    +-------------------+
|  Gazebo Harmonic  |--->|  ros_gz_bridge     |--->|  SLAM              |
|  (Bullet physics, |    |  (12 topics)       |    |  RTAB-Map or       |
|   sensors, EGL)   |    +-------------------+    |  SLAM Toolbox      |
+------------------+            |                 +---------+----------+
                                |                           | /map
                                v                           v
                        +-------------------+    +-------------------+
                        |  Nav2 Stack        |<--|  Costmaps          |
                        |  NavFn + RPP +     |    |  Global (500x500)  |
                        |  BT + Behaviors    |    |  Local (5x5)       |
                        +---------+----------+    +-------------------+
                                  | /cmd_vel
                                  v
                        +-------------------+
                        |  Web UI (Flask)    |
                        |  :8765             |
                        |  Map + Camera      |
                        +-------------------+
```

## Headless Mode and EGL

The simulation runs without a display (headless) using NVIDIA EGL for camera rendering:

```bash
# Required environment variable for EGL rendering in headless mode
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```

Without this variable, the D435i camera will produce black images. EGL allows Ogre2 (Gazebo render engine) to use the GPU without X11/Wayland.

## SLAM

### RTAB-Map (Primary)

Visual SLAM based on the RGB-D camera (D435i):
- **Input:** `/camera/color/image_raw` + `/camera/depth/image_rect_raw` + `/odom`
- **Output:** `/map` (OccupancyGrid), loop closure corrections, 3D map
- **Grid map:** resolution 0.05 m, range 0.3-5.0 m from depth
- **Visual features:** GFTT (Good Features To Track), max 200

Advantages:
- Matches the real robot hardware (D435i is available on the physical Husky)
- Loop closure for drift correction

Disadvantages:
- Resource-intensive (requires GPU for feature extraction)
- Slow map updates on large maps

### SLAM Toolbox (Alternative)

2D LiDAR SLAM:
- **Input:** `/scan` (LaserScan) + `/odom`
- **Output:** `/map` (OccupancyGrid)
- **Resolution:** 0.05 m
- **Max laser range:** 12 m

Selection via launch argument:
- `slam_type:=rtabmap` - RTAB-Map (default for real robot scenarios)
- `slam_type:=slam_toolbox` - SLAM Toolbox (lighter, suitable for testing)

SLAM Toolbox runs as a lifecycle node and requires a separate `lifecycle_manager` with `bond_timeout: 0.0`.

## Nav2 Stack

### Components

| Node | Package | Role |
|------|---------|------|
| planner_server | nav2_planner | Global planning (NavFn/A*) |
| controller_server | nav2_controller | Local control (RPP) |
| bt_navigator | nav2_bt_navigator | Behavior Tree coordinator |
| behavior_server | nav2_behaviors | Recovery behaviors (Spin, BackUp, Wait) |
| collision_monitor | nav2_collision_monitor | Emergency stop |
| velocity_smoother | nav2_velocity_smoother | Command smoothing |

The docking_server and route_server are excluded to reduce process count.

### NavFn Planner

- Algorithm: A* (`use_astar: true`)
- Tolerance: 0.5 m from goal
- Global costmap: 500 x 500 cells (rolling window), resolution 0.20 m
- `allow_unknown: true` - allows planning through unexplored areas

### RPP Controller (Regulated Pure Pursuit)

- `desired_linear_vel: 0.5` m/s
- `lookahead_dist: 0.8 - 2.5` m (adaptive)
- `rotate_to_heading_angular_vel: 0.8` rad/s
- `use_collision_detection: false` - **disabled** because skid-steer on uneven terrain produces phantom collisions
- `use_regulated_linear_velocity_scaling: true` - slows down on curves
- `allow_reversing: false`

### Costmaps

**Global Costmap** (500 x 500 cells, rolling window):
- Resolution: 0.20 m/cell
- Obstacle Layer: depth camera point cloud (range 0.3-8.0 m)
- Inflation Layer: radius 0.55 m, cost_scaling_factor 2.0
- Update frequency: 0.5 Hz

**Local Costmap** (5 x 5 m):
- Resolution: 0.05 m/cell
- Voxel Layer: LiDAR scan + depth point cloud
- Inflation Layer: radius 0.55 m
- Update frequency: 5 Hz

### Collision Monitor

A rectangular stop zone (1.0 x 0.68 m), slightly larger than the robot chassis:
- Source: LiDAR `/scan`
- Action: immediate stop when an object is detected inside the zone

### Recovery Behaviors

Recovery sequence when the robot gets stuck:
1. **Spin** - rotate in place
2. **BackUp** - reverse a short distance
3. **Wait** - pause
4. Clear costmap and retry

## Web UI

Flask server `tools/web_nav.py` on port **8765**:

### Features
- **2D map** (800 x 800 px):
  - Background: terrain from heightmap (green-brown)
  - SLAM overlay: obstacles (dark) + explored boundary (light blue)
  - Object markers from SDF (trees, rocks, buildings)
  - Robot marker (blue) with heading arrow
  - Trail (yellow trajectory line, up to 2000 points)
  - Goal marker (red cross)
  - Scale bar, compass
- **Camera feed** (320 x 240, MJPEG)
- **Click-to-drive** - click on the map to send a Nav2 goal (waypoint navigation, 3 m steps)
- **STOP** - immediate stop (sends 10 zero cmd_vel messages)
- **AUTO EXPLORE** - traverses 7 predefined waypoints
- **Zoom/Pan** - map scaling and panning

### Coordinate Systems
- Ground truth robot position comes from Gazebo (`gz model -m husky_a200 -p`), in world frame
- Nav2 goals are sent in the map frame (via `map -> odom` TF)
- World-to-map conversion uses the current robot offset in both frames

## Launch

### Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- NVIDIA GPU with EGL support

### Build

```bash
cd /workspace/simulation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ugv_description ugv_gazebo ugv_navigation
source install/setup.bash
```

### Launch Files

| File | Purpose |
|------|---------|
| `full_sim.launch.py` | Main entry point: Gazebo + bridge + SLAM + Nav2 |
| `simulation.launch.py` | Gazebo simulation only (no SLAM or Nav2) |
| `teleop.launch.py` | Manual teleoperation (keyboard or gamepad) |

### Full Launch (Headless)

```bash
# Terminal 1: EGL + Gazebo + SLAM + Nav2
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
ros2 launch ugv_gazebo full_sim.launch.py headless:=true

# Terminal 2: relay for camera_info (required for RTAB-Map)
ros2 run topic_tools relay /camera/camera_info /camera/color/camera_info

# Terminal 3: Web UI
cd /workspace/simulation
python3 tools/web_nav.py
# Open http://localhost:8765
```

### With GUI

```bash
ros2 launch ugv_gazebo full_sim.launch.py
```

### SLAM Variants

```bash
# SLAM Toolbox (lightweight, LiDAR-based)
ros2 launch ugv_gazebo full_sim.launch.py slam_type:=slam_toolbox headless:=true

# RTAB-Map (visual, D435i-based)
ros2 launch ugv_gazebo full_sim.launch.py slam_type:=rtabmap headless:=true
```

### Manual Control

```bash
ros2 launch ugv_gazebo teleop.launch.py          # keyboard
ros2 launch ugv_gazebo teleop.launch.py joy:=true # gamepad
```

Keyboard mapping:
```
i - forward        u - forward-left     o - forward-right
k - stop           j - turn left        l - turn right
, - backward       m - backward-left    . - backward-right
```

### Restart

```bash
# Stop all ROS 2 processes
pkill -f ros2
pkill -f gz
pkill -f ruby  # gz sim wrapper

# Relaunch
ros2 launch ugv_gazebo full_sim.launch.py headless:=true
```

## Bridge (Gazebo <-> ROS 2)

Config file: `src/ugv_gazebo/config/bridge_config.yaml`

| ROS 2 Topic | Gz Topic | Type | Direction |
|---|---|---|---|
| `/clock` | `/clock` | Clock | Gz -> ROS |
| `/cmd_vel` | `/cmd_vel` | Twist | ROS -> Gz |
| `/odom` | `/odom` | Odometry | Gz -> ROS |
| `/tf` | `/tf` | TFMessage | Gz -> ROS |
| `/scan` | `/lidar` | LaserScan | Gz -> ROS |
| `/camera/color/image_raw` | `/rgbd_camera/image` | Image | Gz -> ROS |
| `/camera/depth/image_rect_raw` | `/rgbd_camera/depth_image` | Image | Gz -> ROS |
| `/camera/depth/color/points` | `/rgbd_camera/points` | PointCloud2 | Gz -> ROS |
| `/camera/camera_info` | `/rgbd_camera/camera_info` | CameraInfo | Gz -> ROS |
| `/camera/imu` | `/camera_imu` | Imu | Gz -> ROS |
| `/imu/data` | `/imu/data` | Imu | Gz -> ROS |
| `/gz_poses` | `/world/.../pose/info` | TFMessage | Gz -> ROS |

## Launch Timing Sequence

```
t=0s    Gazebo + spawn robot + bridge + robot_state_publisher + SLAM node
t=10s   SLAM lifecycle manager (configure + activate slam_toolbox)
t=15s   Nav2 nodes (controller, planner, bt_navigator, behaviors, collision_monitor, velocity_smoother)
t=30s   Nav2 lifecycle manager (configure + activate all Nav2 nodes)
```

Delays are necessary so that:
1. SLAM has time to publish `/map` before Nav2 starts
2. The bridge has time to relay the first TF/odom messages

## Known Issues

### Odometry Drift
DiffDrive odometry drifts over time, especially on uneven terrain. SLAM (RTAB-Map or SLAM Toolbox) compensates by correcting the `map -> odom` transform.

### Terrain Mesh Collision
The STL collision mesh (65x65) is a coarse approximation of the visual mesh (257x257). The robot may snag on triangle edges or fall through micro-gaps. The +15 cm Z-offset partially mitigates this.

### Skid-Steer Phantom Collisions
The Regulated Pure Pursuit Controller with `use_collision_detection: true` generates phantom collisions on uneven terrain (the LiDAR sees the ground as an obstacle). Therefore, collision detection is **disabled** in RPP, and real protective stopping is handled by the Collision Monitor node instead.

### Open Terrain SLAM Failure
In large open areas with few visual or geometric features, both SLAM approaches can struggle. RTAB-Map may fail to find loop closures, and SLAM Toolbox may produce distorted maps due to featureless laser scans.

### Camera Far Clip
The far clip for the RGB camera is set to 300 m (instead of the default 10 m) so the camera can render distant objects across the 220 x 150 m terrain.

### EGL Rendering
The camera requires EGL rendering in headless mode. Without the `__EGL_VENDOR_LIBRARY_FILENAMES` environment variable, the camera returns black images.

### Costmap Width/Height Type
In Nav2 Jazzy, the `width` and `height` costmap parameters must be integers; floating-point values cause a SIGABRT crash.

### cgroup PID Limit
The container has a PID limit of 2304. Process count is minimized by:
- Using a single `robot_state_publisher` instead of 8 `static_transform_publisher` instances
- Excluding docking_server and route_server
- Running SLAM Toolbox as a lifecycle node instead of using bonds

## See Also

- [Environment](environment.md) - world terrain, objects, and zones
- [Robot model](robot.md) - Husky A200 platform and sensors
