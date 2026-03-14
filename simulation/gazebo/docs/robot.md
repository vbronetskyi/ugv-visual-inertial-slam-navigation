# Robot: Clearpath Husky A200

A four-wheeled unmanned ground vehicle (UGV) with skid-steer drive, modeled after the Clearpath Husky A200 platform.

## Physical Specifications

| Parameter | Value |
|-----------|-------|
| Platform | Clearpath Husky A200 |
| Drive type | Skid-steer (differential, 4 wheels) |
| Chassis dimensions | 990 x 670 x 390 mm |
| Chassis mass | 50 kg |
| Wheel mass | 2.5 kg |
| Total mass | 60 kg |
| Color | RAL 1003 Signal Yellow (#F9A900) |

## Wheel System

| Parameter | Value |
|-----------|-------|
| Wheel diameter | 330 mm (radius 0.165 m) |
| Wheel width | 100 mm |
| Track width | 555 mm |
| Wheelbase | 512 mm |
| Friction mu | 5.0 |
| Friction mu2 | 3.0 |
| Contact kp | 1e6 |
| Contact kd | 100 |
| Contact min_depth | 0.0 |
| Contact max_vel | 0.01 m/s |

High friction (mu=5.0) is necessary for skid-steer on uneven terrain; without it, the wheels slip during turns.

## Motion Dynamics

| Parameter | Value |
|-----------|-------|
| Max linear velocity | 1.0 m/s |
| Max angular velocity | 1.0 rad/s |
| Linear acceleration | +/-0.5 m/s^2 |
| Angular acceleration | +/-0.8 rad/s^2 |
| Odometry | DiffDrive plugin, 50 Hz |
| Odometry frame | odom -> base_footprint |

### DiffDrive Plugin

Gazebo plugin `gz::sim::systems::DiffDrive`:
- 4 wheels (2 left + 2 right joints)
- `wheel_separation: 0.555` (distance between wheel pairs)
- `wheel_radius: 0.165`
- Publishes `/odom` (Odometry) and `/tf` (odom -> base_footprint)
- Subscribes to `/cmd_vel` (Twist)

## Sensors

### Intel RealSense D435i (RGB-D Camera)

| Parameter | Value |
|-----------|-------|
| Resolution | 640 x 480 |
| Horizontal FOV | 85.2 degrees (1.487 rad) |
| Requested rate | 30 Hz |
| Actual rate | ~10-17 Hz (GPU-limited in simulation) |
| Near clip (color) | 0.4 m |
| Far clip (color) | 300.0 m |
| Near clip (depth) | 0.4 m |
| Far clip (depth) | 10.0 m |
| Depth noise | Gaussian, sigma=0.005 m |
| Intrinsics (fx, fy) | 382.0 |
| Intrinsics (cx, cy) | 320.0, 240.0 |
| Mounting | Front of base_link, x=+0.48 m, z=+0.05 m |
| Gazebo frame | husky_a200/camera_link/rgbd_camera |

The camera publishes 4 topics via ros_gz_bridge:
- `/camera/color/image_raw` (Image) - RGB image
- `/camera/depth/image_rect_raw` (Image) - depth map
- `/camera/depth/color/points` (PointCloud2) - 3D point cloud
- `/camera/camera_info` (CameraInfo) - calibration data

The far clip for RGB is set to 300 m so the camera can see distant objects in the open terrain. Depth is limited to 10 m, matching the real D435i sensor.

### Phidgets Spatial 1042 (Primary IMU)

| Parameter | Value |
|-----------|-------|
| Rate | 250 Hz |
| Accelerometer | +/-8g |
| Gyroscope | +/-2000 deg/s |
| Mounting | On top plate, z=+0.08 m from base_link |
| Topic | `/imu/data` |
| Gazebo frame | husky_a200/imu_link/phidgets_imu |

### D435i Camera IMU

| Parameter | Value |
|-----------|-------|
| Rate | 200 Hz |
| Mounting | Co-located with camera |
| Topic | `/camera/imu` |

### 2D LiDAR (GPU)

| Parameter | Value |
|-----------|-------|
| Type | 360-degree laser scanner |
| Samples | 360 (1-degree resolution) |
| Min range | 0.08 m |
| Max range | 12.0 m |
| Rate | 10 Hz |
| Noise | Gaussian, sigma=0.01 m |
| Mounting | On sensor arch, z=+0.24 m from base_link |
| Topic | `/scan` |
| Gazebo frame | husky_a200/lidar_link/gpu_lidar |

**Note:** The LiDAR is disabled in the current configuration to save GPU resources for camera rendering. It is a simulation-only sensor (the real Husky A200 does not include one). When enabled, it is used by SLAM Toolbox and Nav2 costmaps.

## TF Tree

```
map
 +-- odom                            (SLAM drift correction)
      +-- base_footprint             (DiffDrive odometry)
           +-- base_link             (z=0.165 m above ground)
                |-- camera_link      (x=0.48, z=0.05)
                |    +-- camera_optical_frame  (rotated: rpy=-pi/2 0 -pi/2)
                |         +-- husky_a200/camera_link/rgbd_camera (identity)
                |-- imu_link         (z=0.08)
                |    +-- husky_a200/imu_link/phidgets_imu (identity)
                +-- lidar_link       (z=0.24)
                     +-- husky_a200/lidar_link/gpu_lidar (identity)
```

### Design Principles

- **SDF** (`src/ugv_description/sdf/model.sdf`) - physical model for Gazebo (mass, inertia, collision, sensors, plugins)
- **URDF** (`src/ugv_description/urdf/husky_frames.urdf`) - TF tree for ROS 2 (robot_state_publisher)
- Identity transforms from sensor frames to Gazebo frames (e.g., `camera_optical_frame -> husky_a200/camera_link/rgbd_camera`) ensure correct bridging of sensor data

A single `robot_state_publisher` process publishes all static transforms, replacing what would otherwise be 8 separate `static_transform_publisher` instances and saving process slots.

## Chassis Model

The chassis consists of:
- **Chassis base** - lower dark section (0.99 x 0.67 x 0.15 m)
- **Top cover** - yellow upper panel (0.80 x 0.60 x 0.05 m)
- **Bumpers** - front and rear (0.05 x 0.67 x 0.10 m)
- **Sensor arch** - metal arch with two uprights and a crossbar

Collision box: 0.99 x 0.67 x 0.20 m (simplified for physics).

## Inertial Parameters

Chassis (box inertia for 0.99 x 0.67 x 0.20 m, 50 kg):
- Ixx = 1.924 kg*m^2
- Iyy = 4.500 kg*m^2
- Izz = 5.558 kg*m^2

Wheel (cylinder inertia, 2.5 kg):
- Ixx = Iyy = 0.0181 kg*m^2
- Izz = 0.0340 kg*m^2

## File Structure

```
src/ugv_description/
├── sdf/
│   └── model.sdf              # SDF model: geometry, sensors, plugins
├── urdf/
│   └── husky_frames.urdf      # URDF: TF tree (robot_state_publisher)
├── package.xml
└── CMakeLists.txt
```

## See Also

- [Environment](environment.md) - world terrain, objects, and zones
- [Simulation launch](simulation.md) - how to build, launch, and operate the simulation
