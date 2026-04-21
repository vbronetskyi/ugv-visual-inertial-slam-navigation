# navigation experiments - full summary

road route: start (-95,-6) to destination (72,-5), 170m one way.
obstacles: 3 cone groups (x=-50, x=15, x=45) + tent (x=-20).
robot: husky a200, skid-steer, 0.8 m/s max, PhysX wheels on terrain.
slam: ORB-SLAM3 RGBD, D435i camera params (fx=fy=320, 640x480, 10Hz).

---

## 1. nav2 pipeline with GT localization

### architecture

isaac sim writes robot pose to `/tmp/isaac_pose.txt` each frame.
`tf_wall_clock_relay.py` reads it, publishes TF + odom with wall clock
timestamps. nav2 runs with `use_sim_time=false`.

problem: isaac sim OmniGraph always publishes TF on `/tf` with sim_time
timestamps regardless of topicName setting. solution: removed all
TF/odom/clock from OmniGraph. relay handles everything.

DDS zombie participants from killed processes polluted TF buffer.
solution: unique `ROS_DOMAIN_ID` per run, clean `/dev/shm/fastrtps_*`.

### controller selection

MPPI controller gave 0.06 m/s (7.5% of max 0.8). tried: fewer timesteps,
more iterations, higher temperature, fewer critics. no effect - MPPI
optimization underperforms for skid-steer diff-drive.

DWB (Dynamic Window B) gives 0.8 m/s immediately. samples velocity
space directly, no optimization loop.

| controller | speed | notes |
|-----------|-------|-------|
| MPPI 96 steps | 0.06 m/s | too slow, unusable |
| MPPI 56 steps | 0.08 m/s | same problem |
| DWB 1.5s | 0.80 m/s | works for skid-steer |
| DWB 3.0s | 0.00 m/s | sees roadside trees, won't move |

### obstacle avoidance

first attempt: depth only in local costmap, not global. DWB sees cones
in local costmap but NavFn plans through them (not in static map).
robot stuck at x=-50 in backup loop. 44m/170m.

fix: added depth obstacle_layer to global costmap with height and range
filtering. `max_obstacle_height: 1.2` passes cones (1m) but cuts tree
trunks (5m+). `obstacle_max_range: 4.0` sees only what's ahead, not
trees 3-5m to the side.

depth images had sim_time timestamps - costmap message filter dropped
them. built depth-to-pointcloud conversion directly in relay (bypasses
depth_image_proc sync issues).

### result (GT)

182m, 294s, 34/34 waypoints, 4/4 obstacles bypassed.
avg speed 0.62 m/s (0.8 on straights, stops at obstacles).
cone group 1 (x=-50): bypassed south y=-6.9.
tent (x=-20): backup + replan, ~90s delay.
cone groups 2,3: passed without stopping.
return: 35/35 waypoints, clean road.

---

## 2. SLAM mapping mode (no pre-built map)

ORB-SLAM3 RGBD builds map from scratch during navigation. relay reads
`/tmp/slam_pose.txt` and computes map->odom correction.

### wz_max = 0.8 rad/s

137m (75%), 274/970 lost frames (28%). Y drift 10m. SLAM loses tracking
on sharp turns - camera sees new view, feature mismatch. after lost
frames, position jumps 60m+. jump filter (>5m) rejects but SLAM freezes
on stale pose.

### wz_max = 0.4 rad/s

104m (57%), 7/2685 lost (0.9%). Y drift 5.8m. slower turning preserves
feature matching. but slow drift still accumulates - no lateral
constraints on straight road.

---

## 3. SLAM localization mode (pre-built atlas)

first: mapping run without obstacles (pure pursuit on road waypoints).
recorded RGB-D, built atlas (167MB, 519 keyframes, ATE 0.49m on road).
then: localization mode - loads atlas, matches features against it.

110m (60%), 28/1665 lost (1.7%). Y drift 3-5m. cones bypassed.
stuck at x=-23 y=9 - lateral drift after obstacle bypass maneuver.

localization mode better than mapping (1.7% vs 28% lost) because atlas
provides reference features. but drift still accumulates when robot
leaves mapped trajectory (obstacle bypass = new views).

---

## 4. complementary filter (SLAM + cmd_vel odometry)

idea: cmd_vel integration as dead reckoning between SLAM updates.
SLAM corrects long-term, odometry smooths short-term.

### alpha = 0.05 (odometry dominates)

Y drift 18.6m - worse than raw SLAM (9m). skid-steer cmd_vel != actual
motion. wheel slip makes odometry unreliable. odometry pulls fused
position away from SLAM.

### alpha = 0.3 (SLAM dominates)

Y drift 9m. filter tracks SLAM closely, only smooths frame-to-frame
jitter. jump rejection works (>0.5m/tick). but doesn't fix fundamental
SLAM drift.

conclusion: cmd_vel on skid-steer is too noisy for dead reckoning.
filter helps with jitter but not with drift.

---

## 5. IMU integration - ORB-SLAM3 RGBD-Inertial (VIO)

### setup

PhysX IMU sensor on imu_link, 200Hz physics. problems encountered:

**16Hz IMU rate**: OmniGraph ticks at render rate, not physics.
fix: read IMU via `_imu_interface.get_sensor_reading()` in main loop,
write to ring buffer file (`/tmp/isaac_imu.txt`, 100 entries).

**3-6 IMU readings per camera frame**: not enough for preintegration.
fix: linear interpolation in `rgbd_inertial_live.cc` - 7 intermediate
points between each pair -> 41 readings/frame.

**IMU prim rotated in USD**: quaternion (0.7066, 0.0227, 0.7069, -0.0235).
gravity on X axis (+9.81) instead of Z. sensor frame = URF (up-right-forward).
fix: Python conversion URF->FLU before writing to file.

**Tbc matrix calibration**: tried 4 rotation variants.

| variant | rotation | Y drift @60s | X drift @140s |
|---------|----------|-------------|--------------|
| original (cam_Y=-imu_X) | wrong sign | +40m | ~0 |
| corrected (cam_Y=+imu_X) | invert Y | -32m | ~0 |
| A (OpenGL flip Y+Z) | both inverted | -50m | ~0 |
| B (cam_X=+imu_Y, cam_Y=+imu_X, cam_Z=+imu_Z) | best | **5m** | **118m** |

variant B: best Y drift (5m) but quadratic X drift (6m@60s -> 118m@140s).

**accelerometer bias**: NoiseAcc 0.1->0.5 fixed X drift (118m -> 6m).
ORB-SLAM3 trusted accelerometer too much, bias integrated quadratically.

### live mapping result (variant B + NoiseAcc=0.5)

165m, 294s, 38/2578 lost (1.5%). X drift 2.3m (1.4%), Y drift 5m.
2 map resets. best RGBD-I result.

### atlas save failure

ORB-SLAM3 `SaveAtlas` segfaults or produces empty atlas in IMU_RGBD
mode with fragmented maps (32 map resets in offline). `CameraTrajectory.txt`
not saved before crash.

### offline SLAM

associations.txt was sorted alphabetically (10.0->109.9->11.0) instead
of numerically. caused "timestamp older than previous" every 100 frames.
fix: numeric sort. but SLAM still fragmented (32 maps, segfault at save).

### RGBD-I localization on RGBD atlas

loaded RGBD-only atlas (167MB) in IMU_RGBD mode. 53m drift immediately.
IMU preintegration with wrong Tbc destroys pose estimation.

### identity Tbc after FLU conversion

cleared imu_link USD rotation -> PhysX broke (invalid inertia tensor,
garbage accelerations). Python URF->FLU conversion instead. gravity
verified on Z=+9.81. tried identity Tbc (like real D435i reference
config). drift still explodes after 20-30 frames.

### conclusion

PhysX IMU sensor has contact solver micro-oscillations that accumulate
in ORB-SLAM3 preintegration. real MEMS IMU doesn't have this artifact.
VIO unusable with Isaac Sim PhysX IMU without fundamental changes to
either the physics engine or the SLAM algorithm.

---

## 6. IMU gyro yaw fusion (without VIO)

alternative: SLAM RGBD-only for position, IMU gyroscope only for yaw.
no ORB-SLAM3 VIO - gyro integrated directly in relay.

### implementation

`imu_tick()` at 200Hz: read gz (yaw rate in FLU), apply LPF (alpha=0.3),
deadzone (0.08 rad/s - PhysX noise level), integrate into `imu_yaw`.
SLAM yaw correction: alpha=0.15 blends toward SLAM yaw slowly.

### result

102m (56%), cones bypassed. position error 1.2m on straights (vs 8m
without gyro). gyro keeps yaw stable between SLAM updates.

problem: imu_yaw drifted to 6.31 rad during maneuvers - SLAM yaw
correction (alpha=0.15) too slow to pull back. and SLAM yaw conversion
itself unstable at large angles (±pi wrapping).

still stuck at x=-23 y=10 - same as SLAM-only. obstacle bypass takes
robot off mapped trajectory, SLAM has no reference features -> drift.

---

## 7. RGBD-only SLAM final result

best SLAM-based navigation result without IMU.

106m (58%), 19/34 waypoints. cones at x=-50 bypassed (went south y=-7).
3/2550 lost frames (0.1%). stuck at x=-23 y=9 after tent bypass.

### offline SLAM evaluation

full road route recording: 3651 frames, 365s, outbound+return.
RGBD-only offline: 3651/3651 tracked, 0 lost. return error 1.5m
(loop closure). ATE RMSE 7.88m (mostly from turnaround lateral drift).
max lateral 13.5m at turnaround point.

---

## summary table

| method | distance | route % | obstacles | max Y drift | lost % |
|--------|----------|---------|-----------|-------------|--------|
| GT localization | 182m | 100% | 4/4 | 0 | - |
| SLAM mapping wz=0.8 | 137m | 75% | 2/4 | 10m | 28% |
| SLAM mapping wz=0.4 | 104m | 57% | 2/4 | 5.8m | 0.9% |
| SLAM localization | 110m | 60% | bypassed | 3-5m | 1.7% |
| SLAM + filter a=0.05 | 104m | 57% | bypassed | 18.6m | 1% |
| SLAM + filter a=0.3 | 105m | 58% | bypassed | 9m | - |
| SLAM + IMU gyro yaw | 102m | 56% | bypassed | 10m | 0.1% |
| RGBD-I live mapping | 165m | 90% | none | 5m | 1.5% |
| RGBD-only final | 106m | 58% | bypassed | 9m | 0.1% |

## key findings

1. nav2 DWB + depth obstacle layer works for obstacle avoidance when
   localization is accurate. height/range filtering in global costmap
   prevents trees from blocking the road.

2. RGBD SLAM gives ATE 0.49m offline but 5-10m lateral drift online
   on straight roads without loop closure. monocular depth lacks lateral
   constraints.

3. localization mode (atlas) better than mapping (1.7% vs 28% lost)
   but drift pattern similiar.

4. PhysX IMU unusable for ORB-SLAM3 VIO - contact solver micro-oscillations
   accumulate in preintegration. needs real hardware IMU or different
   physics engine.

5. IMU gyroscope alone helps with yaw on straights (1.2m vs 8m error)
   but doesn't fix post-maneuver drift.

6. fundamental limitation: obstacle bypass takes robot off mapped
   trajectory -> SLAM sees new features -> no reference in atlas -> drift.
   this would happen on real hardware too, just less severely with
   stereo camera or IMU.

## plots

- 26_nav2_dwb_gt_obstacle_avoidance.png - GT full route with obstacles
- 27_nav2_gt_vs_slam_localization.png - GT vs SLAM mapping vs localization
- 28_nav2_slam_localization_filter.png - complementary filter comparison
- 29_nav2_slam_filter_comparison.png - filter alpha comparison
- 30_nav2_rgbd_inertial_mapping.png - RGBD-I live mapping
- 31_rgbdi_mapping_route.png - mapping run GT trajectory
- 32_rgbd_slam_vs_gt.png - offline SLAM evaluation
- 33_nav2_rgbd_slam_obstacles.png - RGBD SLAM + obstacles
- 34_slam_gyro_yaw_fusion.png - SLAM + IMU gyro yaw
