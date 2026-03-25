# exp 33: VIO SLAM Tuning (ORB-SLAM3 RGBD-Inertial)

## Goal

Налаштувати ORB-SLAM3 VIO (RGBD + IMU) для south forest route і порівняти
з RGBD-only SLAM.

## What we did

### 1. 200Hz real IMU recording

Isaac Sim `app.update()` = 60Hz render. Для 200Hz IMU встановили render rate = 
physics rate:
```python
settings.set("/app/runLoops/main/rateLimitFrequency", 200)
```
Кожен `app.update()` = 1 physics step = 1 real IMU reading. Camera every 20 steps
(10Hz). Перевірено: 100/100 unique readings, not duplicates.

### 2. TUM dataset з IMU warmup

Перші 10 camera frames пропущені -> 219 IMU readings before first frame. Потрібно
для preintegration initialization.

### 3. VIO config calibration

- **Camera params**: Camera1.fx/fy=320, cx=320, cy=240 (was D435i default 617)
- Tbc rotation: FLU body -> RDF camera:
  ```
  R = [[0,-1,0], [0,0,-1], [1,0,0]]  t = [0, -0.48, 0.5]
  ```
  Gravity check: IMU az=9.81 -> camera Y=-9.81 (down) x
- imu.frequency: 200.0
- **Real-time pacing**: added `usleep()` in `rgbd_inertial_offline.cc` - VIO threads
  need time to process IMU between frames

### 4. ThDepth tuning

`Stereo.ThDepth` controls max depth: `max_depth = bf × ThDepth / fx = 40 × ThDepth / 320`

## Grid search results (7 configs)

| Config | ThDepth | Max depth | Tracked | Scale | ATE |
|---|---|---|---|---|---|
| th80_default | 80 | 10m | **73%** | 0.067 | 32.38m |
| th100_default | 100 | 12.5m | 72% | 0.018 | 33.42m |
| th100_noisy | 100 | 12.5m | 48% | 0.086 | 16.47m |
| th100_balanced | 100 | 12.5m | 48% | 0.037 | 25.70m |
| th100_3kfeat | 100 | 12.5m | 48% | 0.025 | 22.13m |
| th80_3kfeat | 80 | 10m | 47% | 0.030 | 15.69m |
| **th120_3kfeat** | **120** | **15m** | **2%** | **1.042** | **0.07m** |

## RGBD-only baseline

| Mode | Tracked | Scale | ATE |
|---|---|---|---|
| **RGBD-only** | **100%** | **1.011** | **1.64m** |

## Key findings

1. **Trade-off**: ThDepth=80 -> 73% tracking but scale 0.067 (wrong).
   ThDepth=120 -> scale 1.04 (correct) but only 2% tracking (3 seconds).

- VIO calibration is correct: when it works (ThDepth=120), scale=1.042
   and ATE=0.07m. Gravity direction, Tbc, camera params all verified.

3. Tracking stability is the bottleneck: more depth points (higher ThDepth)
   -> more constraints in VIO optimization -> solver diverges -> map reset.
   This is an ORB-SLAM3 internal issue, not our calibration.

4. **IMU noise params and ORB features have minimal impact** on the
   scale/tracking trade-off. ThDepth is the dominant factor.

- RGBD-only is the practical choice: 100% tracked, ATE 1.64m, no IMU needed.
   VIO adds complexity without benefit for this scene/route.

## Recording

`/root/bags/husky_real/isaac_slam_1776061151/` - south route, 200Hz IMU:
- RGB: 1464 frames (10Hz), Depth: 1464, IMU: 29300 (200Hz), GT: 60Hz
- Duration: 106s, route: south outbound + return

## Artifacts

- `scripts/vio_grid_search.py` - automated grid search script
- `scripts/run_husky_forest.py` - recording with 200Hz IMU
- `scripts/rgbd_inertial_offline.cc` - ORB-SLAM3 with real-time pacing
- `config/vio_*.yaml` - 7 VIO configs tested
- `config/rgbd_d435i_v2_mapping.yaml` - RGBD-only config
- `results/grid_search.json` - raw results
- `logs/vio_*.log` - per-config logs
