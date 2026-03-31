# exp 49: Nav2 + VIO SLAM Full Round-Trip Navigation

## Коротко

**Робот автономно проїхав повний round-trip 395м використовуючи VIO SLAM для локалізації.**

| Metric | Value |
|---|---|
| **ATE RMSE** | **0.534м** на 395м |
| ATE max | 0.891м |
| ATE median | 0.527м |
| **Scale** | **1.001** |
| VIO/GT ratio | 1.001 |
| Tracked frames | 5600 (**0 lost**) |
| WPs reached | **81/91** (0 skipped) |
| Duration | 3188с (~53 хв) |

Nav2 MPPI контролер керує роботом; позиція береться з VIO (не GT). Покриває outbound + natural turnaround loop + return.

## Що це НЕ

**Це НЕ teach-and-repeat з VIO trajectory.** Маршрут - pre-planned `south_roundtrip_route.json` (797 dense WPs, з exp 48 - планувалися з urahuvанням USD obstacles). VIO використовується тільки для локалізації в реальному часі.

Для справжнього teach-and-repeat (майбутні експерименти з перешкодами):
- Teach pass: VIO записує trajectory -> `vio_final_camera.txt`
- Repeat pass: використати VIO trajectory як reference path, планувати обходи перешкод від неї

## Архітектура (3-phase)

### phase 1 - VIO Warmup (~60с pure pursuit)

```
Isaac Sim
  --synthetic-imu --route south
  ├── camera RGB + Depth -> recording dir
  ├── synthetic IMU 200Hz -> /tmp/isaac_imu.txt (append format)
  └── GT pose -> /tmp/isaac_pose.txt

rgbd_inertial_live (ORB-SLAM3 VIO)
  ├── reads camera + IMU in real-time
  ├── performs VIBA 1+2 calibration
  └── writes pose -> /tmp/slam_pose.txt

tf_wall_clock_relay --use-gt
  └── publishes GT as TF (тимчасово, поки VIO warmup)
```

Чекаємо 200+ VIO frames -> готово до переключення.

### phase 2 - Switch to VIO Localization

```
kill tf_wall_clock_relay (GT mode)

tf_wall_clock_relay --slam-encoder (VIO+encoder fusion)
  ├── reads SLAM pose (/tmp/slam_pose.txt)
  ├── SE(3)->SE(2) alignment на першу VIO позицію
  ├── encoder fallback коли SLAM stale
  └── publishes /tf + /odom

ros2 launch nav2_launch.py
  ├── map_server (blank_south_map)
  ├── controller_server (MPPI)
  ├── planner_server (SmacPlanner2D)
  ├── behavior_server (spin, backup, wait)
  ├── bt_navigator (recovery BT)
  └── velocity_smoother (cap cmd_vel)

ros2 param set /controller_server progress_checker.movement_time_allowance 120.0
```

Після переключення VIO err одразу < 0.05м.

### Phase 3 - Roundtrip Goals

```
send_roundtrip_goals.py --route south_roundtrip_route.json --spacing 4.0
  ├── Loads 797 dense WPs
  ├── Subsamples to 91 goals @ 4m spacing (46 outbound + 45 return)
  ├── Auto-detects start: closest WP in outbound half (handles warmup drift)
  ├── Sends to /navigate_to_pose action
  └── Reports REACHED/SKIPPED + duration
```

## Accuracy по сегментах

| Segment | ATE RMSE | ATE max | X range | Zone |
|---|---|---|---|---|
| 0-20% | 0.737m | 0.891m | -94..-35 | Outbound forest |
| 20-40% | 0.329m | 0.725m | -35..27 | Open field approach |
| 40-60% | **0.225m** | 0.374m | 27..30 | **Turnaround loop - найточніше** |
| 60-80% | 0.563m | 0.722m | 30..-35 | Return open field |
| 80-100% | 0.639m | 0.771m | -35..-94 | Return forest |

Turnaround loop (найкраща секція) - VIO тримає 0.23м при повороті на 180°.

## Порівняння з попередніми

| Exp | Метод | Route | ATE RMSE | Frames | Lost | WPs Success |
|---|---|---|---|---|---|---|
| 46 offline | Pure pursuit, offline VIO | 70% outbound | 0.10m | - | 0 | - |
| 47 | **Nav2 + VIO** | 193m outbound | 0.15m | 2604 | 0 | 33/39 |
| 48 | Pure pursuit + VIO live | 392m roundtrip | 0.49m | 5588 | 0 | N/A |
| **49** | **Nav2 + VIO** | **395m roundtrip** | **0.53m** | **5600** | **0** | **81/91** |

Exp 49 = exp 47 розширений на повний round-trip (в 2× довший маршрут). VIO accuracy зберігається, 0 lost frames.

## Файли та відтворюваність

### config/
- **`vio_th160.yaml`** - ORB-SLAM3 RGB-D-Inertial config (3000 features, 12 levels, ThDepth=160)
- `nav2_params.yaml` - Nav2 (MPPI vx_max=0.5, SmacPlanner2D, ObstaclesCritic)
- **`nav2_launch.py`** - Nav2 launch з velocity_smoother
- **`nav2_bt_exp45.xml`** - Behavior Tree з bounded recovery (3 retries × 0.4m backup)
- **`south_roundtrip_route.json`** - 797 dense WPs (з exp 48, з USD obstacle awareness)
- **`blank_south_map.{pgm,yaml}`** - empty 200×60 map для Nav2 static_layer

### scripts/
- `run_exp49.sh` - повний 3-phase pipeline (одна команда)
- `send_roundtrip_goals.py` - goals follower для full roundtrip, auto-detects start WP
- **`analyze.py`** - ATE + Umeyama alignment + segment analysis + CSVs for plots
- **`plot_exp49.py`** - графік GT vs VIO (exp 45 style)
- `run_husky_forest.py.ref` - snapshot Isaac driver (~1020 рядків, може змінюватися)
- **`tf_wall_clock_relay.py.ref`** - snapshot TF relay (modes: `--use-gt`, `--slam-encoder`)
- `plot_trajectory_map.py.ref` - snapshot plot helper (exp 45 стиль)

### logs/
- `vio_final_camera.txt` - **VIO TUM trajectory (5600 poses, НЕ ЗАЙМАТИ)**
- `exp49_gt_traj.csv`, `exp49_vio_traj.csv` - для plots (body-frame, timestamp matched)
- `isaac.log`, `vio.log`, `tf_slam.log`, `nav2.log`, `goals3.log` - full run logs
- `run_info.txt` - recording dir path

### results/
- **`exp49_roundtrip.png`** - GT (синя) vs VIO (червона) на 395м

### recording (поза experiments/)
- `/root/bags/husky_real/isaac_slam_1776428973/` - camera + depth + IMU + GT (5601 frames)

## Команди відтворення

```bash
# Повний pipeline (3-phase, ~60 хв)
./scripts/run_exp49.sh

# Коли round-trip закінчиться:
touch /tmp/slam_stop
pkill -9 -f "run_husky_forest|rgbd_inertial|tf_wall|controller_server|planner_server|bt_nav|behavior|velocity_smooth|lifecycle|map_server|send_roundtrip"

# Аналіз (читає run_info.txt автоматично)
python3 scripts/analyze.py

# Графік
python3 scripts/plot_exp49.py
```

## Залежності (поза experiments/)

- `/workspace/third_party/ORB_SLAM3/Examples/RGB-D-Inertial/rgbd_inertial_live` - ORB-SLAM3 VIO binary
- `/workspace/simulation/isaac/scripts/run_husky_forest.py` - Isaac Sim driver (snapshot в scripts/.ref)
- `/workspace/simulation/isaac/scripts/tf_wall_clock_relay.py` - TF relay (snapshot в scripts/.ref)
- `/workspace/simulation/isaac/scripts/plot_trajectory_map.py` - plot helper (snapshot в scripts/.ref)
- `/opt/husky_forest_scene.usd` - Isaac Sim scene (містить /World/Rocks, /World/Trees, /World/ShrubCol)
- `/tmp/slam_routes.json` - route dispatcher
- Nav2 Jazzy (`ros-jazzy-nav2-bringup`)

## Ключові висновки

- Nav2 + VIO працює на full round-trip - 395м, ATE 0.53м, 0 lost frames, 81/91 WPs
2. **VIO warmup критичний** - 200+ frames pure-pursuit перед переключенням на VIO TF
3. Turnaround loop стабільний - ATE 0.23м (найточніше в run)
- Scale 1.001 - VIO ідеально калібрований на всьому round-trip
5. Pre-planned маршрут + VIO localization - перевірена архітектура
6. **Next step (exp 50+)**: teach-and-repeat - VIO trajectory з exp 48 як reference path для навігації з обходом перешкод
