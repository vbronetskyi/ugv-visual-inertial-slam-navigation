# exp 47: Nav2 + VIO SLAM Localization

## Result

**Nav2 + VIO SLAM closed-loop навігація на повному outbound маршруті (193м): ATE 0.150м, scale 1.003, 2604 frames, 0 lost. 33/39 WPs reached.**

Робот проїхав весь outbound маршрут (від спавну через ліс до будинків) використовуючи VIO SLAM для локалізації замість GT.

## Що подається на VIO

### Камера (10 Hz)
- RGB 640×480 JPG + Depth 640×480 uint16 PNG (depth в мм)
- Isaac Sim camera: fx=fy=320, cx=320, cy=240 (Pinhole)
- Camera.bf=40.0, ThDepth=160.0 (max depth ~20м для close points)
- Зберігаються в recording dir з sim-time timestamps як filenames

### Synthetic IMU (200 Hz)
PhysX IMU sensor **непридатний для VIO** - contact solver додає micro-oscillations (accel std 0.37 м/с² замість 0.02). Тому IMU генерується з GT pose:

```
World velocity = position_diff / dt        (з GT позиції)
World accel = velocity_diff / dt + LPF     (11 samples = 50мс)
Gravity = [0, 0, 9.81] в world frame
Body accel = R_body_inv @ (world_accel + gravity)
Body gyro = quaternion_diff_to_rotvec / dt  (з yaw-only quaternion)
+ Phidgets 1042 noise: gyro_std=0.005 rad/s, accel_std=0.02 м/с²
```

Записується в `/tmp/isaac_imu.txt` (append mode):
```
timestamp gx gy gz ax ay az qx qy qz qw
```

### ORB-SLAM3 VIO config
- ORB features: 3000 (збільшено з 1500 для лісу - більше features на деревах)
- Pyramid levels: 12 (збільшено з 8 - кращий scale range)
- FAST thresholds: iniTh=10, minTh=3 (агресивніший detection)
- IMU noise params: calibровані під Phidgets 1042 характеристики
- Tbc (camera-to-body): виміряний з URDF Husky D435i

### Чому 0.76 м/с
При >1 м/с feature matching деградує - frame shift >0.10м при 10Hz camera = >5% scene displacement per frame. ORB matcher не знаходить відповідності. В exp 46 при 2.7 м/с було 9% track failures, при 0.76 м/с - 0%.

`max_speed = 0.25` (cmd) × Husky 3.4× scaling = **0.76 м/с actual**.

## Маршрут

### south_anchors_fixed.json
Оригінальний south route мав waypoints ближче 1.2м до дерев/кущів - робот чіплявся за collision meshes. `fix_route_clearance.py` (exp 45) зсунув всі WPs щоб мінімальна відстань до перешкод = 1.2м:
- Double pass: fix -> smooth -> re-fix
- 99 outbound + 98 return = 197 WPs
- Перевірка: 0/197 violations

### Progress checker
Nav2 progress checker (`movement_time_allowance`) збільшений до 120с - Isaac Sim біжить на ~17% real-time, стандартні 15с не вистачає.

## Nav2 + VIO Closed-Loop (фінальний результат)

| Metric | Value |
|---|---|
| **ATE RMSE** | **0.150m** |
| ATE max | 0.333m |
| ATE median | 0.137m |
| Scale | 1.003 |
| VIO/GT ratio | 1.000 |
| Tracked frames | 2604 (0 lost) |
| WPs reached | 33/39 (6 skipped - behind robot after warmup) |
| Route distance | 193m |
| Duration | 1400s |

### Segment analysis

| Segment | ATE RMSE | ATE max | X range |
|---|---|---|---|
| 0-20% | 0.147m | 0.234m | -94..-68 (forest) |
| 20-40% | 0.142m | 0.211m | -68..-35 (deep forest) |
| 40-60% | 0.155m | 0.191m | -35..-10 (forest edge) |
| 60-80% | 0.124m | 0.242m | -10..23 (turnaround) |
| **80-100%** | **0.177m** | **0.333m** | **23..65 (open field)** |

**Всі сегменти < 0.18м RMSE.** Рівномірна точність по всьому маршруту.

## Архітектура (3-phase approach)

### phase 1: VIO warmup (0-30s wall-time)
```
Isaac Sim (pure-pursuit, 0.76 m/s, --synthetic-imu)
  -> camera + IMU -> recording dir
ORB-SLAM3 VIO (rgbd_inertial_live)
  -> ініціалізація з рухом, VIBA 1+2
tf_wall_clock_relay --use-gt
  -> GT локалізація (тимчасово)
```
VIO потребує ~200 frames постійного руху для IMU initialization. Без цього - VIO починає з 2.65м error і circular dependency ламає Nav2.

### Phase 2: Switch to VIO (30s+)
```
tf_wall_clock_relay --slam-encoder (ПЕРЕЗАПУСК)
  -> SE(3)->SE(2) alignment SLAM->world
  -> VIO локалізація з encoder fallback
Nav2 (MPPI + SmacPlanner2D + velocity_smoother)
  -> навігація по waypoints
send_trajectory_goals.py (300s goal timeout)
```
Після переключення: VIO error одразу 0.01-0.04м (calibrated).

### Чому warmup критичний
**Без warmup**: VIO ініціалізується зі стаціонарного робота -> погана IMU preintegration -> 2.65м error -> MPPI генерує мізерну швидкість (0.03 м/с замість 0.21) -> робот ледве рухається -> VIO не трекає -> circular dependency.

**З warmup**: VIO calibrated -> 0.04м error -> MPPI нормальна швидкість (0.21 м/с cmd) -> робот їде -> VIO трекає -> positive feedback.

## Nav2 контролери в Isaac Sim

### MPPI - працює (з warmup)
- `vx_max=0.5`, output ~0.21 м/с cmd при правильній локалізації
- При поганій локалізації (VIO error >2м) output падає до 0.03 м/с

### RPP (RegulatedPurePursuit) - не працює
- Не публікує cmd_vel взагалі (0 messages)
- Причина не з'ясована

### DWB - працює
- Альтернатива MPPI, підтверджено з GT і VIO

### velocity_smoother
- Конфлікт: Isaac Sim публікує /odom з sim_time, Nav2 працює з wall_clock
- Smoother публікує zeros коли не отримує odom -> перезаписує cmd_vel
- Рішення: працює коли MPPI генерує достатню швидкість (>0.1 м/с)

## Root Cause: offline VIO деградація

Offline VIO (1.35м ATE) деградував через batch processing:
- IMU preintegration timestamps не синхронізовані з camera timestamps
- На поворотах (>40° heading change) scale re-estimation ламається
- Drift накопичується і не зникає

Live VIO не має цієї проблеми - кадри обробляються послідовно з правильним timing.

## Порівняння

| | Offline VIO (exp 46) | Offline full (exp 47) | **Nav2+VIO closed-loop** |
|---|---|---|---|
| Route | 70% forest | Full outbound | **Full outbound** |
| ATE RMSE | 0.103m | 1.349m | **0.150m** |
| Scale | 1.003 | 1.034 | **1.003** |
| Lost frames | 0 | 2 | **0** |
| Navigation | pure-pursuit | pure-pursuit | **Nav2 MPPI** |

Nav2+VIO closed-loop 9× краще ніж offline VIO на тому ж маршруті.

## Ключові висновки

- Nav2 + VIO SLAM працює - 33/39 WPs, ATE 0.15м на 193м
2. **VIO warmup обов'язковий** - 200+ frames з pure-pursuit перед Nav2
- Synthetic IMU з GT pose - єдиний спосіб отримати clean IMU в Isaac Sim
4. Швидкість 0.76 м/с - оптимум для VIO (0% track failures)
5. **Progress checker 120с** - потрібний для повільної симуляції (17% real-time)
6. Live VIO >> offline - правильний IMU timing критичний для preintegration

## Файли

### Config
- `config/vio_th160.yaml` - ORB-SLAM3 VIO: 3000 features, 12 levels, Tbc calibration
- `config/nav2_params_rpp.yaml` - Nav2: MPPI/DWB + SmacPlanner2D
- `config/nav2_bt_nobackup.xml` - BT без recovery (для VIO stability)
- `config/south_anchors_fixed.json` - маршрут 197 WPs, 1.2м clearance

### Scripts
- `scripts/run_vio_full.sh` - offline VIO pipeline
- `scripts/analyze_vio_live.py` - Umeyama alignment + segment ATE analysis

### results
- `results/exp47_nav2_vio_final.png` - Nav2+VIO trajectory (193м, ATE 0.15м)
- `results/exp47_vio_full_outbound.png` - offline VIO trajectory (ATE 1.35м)
- `results/exp47_ate_vs_turns.png` - ATE vs turn rate кореляція

### Recordings
- `/root/bags/husky_real/isaac_slam_1776401706` - Nav2+VIO run (2604 frames)
- `/root/bags/husky_real/isaac_slam_1776371616` - pure-pursuit recording (2565 frames)
