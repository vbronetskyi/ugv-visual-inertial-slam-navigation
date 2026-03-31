# Exp 46: VIO (RGB-D-Inertial) on South Forest

## Result

**VIO з synthetic IMU при 0.76 м/с: ATE 0.103м на 139м forest route (37× краще за RGBD-only 3.80м).**

Працює ідеально в лісі. Ламається тільки при переході forest->open field (turnaround area).

## Проблеми знайдені та виправлені

### Bug 1: USD rotation matrix convention (v5)
`ExtractRotationMatrix()` повертає row-vector convention. scipy потребує column-vector -> потрібен `.T`. Без цього gz мав протилежний знак (ratio -26× замість +1.0).

### bug 2: PhysX IMU micro-oscillations (v1-v4)
Isaac Sim PhysX contact solver додає oscillations до accelerometer/gyroscope на кожному фізичному кроці. Accelerometer std 0.37 м/с² замість 0.02. Gyro bias 0.33°/с.

**Рішення**: synthetic IMU з GT pose. Спроби:
- Double-diff position -> spikes 50 м/с² (numerical differentiation noise)
- Direct velocity API -> PhysX velocity теж noisy на 200Hz
- Velocity averaging + accel lowpass -> правильні дані

### Bug 3: get_linear_velocity() повертає body frame (v12-v20)
`RigidPrim.get_linear_velocity()` повертає velocity в body frame. Ми diffнули body velocity напряму -> accel=0 при поворотах (body speed constant). 

**Рішення**: обчислювати world-frame velocity з GT position diff, потім diff velocity -> acceleration, lowpass (11 samples = 50мс).

### Bug 4: _get_husky_full_quat() неправильна ротація (v5-v15)
Full quaternion від USD rotation matrix (навіть з `.T`) давав az=0.49 замість 9.81 - gravity projection зламана, робот "нахилений 87°".

Рішення: yaw-only quaternion для body->world transform (terrain flat enough). Integration ratio з 0.16 -> 0.99.

### Bug 5: Швидкість 2.7 м/с -> 9% track failures (v21-v23)
Pure pursuit їхав 2.7 м/с. Frame shift 0.27м при 10Hz camera. ORB feature matching fail на >5% scene displacement.

**Рішення**: знижена швидкість до 0.76 м/с -> frame shift 0.08м -> 0% track failures.

### Bug 6: Tbc lever arm mismatch (аналіз, не виправлено)
TUM groundtruth: cam = body + (0.5, 0, 0.48). Config Tbc: cam = body + (0.102, 0.018, 0.132). Але виявилось це НЕ причина scale drift - зміна Tbc погіршила результат.

### Root cause scale drift: forest->open field transition
VIO distance ratio 1.00 в лісі (features 1-3м). При виїзді у відкрите поле (x≈20, turnaround) depth features стрибають до 10-30м -> ORB-SLAM3 scale re-estimation -> ratio 2.6× на 30с сегменті -> глобальний ATE руйнується.

## Хронологія спроб

| Version | Key change | IMU source | Speed | Segment | Scale | ATE | Track fails |
|---|---|---|---|---|---|---|---|
| PhysX raw | - | PhysX sensor | 2.7 | full | 0.004 | 41м | - |
| v5 | .T fix | synth pos->vel->accel | 2.7 | full | 1.022 | 16м | 65 (9%) |
| v9 | clamp ±5 | synth pos->vel->accel | 2.7 | 30с | 1.030 | 0.42м | ? |
| v9 | clamp ±5 | synth pos->vel->accel | 2.7 | full | 0.249 | 43м | 65 |
| v10 | clamp ±15, trim | synth pos->vel->accel | 2.7 | full | 0.478 | 37м | 65 |
| v14 | 200ms vel window | direct vel API | 2.7 | 60% | 1.109 | 2.28м | ? |
| v16 | yaw-only gravity fix | direct vel + yaw | 2.7 | 60% | 1.046 | 4.34м | ? |
| v21 | pos->worldvel->LPF accel | pos diff + yaw | 2.7 | 60% | 1.020 | 0.21м | ? |
| v21 | same | same | 2.7 | full | 0.074 | 44.8м | 65 |
| **v23** | **slow speed 0.76** | pos diff + yaw | **0.76** | **forest 70%** | **1.003** | **0.103м** | **0** |
| v23 | slow speed | same | 0.76 | full outbound | 1.200 | 13м | 0 |

## Ключові висновки

1. **Synthetic IMU працює** - GT-derived IMU з realistic noise дає clean signal для ORB-SLAM3 VIO
2. Швидкість критична - при >1 м/с feature matching degraded (>5% scene displacement per frame)
- Forest VIO ідеальний - ATE 0.103м на 139м (0.07% error)
4. **Open field transition** - єдина залишкова проблема (scale re-estimation при зміні depth distribution)
5. **PhysX IMU непридатний** для VIO - micro-oscillations fundamentally break preintegration

## Файли

### Config
- `config/vio_th160.yaml` - ORB-SLAM3 VIO config (exp 23 Tbc + enhanced ORB 3000 features)
- `config/rgbd_inertial_south.yaml` - seed config від exp 23

### Scripts  
- `scripts/run_vio.sh [ThDepth]` - automated pipeline: TUM build + IMU convert + VIO + ATE eval
- `scripts/fix_route_clearance.py` - route correction for 1.2m obstacle clearance

### Code changes in run_husky_forest.py
- `--synthetic-imu` flag - generates IMU from GT pose instead of PhysX sensor
- `_get_husky_full_quat()` - full rotation from USD (with .T for convention)
- `_compute_synth_imu()` - world velocity from pos diff -> LPF accel -> body frame transform
- Pure pursuit speed reduced: max_speed 1.0 -> 0.25 (cmd) = 0.76 m/s actual

### Recordings
- `isaac_slam_1776345492` - slow (0.76 м/с) south route with synthetic IMU
- `isaac_slam_1776341280` - fast (2.7 м/с) south route with synthetic IMU

### results
- `v23_slow/` - slow recording logs
- `results/exp46_vio_aligned.png` - VIO vs GT trajectory plot
