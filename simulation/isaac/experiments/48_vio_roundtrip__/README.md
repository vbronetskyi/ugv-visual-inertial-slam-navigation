# exp 48: VIO Live Round-Trip (VIO vs RGB-D only)

## Result

**Full round-trip 392м (outbound -> turnaround -> return).**

| Method | ATE RMSE | Scale | Frames | Notes |
|---|---|---|---|---|
| **VIO (RGB-D-Inertial)** | **0.491m** | 1.001 | 5588 (0 lost) | live, synthetic IMU 200Hz |
| RGB-D only (no IMU) | 1.984m | 1.022 | 5543 (1 reset) | offline rgbd_tum |

IMU дає 4× кращу точність. Обидва успішно прошли 392м маршруту.

## Що ми робили

### 1. Pure pursuit їде маршрут
`run_husky_forest.py --synthetic-imu --route south --duration 1200`
- 0.76 м/с актуальна швидкість (cmd 0.25 × Husky 3.4× scaling)
- Route: 797 dense WPs (0.5м spacing) + natural turnaround loop
- Pure pursuit lookahead: 2м (плавна їзда крізь dense WPs)

### 2. Synthetic IMU (не PhysX!)
PhysX sensor непридатний для VIO (contact oscillations).
Генеруємо з GT pose:
```
velocity = position_diff / dt
accel = velocity_diff / dt -> LPF (11 samples = 50ms)
+ Phidgets 1042 noise: gyro 0.005 rad/s, accel 0.02 m/s²
```
Записується в `/tmp/isaac_imu.txt` (append, 200Hz):
```
timestamp gx gy gz ax ay az qx qy qz qw
```

### 3. Камера + GT recording
Isaac Sim записує в `/root/bags/husky_real/isaac_slam_<timestamp>/`:
- `camera_rgb/{sim_time}.jpg` - 640×480 JPG @ 10 Hz
- `camera_depth/{sim_time}.png` - 640×480 uint16 PNG @ 10 Hz
- `groundtruth.csv` - GT pose (x, y, z, yaw)

### 4. VIO Live в паралелі
`rgbd_inertial_live` читає кадри та IMU в real-time, пише позицію в `/tmp/slam_pose.txt`.

### 5. RGB-D only порівняння (offline)
Той самий запис, без IMU. **Критично**: TUM associations ПОВИННІ бути відсортовані **NUMERICALLY** по timestamp (не лексично), інакше 124 map resets.

## Маршрут (south_roundtrip_route.json)

- 397 outbound WPs @ 0.5m spacing
- 17 natural turnaround loop WPs (з original south route, плавна петля)
- 386 return WPs @ 0.5m (reversed outbound)
- Total: 797 WPs
- Clearance: 1.2м від gazebo obstacles + виявлення USD rocks (scene_json + /World/Rocks/rock_XXX)

Критично: USD obstacles треба враховувати при плануванні маршруту. `/tmp/gazebo_models.json` не має камені з USD сцени - це призводило до колізій на попередніх проїздах.

## Файли для відтворення

### config
- `config/vio_th160.yaml` - ORB-SLAM3 RGB-D-Inertial config (3000 features, 12 levels)
- `config/rgbd_only.yaml` - ORB-SLAM3 RGB-D only config (для порівняння)
- `config/south_roundtrip_route.json` - 797 WPs dense route (snapshot використаного)
- `config/south_anchors_fixed.json` - базовий 99-WP route з clearance 1.2м

### scripts
- `run_exp48.sh` - повний pipeline VIO live (Isaac + VIO + запис). `REBUILD_ROUTE=1` щоб перепланувати з нуля
- `build_route.py` - будує 797-WP route з `south_anchors_fixed.json` + USD obstacles (clearance 1.0м, densify 0.5м, natural turnaround loop)
- `extract_usd_obstacles.py` - витягує камені/дерева з `/opt/husky_forest_scene.usd` -> `/tmp/usd_obstacles.json` (потрібен Isaac Python)
- `run_rgbd_only.sh` - RGB-D only на записі (**numeric timestamp sort**)
- `analyze.py` - Umeyama alignment + ATE + CSVs для plots
- `plot_exp48.py` - графік GT vs VIO (exp 45 style)
- `plot_comparison.py` - графік GT vs VIO vs RGB-D
- `run_husky_forest_exp48.py.ref` - snapshot Isaac script (1021 рядків, на випадок змін live версії)
- `RUN_HUSKY_FOREST_SNAPSHOT.md` - короткий опис модифікацій які ми зробили для VIO

### Logs / Data
- `vio_final_camera.txt` - ORB-SLAM3 VIO TUM trajectory (НЕ ЗАЙМАТИ)
- `rgbd_only_camera.txt` - ORB-SLAM3 RGB-D only TUM trajectory
- `exp48_gt_traj.csv`, `exp48_vio_traj.csv`, `exp48_rgbd_traj.csv` - для plots

### Results
- `exp48_vio_roundtrip.png` - VIO (зелена) vs GT (синя)
- `exp48_comparison.png` - VIO vs RGB-D vs GT (3-way порівняння)

### Recording
- `/root/bags/husky_real/isaac_slam_1776423021/` - camera + depth + IMU + GT (5606 frames)

## Команди відтворення

```bash
# Option A: Використати saved route (швидко)
./scripts/run_exp48.sh

# option B: Перепланувати route з нуля (якщо сцена змінилася)
REBUILD_ROUTE=1 ./scripts/run_exp48.sh
# Це робить:
#   1. extract_usd_obstacles.py - витягує obstacles з /opt/husky_forest_scene.usd
#   2. build_route.py - будує 797 WPs з clearance 1.0м + natural turnaround
#   3. Запускає Isaac + VIO

# Після завершення round-trip:
touch /tmp/slam_stop          # зупинити VIO і зберегти trajectory
pkill -9 run_husky_forest      # зупинити Isaac

# RGB-D only для порівняння (offline)
./scripts/run_rgbd_only.sh /root/bags/husky_real/isaac_slam_<timestamp>

# Аналіз: ATE + Umeyama + CSVs
./scripts/analyze.py /root/bags/husky_real/isaac_slam_<timestamp>

# Графіки
python3 scripts/plot_exp48.py
python3 scripts/plot_comparison.py
```

## Залежності (системні, не в експерименті)

- `/workspace/third_party/ORB_SLAM3/` - builds: `rgbd_inertial_live`, `rgbd_tum`
- `/opt/husky_forest_scene.usd` - Isaac Sim scene
- `/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda` - Husky model
- `/workspace/simulation/isaac/scripts/run_husky_forest.py` - live Isaac driver (snapshot в scripts/.ref)
- `/tmp/gazebo_models.json` - regenerated by `run_husky_forest.py` кожен запуск
- `/tmp/slam_routes.json` - route dispatch, `build_route.py` оновлює south

## Ключові висновки

1. **VIO live працює на повному round-trip** - 392м, ATE 0.49м, scale 1.001, 0 lost
2. IMU критичний - RGB-D only має 4× гіршу ATE (1.98м)
3. Synthetic IMU з GT - єдиний спосіб отримати clean IMU в Isaac Sim
4. **Numeric timestamp sort** - без цього RGB-D має 124 resets замість 1
5. **USD obstacles** треба включати в планування маршруту
6. Dense 0.5m WPs + lookahead 2m - плавна їзда без різких поворотів
7. Natural turnaround loop (з original route) - плавний розворот замість U-turn
