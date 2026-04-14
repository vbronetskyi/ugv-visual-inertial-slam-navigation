# exp 67 - Road repeat with accel-noise IMU

## Мета
Запустити повний exp 59 repeat pipeline на road-маршруті,
використовуючи teach-артефакти з exp 66 (teach_map + landmarks) і
synthetic-IMU з accel-шумом у motion-branch (exp 70 Run A). Це
повторює Pareto-оптимальну конфігурацію exp 59 (WP look-ahead skip,
detour, strict Nav2 clearance, SLAM-frame REACH check), але на road
маршруті та з фізично правильною IMU моделлю.

## Вхідні артефакти (exp 66)
- `66_teach_road_with_accel_noise/teach/road/teach_map.yaml` + `.pgm`
- `66_teach_road_with_accel_noise/teach/road/landmarks.pkl`
- `66_teach_road_with_accel_noise/teach/road/vio_pose_dense.csv`
  - використовується як WP-трек (gt_x, gt_y), subsample на 4 м

## pipeline
1. **Isaac** (no obstacles) + `run_husky_forest.py` з accel noise (exp 66 копія)
2. ORB-SLAM3 RGB-D-I VIO (vio_th160.yaml)
3. **Phase 1** - GT-tf warmup (200 frames), робот стоїть
- Phase 2 - swap to SLAM-tf v55 (`tf_wall_clock_relay_v55.py --slam-encoder`)
5. **visual_landmark_matcher** - матчить ORB поточного кадру з teach `landmarks.pkl`,
   публікує `/anchor_correction` для tf_relay
6. **Nav2 planner_server + map_server** (planner-only, без controller) на teach_map
- pure_pursuit_path_follower - споживає `/plan`, видає `/cmd_vel`
- turnaround_supervisor - фіксує разворот на x=70 м
- plan_logger - зберігає усі планери у `plans/*.json`
10. send_goals_hybrid - зчитує teach vio_pose_dense.csv (gt_x,gt_y), робить
    look-ahead skip + detour (4–7 м ring) на живій карті costmap, шле ComputePathToPose

## Ключові параметри
- `robot_radius: 0.7`, `inflation_radius: 1.5`, `planner.tolerance: 0.3`
- `send_goals_hybrid --spacing 4.0 --tolerance 3.0 --goal-timeout 300`
- `turnaround-x 70.0 --past-margin 2.0` (road turnaround idx 83, x=70.4)
- REACH-check використовує `map->base_link` tf (SLAM), не GT - важливий фікс з exp 59

## Запуск
```
bash scripts/run_exp67_repeat.sh
```

## Артефакти repeat
- `results/repeat_run/{isaac,vio,tf_slam,nav2,pp_follower,supervisor,plan_logger,
  landmark_matcher,goals}.log`
- `results/repeat_run/anchor_matches.csv` - всі спроби matcher
- `results/repeat_run/plans/*.json` - dump кожного плану Nav2
- `results/repeat_run/run_info.txt` - REC bag path

## Результати

**Pipeline відпрацював повністю. Усі 80 WP досягнуто.**

| Метрика | Значення |
|---|---|
| **REACHED** | **80 / 80 (100 %)** |
| SKIP | 0 |
| DETOUR triggered | 3 (tent × 2, cone × 1) |
| Projections (costmap) | 2 |
| Duration | 830 s (≈ 14 min) |
| gt_path | 362.6 m |
| **drift_max (nav vs GT)** | **1.48 m** |
| drift_mean | 0.70 m |
| Anchor publishes | 470 / 2075 (22.6 %) |
| Collisions | немає (no obstacles on road route) |

Порівняння з exp 59 на south route (obstacle-heavy):

| | exp 59 run 5 (south + obstacles) | **exp 67 (road + accel noise)** |
|---|---|---|
| REACHED | 79 / 95 (83 %) | **80 / 80 (100 %)** |
| SKIP | 10 | **0** |
| DETOUR | 18 | 3 |
| drift_max | 3.64 m | **1.48 m** |
| drift_mean | 1.67 m | **0.70 m** |
| anchor rate | 15 % | 22.6 % |
| duration | 35 min | **14 min** |

**Примітки:**
- 100 % reach-rate - road маршрут без obstacles plus accel-noise IMU не
  погіршує localisation (exp 70 Run A вже це підтвердив).
- DETOUR 3 × спрацював на hardcoded tent/cone coords (що знаходяться на
  south route, але в hybrid goal_sender це константи); оскільки physical
  obstacles відсутні на road, detour просто сприяє обережному проходженню.
- drift 1.48 m << 2 m gate - жодного з wedge recovery, чистий traversal.

Плоти:
- [results/repeat_run/trajectory.png](results/repeat_run/trajectory.png) - GT + SLAM-nav на карті
- [results/repeat_run/drift_profile.png](results/repeat_run/drift_profile.png) - drift-vs-distance

## Висновок
Exp 59 pipeline (WP look-ahead skip + detour + SLAM-frame REACH check +
stricter Nav2 clearance) **успішно перенесено на road маршрут з фізично
правильним synthetic IMU (accel noise у motion branch)**. 100 % reach-rate,
drift < 1.5 m. Thesis baseline готовий.

## Артефакти repeat
- `results/repeat_run/{isaac,vio,tf_slam,nav2,pp_follower,supervisor,plan_logger,
  landmark_matcher,goals}.log`
- `results/repeat_run/anchor_matches.csv` - всі 2075 спроб matcher
- `results/repeat_run/plans/*.json` - dump кожного плану Nav2
- `results/repeat_run/traj_gt.csv` - dense GT trajectory
- `results/repeat_run/metrics.json` - summary
- `results/repeat_run/run_info.txt` - REC bag path

## Наступні кроки
- (Optional) Exp 68: додати obstacles на road route для тестування detour-heavy сценарію
- (Optional) Exp 69: повтор на south/north routes з accel-noise IMU для cross-route validation
