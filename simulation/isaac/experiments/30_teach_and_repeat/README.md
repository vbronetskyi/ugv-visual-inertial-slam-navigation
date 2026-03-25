# exp 30: Teach-and-Repeat Navigation

## goal

Побудувати систему навігації яка НЕ залежить від глобальної SLAM локалізації.
Робот запам'ятовує маршрут (teach) і потім повторює його (repeat) використовуючи
wheel encoders, IMU gyro, depth камеру і visual anchor matching.

## Scene

Симуляція в Isaac Sim з Gazebo-converted forest scene:
- **743 об'єкти**: 195 pine + 113 oak + 357 shrub + 28 rock + 40 fallen trees + 10 інших
- Маршрут: south outbound (195m, 97 anchors), U-подібна дуга через густий ліс
- Terrain: нерівний, з bumps до ±0.5m
- Мінімальні проходи між деревами: 2-4m в густих зонах

## approach

```
TEACH:  GT waypoints -> pure pursuit -> record anchors (RGB+ORB+pose) every 2m
        down teleport to start
REPEAT: encoder odom + IMU gyro -> position + heading
        odom_s (distance) -> anchor index -> lookahead target
        VFH depth navigator -> find free path toward route heading
        ORB visual matching -> soft position correction (conf > 0.6)
        stuck detection -> backup + turn + skip recovery
```

## phase 1: Route memory

Source: exp20_south recording (4317 frames, 423s, 396m)

**196 anchors** (99 outbound + 97 return), 2m spacing:
- ORB features: avg 995, min 255
- Depth: 196/196 valid
- Direction field: outbound/return per anchor
- Tree-safe nav route: 643 wp, min clearance +0.93m

## Phase 2: Anchor localizer

Direction-aware ORB matching, Lowe's ratio test 0.75, sliding window [−5,+15]:

| Metric | Outbound | Return |
|---|---|---|
| Mono violations | 3.4% | 0.4% |
| Mean confidence | 0.753 | 0.968 |
| Localization error | 0.55m | 0.58m |
| Speed | 25ms (41 FPS) | 31ms (32 FPS) |

**Обмеження**: self-matching bias (тест на тих же даних). Cross-session
ORB matching дає лише 0.30 confidence - viewpoint divergence 1-2m ламає
feature matching.

## Phase 3: Route following - iterative development

### Iteration 1: GT controller (baseline)

`follower.odom_x/y/yaw = GT` - ізолює controller від localization.

**Результат: 196m (99%), cross-track 0.68m, 0 зіткнень.** Controller працює.

### iteration 2: GT + odom_s + stuck recovery

GT для позиції, odom_s (cmd_vel integration) для route progress. Stuck
detection: backup 2s -> turn 30° -> skip 4m.

**Результат: 191m (99%), 2 stuck events (recovered).**

### Iteration 3: Pure visual matching

Same-session ORB matching для позиції. Confidence 0.30-0.37.

**Результат: 76m (39%), cascade failure** - low confidence -> wrong anchor ->
odom pulled to wrong position -> wrong heading -> diverge.

### iteration 4: cmd_vel + IMU (no GT)

cmd_vel integration + IMU gyro для позиції. 

**Результат: 24m (12%)** - cmd_vel дає 37% slippage error на skid-steer ->
4-7m drift за 30s -> wrong heading -> stuck.

### iteration 5: Encoder odometry + IMU

PhysX pose diff (імітація wheel encoders, 0.5% noise) + IMU gyro.

**Результат: 21m (11%)** - encoder дає 2m drift (краще ніж cmd_vel), але
**anchor correction bug**: odom_s overcounting turning distance -> wrong
anchor index -> correction додає 5m+ error замість виправляти.

### iteration 6: Fixed encoder + depth avoidance

Фікси: (1) odom_s не рахує повороти, (2) anchor correction тільки при
visual conf > 0.6, (3) reactive depth avoidance.

**Результат: 80m (41%), odom_err 2-13m.** 4x покращення від фіксів.
Depth avoidance запобігає фізичним зіткненням, але не вирішує heading
drift на curved path.

### iteration 7: VFH depth navigator (final)

Замінив pure pursuit + depth correction на **VFH (Vector Field Histogram)**:
depth image -> 15 секторів -> desirability = route_weight × route_score +
free_weight × free_score -> найкращий вільний сектор -> cmd_vel.

**Результат: 51m (26%), odom_err 1.3-1.8m** до stuck zone.
- Odom drift під контролем (1.8m vs 4-7m раніше)
- VFH знаходить проходи між деревами (12/15 blocked -> знайшов gap)
- Stuck на dense forest zone (-72, -24): shrub на 1.7m + pines на 4.5m з
  обох боків, прохід <3m -> VFH не може знайти free path

## summary table

| # | Aproach | Position | Distance | odom_err | Status |
|---|---|---|---|---|---|
| 1 | GT controller | GT | **196m (99%)** | 0m | PASS |
| 2 | GT + odom_s + stuck | GT | **191m (99%)** | 0m | PASS |
| 3 | Pure visual | ORB anchors | 76m (39%) | >200m | FAIL |
| 4 | cmd_vel + IMU | cmd_vel | 24m (12%) | 4-7m | STUCK |
| 5 | Encoder + IMU | PhysX diff | 21m (11%) | 2-7m | STUCK (bug) |
| 6 | Fixed encoder + depth | PhysX diff | **80m (41%)** | 2-13m | partial |
| 7 | VFH depth nav | PhysX diff | 51m (26%) | **1.3-1.8m** | dense forest |

## key findings

1. **Controller працює** - з точною позицією проходить 196m (99%)
2. **ORB matching крихкий** - 0.75 offline, 0.30 cross-session. Viewpoint
   divergence >1m ламає feature matching
3. **Encoder odom дає ~2m drift** на curved path - достатньо для road,
   недостатньо для narrow forest (проходи 2-4m)
4. Depth VFH ефективний - знаходить проходи навіть при 10/15 blocked,
   тримає odom_err під 2m
- South forest route надто густий - 743 об'єкти, мінімальні проходи
   2-3m. Потрібна sub-meter позиція або ширший маршрут
- Bug odom_s overcounting - рахування turning distance як forward
   progress зламало anchor correction. Фікс дав 4x покращення (21->80m)

## Artifacts

### Scripts
- `scripts/build_south_route.py` - tree-safe route generator
- `scripts/build_route_anchors.py` - anchor builder (RGB+ORB+depth every 2m)
- `scripts/anchor_localizer.py` - direction-aware ORB matching
- `scripts/route_follower.py` - pure pursuit + adaptive lookahead
- `scripts/depth_navigator.py` - VFH depth-primary navigation
- `scripts/run_husky_teach_repeat.py` - GT-based route following (--use-gt)
- `scripts/run_husky_teach_then_repeat.py` - full pipeline: teach -> repeat
- `scripts/test_localizer_offline.py` - offline localizer test
- `scripts/plot_localization_on_route.py` - localization visualization

### Results
- `results/south_route_full_scene.png` - scene map (build_forest_scene objects)
- `results/full_scene_all_objects.png` - **full scene: 743 Gazebo objects**
- `results/localizer_test_v2.png` - offline localizer test
- `results/localization_on_route.png` - localization error plot
- `results/gt_baseline_south_outbound.png` - GT controller (196m)
- `results/teach_repeat_south_outbound_result.png` - GT+odom_s (191m)
- `results/teach_repeat_approaches_comparison.png` - GT vs visual comparison
- `results/fixed_odom_live_trajectory.png` - encoder+depth (80m)
- `results/vfh_depth_nav_trajectory.png` - VFH depth navigator (51m)

### Config
- `config/south_route.json` - tree-safe waypoints + obstacle positions

### Data
- `/root/bags/husky_real/exp20_south/` - source recording
- `/workspace/simulation/isaac/route_memory/south/` - anchor memory (196 anchors)
- `logs/` - CSV logs from all runs
