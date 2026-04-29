# Experiment index (Isaac Sim)

*[thesis root](../README.md) > docs > experiment index*

79 experiments done in Isaac Sim between ~Feb 2026 (exp 01) and Apr 2026 (exp 76).

these are rough one-line summaries from each experiment's README.  for the full writeup, click the experiment name.


| # | name | headline | README |
|---|---|---|---|
| 01_gt_navigation | `01_gt_navigation` | Establish a baseline for Nav2-based navigation in Isaac Sim using ground-truth (GT) localization. This is the upper boun | [README](../simulation/isaac/experiments/01_gt_navigation/README.md) |
| 01_rivermark | `01_rivermark` | load the NVIDIA Rivermark ready-made outdoor scene in Isaac Sim and evaluate | [README](../simulation/isaac/experiments/01_rivermark/README.md) |
| 02_slam_rgbd_only | `02_slam_rgbd_only` | Replace GT localization (exp 01) with real visual SLAM. ORB-SLAM3 RGBD mode runs in **mapping mode** (no atlas), builds  | [README](../simulation/isaac/experiments/02_slam_rgbd_only/README.md) |
| 03_slam_localization | `03_slam_localization` | Improve over exp 02 (mapping mode, drift) by giving SLAM a pre-built reference map. ORB-SLAM3 localization mode loads an | [README](../simulation/isaac/experiments/03_slam_localization/README.md) |
| 04_slam_complementary_filter | `04_slam_complementary_filter` | Reduce SLAM drift between updates by fusing it with `cmd_vel` integration (dead reckoning). Idea: SLAM corrects long-ter | [README](../simulation/isaac/experiments/04_slam_complementary_filter/README.md) |
| 05_slam_imu_gyro_yaw | `05_slam_imu_gyro_yaw` | Use the IMU gyroscope (which is cheap and has very low noise on the z axis) to provide a stable yaw reference. SLAM RGBD | [README](../simulation/isaac/experiments/05_slam_imu_gyro_yaw/README.md) |
| 06_slam_drift_monitor | `06_slam_drift_monitor` | Detect when robot drifts too far from the planned trajectory and force a return to path maneuver. Idea: if SLAM thinks | [README](../simulation/isaac/experiments/06_slam_drift_monitor/README.md) |
| 07_slam_adaptive_drift | `07_slam_adaptive_drift` | Fix exp 06's drift monitor oscillation with **adaptive thresholds and forward-only waypoints**. | [README](../simulation/isaac/experiments/07_slam_adaptive_drift/README.md) |
| 08_slam_mapping_fusion | `08_slam_mapping_fusion` | Combine three signals: SLAM position + wheel odometry + IMU gyro compass. Use mapping mode instead of localization to av | [README](../simulation/isaac/experiments/08_slam_mapping_fusion/README.md) |
| 09_slam_frame_navigation | `09_slam_frame_navigation` | Stop fighting SLAM drift in world frame. Instead, **navigate entirely in SLAM coordinate frame**: waypoints and robot po | [README](../simulation/isaac/experiments/09_slam_frame_navigation/README.md) |
| 10_slam_frame_localization | `10_slam_frame_localization` | Combine the two best approaches: SLAM frame navigation (exp 09, 141 m) + atlas localization (exp 03). Hypothesis: locali | [README](../simulation/isaac/experiments/10_slam_frame_localization/README.md) |
| 11_slam_frame_improved_skip | `11_slam_frame_improved_skip` | Fix cascading waypoint skip from exp 10. Add safeguards to prevent the skip logic from accidentally finishing the route  | [README](../simulation/isaac/experiments/11_slam_frame_improved_skip/README.md) |
| 12_revert_to_exp09 | `12_revert_to_exp09` | Reproduce exp 09's 141 m result with the original (simple) waypoint logic, after exp 10-11 made things worse with aggres | [README](../simulation/isaac/experiments/12_revert_to_exp09/README.md) |
| 13_rpp_inflation | `13_rpp_inflation` | Replace DWB with **Regulated Pure Pursuit (RPP)** controller. RPP has built-in features that DWB doesn't: | [README](../simulation/isaac/experiments/13_rpp_inflation/README.md) |
| 14_rpp_sanitized_waypoints | `14_rpp_sanitized_waypoints` | (no README) | [README](../simulation/isaac/experiments/14_rpp_sanitized_waypoints/README.md) |
| 15_imu_noise_filter | `15_imu_noise_filter` | Find a way to get clean IMU readings from PhysX in Isaac Sim so ORB-SLAM3 RGBD-Inertial mode can work. Previous attempts | [README](../simulation/isaac/experiments/15_imu_noise_filter/README.md) |
| 16_vio_filter60_attempt | `16_vio_filter60_attempt` | Test ORB-SLAM3 RGBD-Inertial mapping with the cleaner IMU from exp 15 | [README](../simulation/isaac/experiments/16_vio_filter60_attempt/README.md) |
| 17_vio_offline_filter60 | `17_vio_offline_filter60` | Test ORB-SLAM3 RGBD-Inertial offline mode (avoiding live VIO sync bugs from exp 16) with the cleaner IMU from exp 15 (fi | [README](../simulation/isaac/experiments/17_vio_offline_filter60/README.md) |
| 18_vio_zigzag_init | `18_vio_zigzag_init` | Make ORB-SLAM3 RGBD-Inertial SLAM produce a usable trajectory by fixing | [README](../simulation/isaac/experiments/18_vio_zigzag_init/README.md) |
| 19_vio_atlas | `19_vio_atlas` | Build a reusable VIO atlas offline, then use it in live localization mode for navigation. This would let us: | [README](../simulation/isaac/experiments/19_vio_atlas/README.md) |
| 20_localization_comparison | `20_localization_comparison` | Compare ORB-SLAM3 RGBD-only vs RGBD-Inertial (VIO) on all three Husky routes (road, north, south) with a unified IMU pip | [README](../simulation/isaac/experiments/20_localization_comparison/README.md) |
| 21_north_drift_analysis | `21_north_drift_analysis` | Investigate why north RGBD-only ATE was 7.79m in exp 20 (vs old 1.93m baseline), and verify VIO failure mode + IMU confi | [README](../simulation/isaac/experiments/21_north_drift_analysis/README.md) |
| 22_warmup_for_vio | `22_warmup_for_vio` | After confirming VIO offline works on road (exp 18: ATE 0.116m), but fails on forest routes (exp 20: 60+ resets), try pr | [README](../simulation/isaac/experiments/22_warmup_for_vio/README.md) |
| 23_imu_calibration_fix | `23_imu_calibration_fix` | Find and fix all IMU/Tbc calibration errors that were breaking VIO on forest routes (and silently degrading it on road). | [README](../simulation/isaac/experiments/23_imu_calibration_fix/README.md) |
| 24_smooth_waypoints | `24_smooth_waypoints` | After fixing IMU calibration in exp 23 (VIO road ATE 0.089m), test if **smoothing forest waypoints** (removing A* planne | [README](../simulation/isaac/experiments/24_smooth_waypoints/README.md) |
| 25_vio_nav_road | `25_vio_nav_road` | Run Nav2 with obstacles on road route using the best localization. After exp 23's | [README](../simulation/isaac/experiments/25_vio_nav_road/README.md) |
| 26_forward_regression_watchdog | `26_forward_regression_watchdog` | Exp 25 run 4 revealed a recurring failure: after RGBD-only Nav2 bypasses | [README](../simulation/isaac/experiments/26_forward_regression_watchdog/README.md) |
| 27_fullpath_lateral_snap | `27_fullpath_lateral_snap` | Exp 26 discovered that the window-based lateral corridor check missed cases | [README](../simulation/isaac/experiments/27_fullpath_lateral_snap/README.md) |
| 28_nav_reproducibility_diagnosis | `28_nav_reproducibility_diagnosis` | Exps 25–27 kept adding complexity (forward-regression watchdog, lateral | [README](../simulation/isaac/experiments/28_nav_reproducibility_diagnosis/README.md) |
| 29_route_anchor_navigation | `29_route_anchor_navigation` | (no README) | [README](../simulation/isaac/experiments/29_route_anchor_navigation/README.md) |
| 30_teach_and_repeat | `30_teach_and_repeat` | Побудувати систему навігації яка НЕ залежить від глобальної SLAM локалізації. | [README](../simulation/isaac/experiments/30_teach_and_repeat/README.md) |
| 31_gap_navigator | `31_gap_navigator` | Замінити VFH-секторний підхід (exp 30) на gap-based навігатор який знаходить | [README](../simulation/isaac/experiments/31_gap_navigator/README.md) |
| 32_robust_anchoring | `32_robust_anchoring` | (no README) | [README](../simulation/isaac/experiments/32_robust_anchoring/README.md) |
| 33_vio_slam | `33_vio_slam` | Налаштувати ORB-SLAM3 VIO (RGBD + IMU) для south forest route і порівняти | [README](../simulation/isaac/experiments/33_vio_slam/README.md) |
| 34_traversability | `34_traversability` | Render-only vegetation (leaves, grass, ferns, fallen trees) blocks depth camera | [README](../simulation/isaac/experiments/34_traversability/README.md) |
| 35_road_obstacle_avoidance | `35_road_obstacle_avoidance` | Evaluate the teach-and-repeat pipeline on the road route with physical collision | [README](../simulation/isaac/experiments/35_road_obstacle_avoidance/README.md) |
| 36_obstacle_grid | `36_obstacle_grid` | Replace single-frame reactive depth detection with an ego-centric obstacle | [README](../simulation/isaac/experiments/36_obstacle_grid/README.md) |
| 37_grid_planner | `37_grid_planner` | Replace reactive gap navigation with A* path planning on the ego-centric | [README](../simulation/isaac/experiments/37_grid_planner/README.md) |
| 38_fast_planner | `38_fast_planner` | Optimize A* planner from exp 37 to maintain higher speed. Exp 37 proved A* | [README](../simulation/isaac/experiments/38_fast_planner/README.md) |
| 39_hybrid_navigator | `39_hybrid_navigator` | Combine gap navigator speed (0.55 m/s) with A* planner intelligence (routes | [README](../simulation/isaac/experiments/39_hybrid_navigator/README.md) |
| 40_tr_nav2_hybrid | `40_tr_nav2_hybrid` | Single destination goal (70.4, -2.3) sent to Nav2. Nav2 plans on blank static | [README](../simulation/isaac/experiments/40_tr_nav2_hybrid/README.md) |
| 41_trajectory_follow | `41_trajectory_follow` | **42/43 waypoints REACHED, 1 SKIPPED. 98% route completion with solid barriers.** | [README](../simulation/isaac/experiments/41_trajectory_follow/README.md) |
| 42_real_localization | `42_real_localization` | Encoder+compass localization matches GT performance on all tests. | [README](../simulation/isaac/experiments/42_real_localization/README.md) |
| 43_slam_localization | `43_slam_localization` | **SLAM+Encoder fusion achieves 95% completion with obstacles - matching | [README](../simulation/isaac/experiments/43_slam_localization/README.md) |
| 44_south_forest_nav2 | `44_south_forest_nav2` | **Nav2 + teach-repeat fails on dense forest - both sparse and dense waypoints hit the same obstacle cluster at (-19, -20 | [README](../simulation/isaac/experiments/44_south_forest_nav2/README.md) |
| 45_south_forest_smac | `45_south_forest_smac` | **47/99 REACHED / 27 SKIPPED - peak 93m (vs exp 44's 89m with NavFn).** | [README](../simulation/isaac/experiments/45_south_forest_smac/README.md) |
| 46_south_vio | `46_south_vio` | **VIO з synthetic IMU при 0.76 м/с: ATE 0.103м на 139м forest route (37* краще за RGBD-only 3.80м).** | [README](../simulation/isaac/experiments/46_south_vio/README.md) |
| 47_nav2_vio_slam | `47_nav2_vio_slam` | **Nav2 + VIO SLAM closed-loop навігація на повному outbound маршруті (193м): ATE 0.150м, scale 1.003, 2604 frames, 0 los | [README](../simulation/isaac/experiments/47_nav2_vio_slam/README.md) |
| 48_vio_roundtrip__ | `48_vio_roundtrip__` | Full round-trip 392м (outbound -> turnaround -> return). | [README](../simulation/isaac/experiments/48_vio_roundtrip__/README.md) |
| 49_nav2_vio_roundtrip | `49_nav2_vio_roundtrip` | **Робот автономно проїхав повний round-trip 395м використовуючи VIO SLAM для локалізації.** | [README](../simulation/isaac/experiments/49_nav2_vio_roundtrip/README.md) |
| 50_vio_obstacles_removed | `50_vio_obstacles_removed` | (no README) | [README](../simulation/isaac/experiments/50_vio_obstacles_removed/README.md) |
| 51_hybrid_nav2_pp | `51_hybrid_nav2_pp` | Goal: full south-route teach-and-repeat using ORB-SLAM3 RGB-D-Inertial VIO for | [README](../simulation/isaac/experiments/51_hybrid_nav2_pp/README.md) |
| 52_obstacles_v9 | `52_obstacles_v9` | Run the v9 hybrid pipeline (Nav2 planner_server + custom pure-pursuit | [README](../simulation/isaac/experiments/52_obstacles_v9/README.md) |
| 53_proactive_reroute | `53_proactive_reroute` | Exp 52 run 4 failed in a new way: robot drove into the on-route | [README](../simulation/isaac/experiments/53_proactive_reroute/README.md) |
| 54_adaptive_encoder | `54_adaptive_encoder` | Exp 53 reached 37/94 WPs at 540 m before SLAM drifted past recovery. The | [README](../simulation/isaac/experiments/54_adaptive_encoder/README.md) |
| 55_visual_teach_repeat | `55_visual_teach_repeat` | Exp 53 run 2 reached 37/94 WPs, 539 m before SLAM drift ran away (32 m err | [README](../simulation/isaac/experiments/55_visual_teach_repeat/README.md) |
| 56_projection_cap_1m | `56_projection_cap_1m` | Exp 56 inherits the visual teach-and-repeat stack from exp 55 and adds | [README](../simulation/isaac/experiments/56_projection_cap_1m/README.md) |
| 57_depth_correlation | `57_depth_correlation` | Add a second anchor channel (depth-based cross-correlation against the | [README](../simulation/isaac/experiments/57_depth_correlation/README.md) |
| 58_route_sanitize_accum | `58_route_sanitize_accum` | Exp 56 reached 1 390 m and 83 / 94 WPs - the best result so far - but | [README](../simulation/isaac/experiments/58_route_sanitize_accum/README.md) |
| 59_wp_lookahead_detour | `59_wp_lookahead_detour` | Continuous teach-and-repeat (T&R) navigation on the south forest roundtrip, | [README](../simulation/isaac/experiments/59_wp_lookahead_detour/README.md) |
| 60_final_approach_10cm | `60_final_approach_10cm` | Builds on exp 59. Two classes of problem surfaced there and are | [README](../simulation/isaac/experiments/60_final_approach_10cm/README.md) |
| 61_dense_anchors_precise_endpoints | `61_dense_anchors_precise_endpoints` | User-directed follow-up to exp 60. Two priorities: | [README](../simulation/isaac/experiments/61_dense_anchors_precise_endpoints/README.md) |
| 62_tight_detour_anchor_sanity | `62_tight_detour_anchor_sanity` | Drift-focused follow-up to exp 61's failure. Exp 61 inflated the detour | [README](../simulation/isaac/experiments/62_tight_detour_anchor_sanity/README.md) |
| 63_global_reloc | `63_global_reloc` | Target the drift root cause identified accross exps 60-62: | [README](../simulation/isaac/experiments/63_global_reloc/README.md) |
| 64_best_of_breed | `64_best_of_breed` | Minimal-delta experiment: take exp 59 (#1 in composite ranking) and add | [README](../simulation/isaac/experiments/64_best_of_breed/README.md) |
| 65_teach_all_routes | `65_teach_all_routes` | (no README) | [README](../simulation/isaac/experiments/65_teach_all_routes/README.md) |
| 66_teach_road_with_accel_noise | `66_teach_road_with_accel_noise` | Повторити teach-проїзд експ 59 pipeline на full road-roundtrip маршруті | [README](../simulation/isaac/experiments/66_teach_road_with_accel_noise/README.md) |
| 67_repeat_road_no_obstacles_accel_noise | `67_repeat_road_no_obstacles_accel_noise` | Запустити повний exp 59 repeat pipeline на road-маршруті, | [README](../simulation/isaac/experiments/67_repeat_road_no_obstacles_accel_noise/README.md) |
| 68_repeat_road_with_obstacles_accel_noise | `68_repeat_road_with_obstacles_accel_noise` | Exp 59 Pareto-оптимальний pipeline на road маршруті з фізично коректною IMU | [README](../simulation/isaac/experiments/68_repeat_road_with_obstacles_accel_noise/README.md) |
| 69_repeat_road_split_landmarks_accel_noise | `69_repeat_road_split_landmarks_accel_noise` | Усунути 14 m anchor-misfire spike, виявлений у exp 68. | [README](../simulation/isaac/experiments/69_repeat_road_split_landmarks_accel_noise/README.md) |
| 70_imu_fidelity | `70_imu_fidelity` | Перевірити чи заявлені у дипломі моделі шуму synthetic-IMU (Phidgets 1042) | [README](../simulation/isaac/experiments/70_imu_fidelity/README.md) |
| 71_teach_north_forest_accel_noise | `71_teach_north_forest_accel_noise` | (no README) | [README](../simulation/isaac/experiments/71_teach_north_forest_accel_noise/README.md) |
| 72_repeat_north_forest_accel_noise | `72_repeat_north_forest_accel_noise` | Запустити повний exp 59 repeat pipeline на road-маршруті, | [README](../simulation/isaac/experiments/72_repeat_north_forest_accel_noise/README.md) |
| 73_stock_nav2_baseline | `73_stock_nav2_baseline` | Drop-in replacement for our custom `send_goals_hybrid.py + pure_pursuit_path_follower.py + WP lookahead skip + 4-7 m det | [README](../simulation/isaac/experiments/73_stock_nav2_baseline/README.md) |
| 74_pure_stock_nav2_baseline | `74_pure_stock_nav2_baseline` | Compared to exp 73 (navigation-logic ablation), this one **removes the | [README](../simulation/isaac/experiments/74_pure_stock_nav2_baseline/README.md) |
| 75_gap_navigator_teach_wp | `75_gap_navigator_teach_wp` | Reactive depth-based navigator adapted from exp 31/35, driven by a teach | [README](../simulation/isaac/experiments/75_gap_navigator_teach_wp/README.md) |
| 76_rgbd_no_imu_ours | `76_rgbd_no_imu_ours` | Same as our per-route `routes/09_se_ne/repeat` pipeline, but ORB-SLAM3 | [README](../simulation/isaac/experiments/76_rgbd_no_imu_ours/README.md) |
