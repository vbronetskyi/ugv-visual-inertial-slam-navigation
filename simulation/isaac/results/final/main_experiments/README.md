# development arc, milestone runs

*[thesis root](../../../../../README.md) > [simulation](../../../../README.md) > [isaac](../../../README.md) > [results/final](../README.md) > main_experiments*

18 figures rendered in the same `plot_trajectory_map` template as the
per-route plots in `../`. Source CSVs come from
`/root/isaac_archive/experiments/<NN>/`, the live
`simulation/isaac/experiments/<NN>/` tree, or the rosbag groundtruth
files where the run produced no separate trajectory dump.

The script that builds them lives at
[`../../../scripts/analysis/make_dev_history_plots.py`](../../../scripts/analysis/make_dev_history_plots.py).

This is the visual evidence trail behind chapter 5 §5.3 (iterative
process).

## what each plot shows

| stage | exp | plot | one line |
|---|---|---|---|
| baseline SLAM | 02 | [`02_slam_rgbd_only.png`](02_slam_rgbd_only.png) | plain RGB-D mapping mode, 58% reached, 5-10 m drift on the way back |
| baseline SLAM | 03 | [`03_slam_localization.png`](03_slam_localization.png) | same scene with a pre-built atlas - fewer lost frames, drift after bypass still there |
| recovery probe | 27 | [`27_fullpath_lateral_snap.png`](27_fullpath_lateral_snap.png) | silent SLAM stall - pose froze, 44% before failure, no warning fired |
| first T&R | 30 | [`30_teach_and_repeat.png`](30_teach_and_repeat.png) | encoder + IMU + ORB anchors, 51 m of 196 m, drift 1.3-1.8 m |
| method shift | 31 | [`31_gap_navigator.png`](31_gap_navigator.png) | gap-based navigator, gap = 0.67 + 2x0.3 m, scoring 40/25/20 split |
| obstacles, road | 35 | [`35_road_obstacle_avoidance.png`](35_road_obstacle_avoidance.png) | reactive gap-nav: clean road runs full, cone group wedges the robot |
| hybrid attempt | 39 | [`39_hybrid_navigator.png`](39_hybrid_navigator.png) | gap-nav for cruise + A* on local grid when obstacles appear |
| Nav2 + GT proof | 41 | [`41_trajectory_follow.png`](41_trajectory_follow.png) | Nav2 with GT pose, 43/43 on clean road, 41/43 with four barriers |
| VIO becomes primary | 49 | [`49_nav2_vio_roundtrip.png`](49_nav2_vio_roundtrip.png) | 395 m roundtrip on raw VIO, ATE 0.534 m, 81/91 WPs |
| key idea | 55 | [`55_visual_teach_repeat.png`](55_visual_teach_repeat.png) | first run of the PnP-RANSAC visual landmark matcher |
| projection cap | 56 | [`56_projection_cap_1m.png`](56_projection_cap_1m.png) | 1 m WP projection cap - 2.9x longer run than 55, 88% WPs |
| caught bug | 58 | [`58_route_sanitize_accum.png`](58_route_sanitize_accum.png) | running landmark accumulator drifted on itself, peak 80+ m |
| finisher | 60 | [`60_final_approach_10cm.png`](60_final_approach_10cm.png) | 10 cm tolerance on the last WP, no skip on final five |
| sanity gate | 62 | [`62_tight_detour_anchor_sanity.png`](62_tight_detour_anchor_sanity.png) | anchor consistency tightened from 10 m to 3 m |
| best-of-breed | 64 | [`64_best_of_breed.png`](64_best_of_breed.png) | exp 59 base + STOP fix + precise finisher, 85/95 WPs |
| IMU ablation | 70 | [`70_imu_fidelity.png`](70_imu_fidelity.png) | extra accelerometer noise, drift goes 0.74 → 0.69 m, scale matters more |
| stock baseline | 73 | [`73_stock_nav2_baseline.png`](73_stock_nav2_baseline.png) | stock Nav2 without our matcher, stalled 56 m short of the apex |
| no-Nav2 ablation | 75 | [`75_gap_navigator_teach_wp.png`](75_gap_navigator_teach_wp.png) | gap-nav driven by teach WPs only - no Nav2, no teach map |

## extra: anchors in action

two stand-alone figures for the matcher chapter:

- [`anchors_in_action.png`](anchors_in_action.png) - overlays every
  matcher attempt from exp 64 (479 accepted, 71 rejected, 43 with no
  nearby teach landmarks) on top of the south-route trajectory, with
  arrows showing where each accepted correction pulled the pose.
  rendered by
  [`../../../scripts/analysis/make_anchor_action_plot.py`](../../../scripts/analysis/make_anchor_action_plot.py).
- [`anchor_effectiveness.png`](anchor_effectiveness.png) - quantitative
  proof that anchors reduce drift. err over distance with anchor
  publish ticks, plus a boxplot of err versus time-since-last-anchor.
  on exp 56: fresh anchor → 0.26 m median err, silent for over 5 min
  → 4 m median, 8.6 m max. rendered by
  [`../../../scripts/analysis/make_anchor_drift_plot.py`](../../../scripts/analysis/make_anchor_drift_plot.py).

## visual style

every figure uses the same template as the per-route plots:

- scene background - trees, shrubs, rocks, houses, road corridor pulled
  from `gazebo_models.json` so layout stays consistent with the campaign
  plots
- trajectory colors - red `#dc2626` for GT (stays legible on green
  grass), orange dashed `#f97316` for VIO/nav pose, dark dashed
  `#0f172a` for teach reference
- obstacles - cone circles + tent rectangles + prop clusters listed in
  the legend (not labelled on the map itself)
- metrics box - bottom-left, monospace, four or five short lines

## regeneration

```bash
cd /workspace/simulation/isaac
python3 scripts/analysis/make_dev_history_plots.py
# writes one PNG per experiment under experiments/<NN>/results/thesis_traj.png
```

then mirror them here:

```bash
for e in 02 03 27 30 31 35 39 41 49 55 56 58 60 62 64 70 73 75; do
    f=$(ls experiments/${e}_*/results/thesis_traj.png)
    cp "$f" results/final/main_experiments/$(basename $(dirname $(dirname $f))).png
done
```
