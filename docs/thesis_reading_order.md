# Thesis reading order

if you want to read the project as if it were a thesis arguement, go in this
order.  each step has a recommended file to read and what question it answers

## 1.  the motivation

[`/README.md`](../README.md) - the 1-paragraph thesis story + headline results
table.  explains why the project has two halves (datasets first, then   
simulation)

## 2.  datasets: what works on real sensors

the dataset half of the project figures out what SLAM methods work on real
outdoor sensors.  read in this order:

1. [`datasets/nclt/README.md`](../datasets/nclt/README.md) - start here.  LiDAR ICP is
   the only thing that works on a 5 Hz fisheye camera + 48 Hz IMU.  shows
   the limits of visual SLAM when the sensors aren't good enough

2. [`datasets/robotcar/README.md`](../datasets/robotcar/README.md) - first working
   visual SLAM.  Stereo at 3.91 m ATE on 834 m.  also hloc cross-season
   localisation at 64.5 % correct.  but Stereo-Inertial doesn't work because
   RobotCar only exposes fused INS, not raw IMU

3. [`datasets/4seasons/README.md`](../datasets/4seasons/README.md) - the "IMU realy
   matters" proof.  same class of stereo camera as RobotCar, but with a real
   2000 Hz IMU.  ATE drops from 3.91 m -> 0.93 m

4. [`datasets/rover/README.md`](../datasets/rover/README.md) - the closest to our
   real Husky setup.  D435i RGB-D + T265 on a small ground robot.  RGB-D wins
   at 0.37 m, stereo fisheye fails without undistortion.  this is why our
   Isaac Sim pipeline uses D435i, not stereo

5. [`datasets/nclt_kaggle/README.md`](../datasets/nclt_kaggle/README.md) - side
   project: MinkLoc3D place recognition scaffold.  not finished for thesis,
   but documented for completeness

## 3.  simulation: building the pipeline

now that we know RGB-D + IMU is the right combination, time to build the
teach-and-repeat pipeline on our own robot

6. [`simulation/README.md`](../simulation/README.md) - gazebo vs isaac overview

7. [`simulation/gazebo/README.md`](../simulation/gazebo/README.md) - early Gazebo
   Harmonic baseline work.  ROS 2 + Nav2 + RTAB-Map.  3 experiments

8. [`simulation/isaac/README.md`](../simulation/isaac/README.md) - Isaac Sim setup.
   why we moved from Gazebo to Isaac (PhysX wheel simulation + realistic
   photorealistic rendering for visual SLAM)

## 4.  the T&R campaign (the main thesis result)

9. [`simulation/isaac/routes/README.md`](../simulation/isaac/routes/README.md) - the
   9-route teach-and-repeat campaign.  results per route, baselines, the
   3 thesis metrics

10. [`simulation/isaac/experiments/74_pure_stock_nav2_baseline/README.md`](../simulation/isaac/experiments/74_pure_stock_nav2_baseline/README.md) -
    the stock Nav2 baseline.  shows what a default Nav2 config does
    without our T&R additions.  our pipeline 100% vs stock Nav2 63% on
    route 09 is the headline comparison

11. [`simulation/isaac/experiments/76_rgbd_no_imu_ours/README.md`](../simulation/isaac/experiments/76_rgbd_no_imu_ours/README.md) -
    the RGB-D-only ablation.  our full pipeline but ORB-SLAM3 in pure RGB-D
    mode (no IMU).  answers how much does the IMU actually buy us

## 5.  the pipeline code

12. [`simulation/isaac/scripts/common/tf_wall_clock_relay_v55.py`](../simulation/isaac/scripts/common/tf_wall_clock_relay_v55.py) -
    the VIO + encoder + anchor fusion publisher.  4 fusion regimes.  most of
    the algorithmic thesis content is in here and in visual_landmark_matcher

13. [`simulation/isaac/scripts/common/visual_landmark_matcher.py`](../simulation/isaac/scripts/common/visual_landmark_matcher.py) -
    the matcher that generates anchor corrections at repeat time (ORB + PnP)

14. [`simulation/isaac/scripts/nav_our_custom/send_goals_hybrid.py`](../simulation/isaac/scripts/nav_our_custom/send_goals_hybrid.py) -
    the Nav2 patch for proactive waypoint projection around obstacles

15. [`simulation/isaac/scripts/analysis/compute_metrics.py`](../simulation/isaac/scripts/analysis/compute_metrics.py) -   
    the 3 thesis metrics (GT WP coverage / endpoint success / localisation drift)

## 6.  failure analysis

16. [`simulation/isaac/experiments/`](../simulation/isaac/experiments/) - 79
    experiments indexed in [`experiment_index.md`](experiment_index.md).  read the
    ones referenced by the scripts' changelogs (e.g. exp 58 drift runaway,
    exp 62 anchor sanity gate)

## 7.  limitations and future work

- no real Husky field tests (only simulation)
- route 02_north_forest still at 51 % coverage, deep forest is the hard case
- accel-noise ablation (exp 70) shows synthetic IMU choices matter
- 2D Nav2 costmap only - no 3D terrain awareness
