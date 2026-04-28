# what each milestone plot is about

context for the writing pass: every PNG in this folder is one run from the
iterative process behind chapter 5. the descriptions below were written after
inspecting the underlying CSVs (trajectories, anchor logs, goal logs) for
each plot, not just summarising the experiment readme. numbers cited are the
numbers the figure was rendered from.

read top to bottom - the experiments form a single arc.



## 02 - plain RGB-D SLAM in mapping mode (road)

source: `02_slam_rgbd_only/results/trajectory.csv`, 2 608 GT samples, 105.6 m of
path. range x ∈ [-95, -22], y ∈ [-7, 9.7].

**what to read off the plot.** a single red GT line on the road. the robot
starts at the western spawn (-95, -6), runs along the road corridor, and
stops short of the turnaround. between roughly x = -50 and x = -22 the line
bulges *above* the road - that is the 5-10 m lateral drift after the obstacle
bypass. there is no jump: the trajectory is continuous, but it leaves the
visible road corridor and never quite comes back.

**why.** ORB-SLAM3 in mapping mode has metric depth from the RGB-D camera but
nothing to constrain side-to-side motion on a feature-poor straight road.
sharp turns at wz = 0.8 rad/s also lost 28 % of frames in the longer of the
two runs. the figure is the canonical silent SLAM drift picture the rest
of the thesis is fixing.



## 03 - same scene, SLAM in localization mode against a pre-built atlas

source: `03_slam_localization/results/trajectory.csv`, 6 000 GT samples,
200.6 m of path. range x ∈ [-95, +30].

**what to read off the plot.** red GT line goes much further east than in
02 - it actually reaches around x = +30 before the run ended. the lateral
bulge above the road is still there but smaller (peaks around y = +7 instead
of +10). the shape is the same as 02 but with the trajectory walked twice
as long before the same kind of failure starts to appear.

**why.** the pre-built atlas (519 keyframes, 167 MB) gives the live tracker
something to match against, so lost-frame rate dropped from 28 % to 1.7 %.
that lets the robot walk further before the cumulative lateral drift gets
bad. but the atlas does not cover the bypass corridor because the teach run
did not bypass anything - once the obstacle pushes the robot off the
mapped trajectory, the atlas stops helping. so the plot shows: tracking is
healthy for longer, but the same drift mode is still latent and grows
once the robot leaves the mapped path.



## 27 - silent SLAM stall (full-path lateral snap)

source: `27_fullpath_lateral_snap/results/exp27_trajectory.csv`, 2 309 GT   
samples, 106.1 m of path. range x ∈ [-95, -20.8], y ∈ [-7, **+25.2**].

**what to read off the plot.** the GT line does not stay on the road. it
peels north from around x = -50 and ends roughly 25 m above the road
corridor - a wide, smooth detour. there is no jump or break in the line,
just a wrong path: the robot quietly drove off into the trees while   
believing it was on the road.

**why.** the SLAM pose froze at (43.6, -3.1) but the watchdog kept advancing
the target index. window-based lateral checks could not see the desync
because all candidate segments were beyond the robot. the watchdog did
*not* fire any warning. exp 27's fixes (full-path search + snap-back) are
what catch this in later runs. visually this is the robot drove blind
plot and it is the trigger for moving away from raw global SLAM.



## 30 - first teach-and-repeat prototype (south route)

source: best `odom_imu_south_*.csv`, 185 samples, GT path 37.3 m, ODOM path
39.3 m. GT x ∈ [-91.5, -72.5], odom x ∈ [-91.6, **-63.0**].

**what to read off the plot.** two lines that start together at the spawn
and slowly fan apart. the orange dashed line (what the pipeline thought)
runs about 9 m further east than the red GT - it believes the robot drove
+-63 m east, but the robot only got to -72 m. odom y also undershoots by
+-3 m. neither line shows discontinuities: it is pure dead-reckoning drift
that the ORB anchor matcher could not catch in time.

**why.** the south route is 196 m total. the robot only finished 37 m
(about 19 %) before SLAM lost the loop and the run was killed. drift is
what you would expect from encoder + IMU at this distance. the ORB
anchors triggered occasionally but their corrections were soft and the
forest is feature-poor enough that the matcher fired rarely.



## 31 - gap-based navigator with forest thinning study

source: longest `gap_nav_south_*.csv`, 186 samples, GT path 27.6 m, ODOM
path 25.9 m. GT x ∈ [-86.6, -75.0], y ∈ [-33, -16].

**what to read off the plot.** two lines (orange odom, red GT) that wind
from the spawn area down a southward arc and *stop*. they both stop in
the same place, around (-80, -22). the GT line has a slight curl at the
end where the robot rotated trying to find a way thorugh. there is no
big visible drift in this snippet because the run is too short - the
first gap problem stops it before drift can grow.

**why.** the gap navigator computed the inflated obstacle profile and
correctly identified that no gap of 1.27 m exists at this point in the
forest - two shrubs at about 1.2 m apart from the robot. the geometric
limit, not localisation, is the failure. the dense-forest run reached
anchor 10/96; the thinned-forest version (every third tree removed)
reached 23/96 (2.3* further) - that is the comparison in the readme,
not in this single figure.



## 35 - same gap nav on the road, with and without obstacles

sources: clean `road_noobs_*.csv` 595 samples, 157.5 m path, x ∈ [-91, +64];
obstacle `road_obs_*.csv` 99 samples, 41.7 m path, x ∈ [-91, **-52.5**].

**what to read off the plot.** two superimposed runs. the green line goes
end to end down the entire road corridor (about 158 m driven). the red
line starts at the same spawn, goes a bit over a third of the way along
the road, and stops dead at x ≈ -50, just before the first cone group
icon. the red line ends before the bulk of the route even begins.

**why.** between cones spaced 1 m apart the physical free width is 0.4 m;
the robot is 0.67 m wide. the gap navigator correctly refuses to push
into a sub-robot gap, so it stalls just before the wall. on a clean road
with no obstacles the same controller goes straight thorugh at 0.62 m/s.
this is the cleanest visualisation in the thesis of why a *purely
reactive* depth navigator cannot bypass an obstacle group.



## 39 - hybrid: gap nav for cruise, A* on a local grid for obstacles

sources: `road_noobs_*.csv` 39 samples 22 m, `road_obs_*.csv` 145 samples,
46.7 m, x ∈ [-91, -50.4].

**what to read off the plot.** the obstacle run looks almost identical to
exp 35: red line stops at x ≈ -50, near the first cone group. the
clean-road sample we kept here is short (only 22 m) but went straight at
cruise speed; the obstacle run has small zigzags right before the stop
where the controller probed for a gap.

**why.** the local grid never accumulated enough cells in time to flip
into A* mode. the gating threshold (>100 occupied cells) was reached
only briefly, so cruise stayed in charge. the controller did slip
between two cones at +25° before pinning on the exit at x ≈ -50, but
recovery did not get it out. the lesson is *not* that A* failed - A*
never ran. it is that the gating from reactive to planned mode was
mistuned; a persistent costmap is the right answer, which is what Nav2
provides in the next experiment.



## 41 - Nav2 with ground-truth pose, with solid barriers

source: `exp41_roundtrip_outbound_traj.csv`, columns t,x,y,yaw (no
GT/odom split - the pose source for this run *was* GT, so the single
trajectory is both).

**what to read off the plot.** one green line that runs the full road
(167 m) and visibly bulges around each obstacle marker. the bypass at
barrier 1 (x = -50) is to the north, around the tent it is symmetric,
at barrier 2 (x = +15) it is to the south, and at barrier 3 (x = +45)
it is back to the south. between barriers the line tracks the road
centre.

**why.** Nav2's planner sees the static obstacle layer + global costmap
and re-plans around each barrier. with GT pose there is no localisation
error, so the bypass that the planner computes lines up exactly with
where the robot ends up in GT. final score: 41 of 43 waypoints reached
with barriers, 43 of 43 on a clean road. this is the figure that
demonstrates the planner side of the stack works given perfect pose
- every later run is a story of replacing GT pose with VIO + matcher
while trying to keep this 95 % bypass rate.



## 49 - first full round trip with VIO localization (no matcher)

sources: `exp49_gt_traj.csv` 5 601 samples 393.6 m, `exp49_vio_traj.csv`
5 600 samples 394.9 m. both x ∈ [-95, +70], y ∈ [-41, -2].

**localization stack.** ORB-SLAM3 RGB-D-Inertial (`rgbd_inertial_live`,
config `vio_th160.yaml`, gyro noise 5e-3, accel noise 2e-2,
IMU.Frequency 200 Hz). NO visual landmark matcher, NO teach landmarks,
NO `/anchor_correction` topic - the pipeline grep for
`anchor|matcher|landmark` only hits the route waypoint file. so this
*is* the anchor-less roundtrip baseline that exp 55 later replaces.

one nuance, since the run script uses the relay's `--slam-encoder`
flag: the actual map→base_link tf published to Nav2 is a fusion -
70% VIO + 30% encoder dead-reckoning, with yaw taken from a compass
(GT yaw + 0.05 rad gaussian noise). the encoder branch is simulated
from GT displacement + 0.5% noise. there is also a sanity check that
falls back to pure encoder if the SLAM-vs-encoder gap exceeds 5 m. on
a real Husky the compass would be a magnetometer; here it is a
simulator cheat for yaw only.

**what to read off the plot.** dashed dark GT line and solid orange line
(*raw* ORB-SLAM3 VIO pose, before relay fusion) tracing a south-side U:
out along y ≈ -5, down to y ≈ -41 around x = +60, back along y ≈ -25.
the two lines visually overlap almost everywhere - the largest gap
between them is under 1 m. raw VIO path is 394.9 m vs GT 393.6 m
(scale 1.001). zero lost frames over 5 600 frames. ATE 0.534 m RMSE
/ 0.891 m max - that is the *raw* VIO ATE, not the fused-pose ATE.

**why this works.** ORB-SLAM3 RGB-D-Inertial after VIBA 1+2 calibration
is a strong visual-inertial estimator on a clean run with no obstacles
to push the robot off course. the figure proves that VIO alone can get
most of the way to a closed loop, and the relay's encoder + compass
blend takes care of the rest. the harder runs - with obstacles,
repetitive scenes, and minute-long matcher outages - start in exp 55.

**what is missing for the campus campaign.** on a real Husky the
compass is replaced by a magnetometer (which is noisier and biased by
nearby metal), and the encoder is real wheel encoders (which slip on
mud and grass). VIO on its own would drift over a >500 m loop in a
feature-sparse forest - that is the failure mode exp 53 hit and what
the visual landmark matcher in exp 55 is designed to fix.



## 55 - visual landmark matcher introduced (PnP RANSAC)

source: `run3_success_outbound_drift_return/trajectory.csv`, 450 samples,
GT path 477.2 m, NAV path 499.7 m. nav vs GT error: mean 4.98 m, max
**18.7 m**, with **3 visible jumps** in the nav line (>5 m between
consecutive samples).

drift along the route:
- at 50 m: 0.3 m (clean outbound)
- at 100 m: 1.2 m
- at 200 m: 2.6 m (around the apex)
- at 300 m: 8.2 m (return half - degraded)
- at 400 m: 8.2 m
- at 477 m: 9.1 m (close to where the run died)

**what to read off the plot.** orange nav-pose line tracks GT closely on
the outbound half of the loop. on the return it visibly fans away,
mostly to the south. three large discontinuities are visible:
- jump @ index 204: 10 m, around (0, -15) → (-9, -14). matcher fired late
  and pulled the pose backwards.
- jump @ 427: **27 m**, (64, -2) → (91, +1). matcher latched on the
  wrong cluster of trees on the way back.
- jump @ 431: 19 m, (90, +2) → (71, +1). the next match correcting
  back toward reality.

**why.** the matcher works in principle (drift kept under 1 m for the
first 100 m), but two structural problems show up. first, it goes silent
in featureless stretches and lets drift grow to 8-9 m, which is enough to
push the pose into wrong-cluster matches. second, the proactive waypoint
projector was shoving WPs *several metres* off the teach line into
free cells; the resulting trajectory was visibly off the teach corridor,
the camera view diverged from the teach images, and the matcher started
matching the wrong building. exp 56 caps the projector to 1 m to break
that loop.



## 56 - projection cap of 1 m on the next waypoint

source: `repeat_run/trajectory.csv`, 1 879 samples, GT path **1 390.2 m**,
NAV path 1 364.8 m. **0 jumps** in the nav line. matcher published 44
out of 305 attempted matches (14 %).

drift along the route:
- at 100 m: 2.4 m
- at 200 m: 1.3 m (matcher caught it back)
- at 400 m: 5.5 m
- at 600 m: 3.6 m
- at 1 000 m: 5.2 m
- at 1 200 m: 8.5 m (max)
- at 1 390 m: 8.2 m, robot back at (-84, -23) ≈ spawn

**what to read off the plot.** orange nav-pose and red GT line trace the
full south loop together. the orange line stays inside the same forest
corridor as GT all the way around and back to within 8 m of spawn - no
jumps, no wedge-recovery teleports. drift is visible as a slow widening
gap on long no-anchor stretches but it never runs away and is always
caught by the next match.

**why.** the projection cap kills the off-corridor feedback loop from
exp 55. wherever a teach WP sits in the matcher's preferred view
direction, the matcher keeps firing; wherever the teach trajectory was
tight against an obstacle, the WP stays put rather than being shoved
several metres into a free cell. this is the first run that closes the
south loop on-board only.



## 58 - caught bug: running landmark accumulator drifted on itself

source: `repeat_run/trajectory.csv`, 2 009 samples, GT path 1 362.7 m.
**NAV path 7 581.7 m** - **5.6* the actual distance driven**.
**197 jumps over 5 m**, biggest five all about 90 m:
- jump @ 719: 95.1 m, (-90.6, -6.1) → (3.0, -23.2)
- jump @ 707: 93.8 m, (-92.8, -6.0) → (-0.2, -20.8)
- jump @ 900: 89.5 m, (-102.4, +4.5) → (-18.1, -25.5)

**what to read off the plot.** the orange nav-pose line is a dense mess.
it teleports across most of the map repeatedly, drawing many long
straight segments from one part of the loop to another. every visible
strand connecting (-100, 0) to the south interior (-25, -25) is one
of these 90 m jumps. the red GT line, by contrast, is a normal smooth
loop on the south side.

**why.** the running landmark accumulator stored fresh landmarks at the
*current* (already drifted) nav pose. on the next traversal the
matcher's nearest-neighbour search picked up its own drifted entries
with high confidence and pulled the pose to them. the next correction
pulled it back. each pull is a discontinuity in the orange line. this
is what a self-poisoning visual map looks like in trajectory space.
feature was disabled in exp 59.



## 60 - precise final-waypoint finisher and no-skip-on-final-five

sources: `repeat_run/traj_gt.csv` 4 003 samples, 468.2 m of GT path;
`traj_slam.csv` 495 samples (sparse log every +-5 s), 480 m. anchors
published 47 / 336 (14 %). x ∈ [-95, +67], y ∈ [-46, -2].

**what to read off the plot.** dashed orange (sparse SLAM nav-pose) and
red GT both trace a clean south loop, with start and end points sitting
on top of each other near (-90, -6). the loop closes visibly within
the marker size. no jumps, no wedges. orange is sparser than red because
it samples at the relay's logging cadence rather than the bag rate.

**why.** the projection cap (from 56), tighter detour ring (2-5 m), 10 cm
final-WP tolerance, no-skip on final five, and STOP-after-RESULT (empty
Path + zero Twist so pure pursuit stops chasing a stale plan) all stack
up cleanly. the precise finisher is what reels the robot in to within
half a metre of the teach endpoint instead of stopping a few metres
short. visually this is the prettiest of the south plots and the
clearest demonstration of the full stack working.



## 62 - anchor sanity check tightened from 10 m to 3 m

sources: `repeat_run/traj_gt.csv` 4 327 samples, 599.2 m, x ∈ [-95, +81],
y ∈ [-56, -6]. (no `traj_slam.csv` in this run - only the sparse log.)
anchors published 336 / 2 578 (13 %).

**what to read off the plot.** red GT line traces an extended south loop
that goes a bit further south (down to y ≈ -56) than the exp 60 loop
(down to y ≈ -46). the orange dashed line follows it through the entire
route. visually the only difference from exp 60 is route shape; what
the run actually proves is in the *absence* of bad anchors - there is
no big single-step jump in the orange line where there easily could
have been one without the gate.

**why.** the new gate rejects anchors that meet *all three*: encoder
disagrees with VIO by more than 2 m, the proposed shift is small
(<0.5 m), and the matcher reports very low covariance. that is
precisely the wrong-building latched signature from earlier runs.
when the gate fires, the encoder vs SLAM α-blend takes over until the
matcher recovers honestly. drift behaviour is similar to exp 60; what
the run buys is robustness on repetitive scene parts the campus
campaign will hit.



## do anchors actually help? - straight comparison from the logs

after parsing every `tf_slam.log` entry across the matcher-active runs, the
question does the matcher reduce drift has a clear quantitative answer.
binning samples by *time since the last accepted anchor* (the
`anchor_age` field in the log) on **exp 56** gives this:

| time since last anchor | samples | median err | max err |
|---|---|---|---|
| just published (<5 s) | 6   | **0.26 m** | 0.46 m |
| recent (5-30 s)       | 5   | 0.32 m     | 0.38 m |
| stale (30-120 s)      | 6   | 0.38 m     | 0.48 m |
| going silent (2-5 min) | 36  | 2.24 m    | 3.21 m |
| silent (> 5 min)      | 1814 | **3.96 m** | 8.61 m |

drift jumps from sub-metre to several metres the moment the matcher stops
firing. on the same scene without a matcher (**exp 53**) drift hit 32 m by
400 m of route. so on the long south loop, **anchors are necessary** -
without them VIO + encoder + compass together cannot constrain drift past
+-400 m.

on the **short** clean road (**exp 49**, 395 m, no obstacles, no matcher)
the same VIO + encoder + compass stack finished with **max err 0.77 m**.
no matcher needed there. the conclusion is route-dependent: anchors are
load-bearing on long forest loops, optional on short clean ones.

**but anchors can also hurt if the gate is loose.** the matcher's own
consistency check rejects only proposed shifts above 5 m. wrong-cluster
matches with shift just under 5 m pass through and pull the pose toward
the wrong tree group. on **exp 64** (which kept exp 59's matcher
parameters and *did not* take exp 62's stricter sanity gate) the same
binning gives:

| time since last anchor | median err | max err |
|---|---|---|
| just published (<5 s) | **6.93 m** | 47.7 m |
| recent (5-30 s)       | 5.27 m     | 59.2 m |
| stale (30-120 s)      | 5.11 m     | 10.8 m |
| silent (>5 min)       | 14.82 m    | 17.8 m |

these numbers are telling: even *fresh* anchors leave the pose nearly
7 m off, and accepted matches occasionally produce 50+ m apparent err
(the wrong-cluster pull hits, then the next match pulls back, the log
samples between snap to whatever the relay had at that moment). the
**figure to use** for this discussion is
[`anchor_effectiveness.png`](anchor_effectiveness.png) - described below.

practical takeaway for chapter 5: matcher + sanity gate (exp 62 style)
is what actually reduces drift; matcher alone, with a loose 5 m gate,
can introduce its own failure mode.

## anchor effectiveness figure

source: [`anchor_effectiveness.png`](anchor_effectiveness.png), rendered
by [`make_anchor_drift_plot.py`](../../../scripts/analysis/make_anchor_drift_plot.py).
data from exp 56 (cleanest matcher-active run, well-tuned configuration).

**what it shows.** two panels stacked vertically.

- **top:** localization error against ground truth as the robot drives
  along the 1 390 m south loop. background is shaded green where the
  matcher just fired and red where it has been silent for over five
  minutes; the in-between colours map to the bucket table above.
  green vertical ticks at the bottom mark every accepted-anchor event.
- **bottom:** boxplots of error grouped by the *time since the last
  anchor*. the medians are annotated explicitly. the leftmost box -
  fresh anchor - sits at 0.26 m. the rightmost - silent for over five
  minutes - sits near 4 m and reaches 8.6 m at the worst.

**how to read it in writing.** point at the top panel where the green
ticks are dense and the line stays low; that is the matcher actively
correcting. point at the long red stretches with no ticks; that is
where dead reckoning takes over and error grows roughly linearly with
distance. point at the bottom panel as the *quantitative* statement of
the longer the matcher is silent, the more drift accumulates.



## 64 - best-of-breed (exp 59 base + STOP fix + precise finisher)

source: `tf_nav_gt.csv` parsed from `tf_slam.log`, 609 samples, GT path
sparse-sampled along the south loop, x ∈ [-99, +61], y ∈ [-54, +1].
**579 unique GT samples** because the log is once every 5 s.
**85/95 waypoints reached**, 7 skipped, 49 minutes total. anchors
published **479 / 4 585 (10.4 %)**.

drift evolution along the run (from `tf_slam.log` `err=…` field):
- 0-105 m: clean, err < 1 m
- 120 m: err spikes to 4.5 m on a marginal anchor
- 200-300 m: err 5-6 m, anchor age oscillates 0.2-120 s
- 308 m: err jumps to 11 m
- 337-477 m: 5-7 m, matcher firing again
- 478 m: anchor age starts climbing past 100 s
- 503 m: err 13 m, anchor_age 296 s
- 534 m: err 14.7 m, anchor_age 446 s
- **577 m onwards** (final 12+ minutes): nav pose **frozen at
  (-84, -8)** while GT is at (-95.9, -20.6) - err 17.3 m, anchor_age
  growing past 700 s.

big jumps in the orange nav-pose line (consecutive samples > 5 m apart):
clusters around the 100-280 m range and again 480-577 m. biggest
single jumps: 18.6 m, 19.5 m, 16.8 m, 16.2 m, multiple in the 10-15 m
range.

**what to read off the plot.** red GT line traces a normal south loop.
the orange nav-pose line is *not* smooth - it has many short strands
connecting one part of the south corridor to another. the visible
scatter to other parts of the map the user noted is exactly this.
near the end of the loop, the orange line stops moving and the red GT
line keeps going for +-13 m more - that is the final 12-minute period
where the matcher gave up and dead-reckoning was decoupled from the
real motion.

**why the orange line jumps.**
1. every published anchor pulls the nav pose toward the matcher's
   PnP-solved pose. with 479 anchors over the run, that is roughly one
   correction every 6 s of drive time. when drift is small the pull is
   sub-metre and invisible. when drift is several metres and the
   matcher fires, the pull becomes a visible step.
2. in the 100-280 m segment the route passes through a feature-sparse
   forest stretch where the matcher's candidates often share descriptors
   between adjacent tree clusters. it occasionally picks the wrong
   cluster, the resulting shift is 10-15 m, and the consistency gate
   in this configuration was not strict enough to reject it. exp 62
   later closes that gap.
3. the final freeze (last +-50 m of route) is the matcher going silent
   for 13 minutes - landmarks within 8 m of the (already-drifted) VIO
   pose simply did not have descriptor matches in the teach landmarks
   set. the pose just stopped getting corrections, the precise finisher
   ran and timed out twice (`PRECISE TIMEOUT END d=1551 cm`,
   `d=1163 cm`), and the run ended.

**caveat - this run is *not* the cleanest matcher behaviour.** the
best-of-breed label was based on composite TRACK / loop-closure /
smoothness metrics from exp 59. it specifically excluded exp 62's
stricter sanity gate. the per-bucket error analysis (see the "do
anchors actually help?" section above) shows that, in this
configuration, the matcher's median fresh-anchor err is 6.93 m -
nearly twenty times worse than exp 60's 0.39 m. the matcher is
publishing wrong-cluster matches at the rate of +-1 per 6 seconds
and the planner faithfully follows them, smoothed out only by the
controller and Nav2 replanning. the robot still completes 85 / 95
waypoints because the *physical* trajectory benefits from frequent
replans, but the estimate underneath is genuinely poor.

**better config for the campus campaign.** exp 60 numbers (median
fresh-anchor err 0.39 m, max 5.49 m on a similar route) are the
realistic reference. for chapter 5, exp 60 is the run to point at as
the system working well; exp 64 is the run to point at as "what
happens if you skip the sanity gate."



## 70 - IMU fidelity ablation (does the noise model matter?)

sources: `baseline/traj_gt.csv` 4 192 samples 337.2 m, `run_A_accel_noise/traj_gt.csv`
4 185 samples 337.1 m. baseline `metrics.json` says ATE max 0.74 m,
mean 0.49 m, scale 1.0011, GT path 321.3 m total round trip.

**what to read off the plot.** two GT lines along the road, basically
on top of each other. they share x ∈ [-95, +70.5], y ∈ [-6, +2]. the
slight difference is well below visible plot resolution. the legend
calls out the drift-max numbers per run.

**why.** the same route was driven twice with two different IMU models -
one with the simulator's default noise, one with synthetic Phidgets-1042
accelerometer noise + bias added. drift max went from 0.74 m to 0.69 m
(if anything, slightly *better*), well within run-to-run variance. the
sim IMU is therefore not the main lever on drift; route distance and
feature density are. this is the right ablation to cite when defending
why is your sim IMU clean enough?



## 73 - stock Nav2 baseline (no matcher, no detour ring)

source: `/root/isaac_tr_datasets/01_road/baseline_stock_nav2/traj_gt.csv`
1 553 samples, 162.7 m of GT path. teach reference 25 850 samples,
321.4 m of round trip. watchdog abort: `no WP REACHED after 840 s`.
robot stalled with red GT going only to x ≈ +14.6 - the teach goes to
+70.5 and back.

**what to read off the plot.** the dashed dark teach line traces a full
road round trip out and back, x ∈ [-92, +70.5]. the thick red GT line
of the actual stock-Nav2 run follows the same corridor for the
outbound half but stops cleanly partway along, around x = +14.6, with
y deviations up to 3.5 m off the road centre. the run never reaches
the apex.

**why.** stock Nav2 with the default behaviour tree (RPP controller,
spin / back-up / drive-on-heading / wait recoveries) gets the robot
into the tree-inflation zone of the teach occupancy map and then the
default recoveries cannot extract it. without our custom WP look-ahead
skip, detour ring, and no-skip-final-5 logic, the planner keeps trying
to thread an inflated corridor and eventually the watchdog aborts after
14 minutes of no waypoint progress. this is the ablation that says
yes, the custom local pieces are load-bearing.



## 75 - gap navigator without Nav2, teach waypoints as the only goal source

source: `tf_nav_gt.csv` 43 samples, **bag GT path 26.9 m**, x ∈ [65, 78],
y ∈ [-35, -14]. mean nav-vs-GT err 0.58 m, max 1.25 m. anchors published
**33 / 423 (7.8 %)**.

**important: this run is on route 09_se_ne, not the south route.** spawn
at (65, -35), heading 1.05 rad. the route is the short south-east →
north-east leg with cardboxes/dumpster/barrels as obstacles.

**what to read off the plot.** the orange nav-pose and red GT lines run
together from spawn at (65, -35) up toward (78, -14), about 27 metres,
then *stop*. the orange line tracks GT closely the whole way (sub-metre
error). there is no full-loop trajectory in this figure - the run
never got that far.

run timeline:
- WP 0 reached at d = 2.7 m (10 s)
- WP 1 reached at d = 2.9 m
- WP 2 needed one recovery (back-up + rotate), then reached (33 s after
  start) - that is around (78, -14)
- WP 3 needed three recoveries, never reached. robot is stuck near
  the cardboxes/dumpster cluster around (76-77, -14).

**why.** without Nav2 there is no global planner that can see the obstacle
group as a single thing to bypass. the gap navigator is reactive: it
finds the best gap in the *current* depth frame. when the teach WP is
on the far side of an obstacle group with no clean reactive gap, the
controller runs back-up + rotate recoveries and tries again, and again,
and again. this is the *symmetric* ablation to exp 73: 73 said "you
can not drop the local pieces, 75 says you can not drop the global
planner." together they justify the full stack.



## anchors in action - extra figure for the matcher chapter

source: `anchors_in_action.png`, rendered by
[`make_anchor_action_plot.py`](../../../scripts/analysis/make_anchor_action_plot.py)
from the same exp 64 data as the trajectory plot above.

**what it shows.** the south route map, with the GT and nav-pose lines from
exp 64 underneath, and *every single matcher attempt* drawn on top:

- **green dots + thin arrows** mark the 479 accepted corrections. each arrow
  points from the VIO pose at the moment of the match to where the matcher
  put the pose afterwards. the arrow length is the size of the pull -
  median 1.1 m, longest 5 m.
- **red x markers + dotted lines + red stars** are the 71 corrections the
  matcher's consistency check refused to publish. the x sits at the VIO
  pose at the moment of the attempt - that is, where the planner already
  thinks the robot is. the dotted line points out to a red star, which is
  where the proposed (rejected) anchor would have placed the pose. the
  rejected proposals all had shifts above 5 m (the matcher's hard limit);
  measured shifts: 5.05 m to 15.74 m, median 6.3 m, mean 7.5 m. accepted
  shifts, by comparison, top out at exactly 5.0 m.

  this is also the answer to "why are the red x markers sitting *on* the
  trajectory and not at the visible peaks of the orange line." the x
  shows where the pose was *before* the rejected attempt - i.e. on the
  planner's pose, which is on or close to the nav-pose line. the proposed
  wrong-cluster match (the star at the end of the dotted line) was
  rejected, so it never moved the pose; only the *would-have-been* peaks
  are out there. the visible peaks in the orange nav-pose line, in
  contrast, come from corrections that were *accepted* - those each had
  shift up to 5 m, but the relay logs every 5 s so two or three
  back-to-back near-limit corrections plus normal motion can stack into
  a 10-15 m apparent step in the line at log resolution.
- **gray squares** mark the 43 events where the matcher had *no* teach
  landmark within its 8 m search radius - usually because VIO drift had
  already pushed the search centre out of the teach corridor.
- the remaining 3 992 attempts (not drawn - too dense) tried matching but
  did not get enough PnP inliers. those are silent failures: matcher
  ran, no harm done.

**how to read it in writing.** point at the dense clusters of green arrows
along the corridor as evidence the matcher fires routinely on healthy
parts of the route. point at the red x markers, mostly along the
return half, as evidence that the sanity gate is doing real work.
point at any straight stretch with no dots at all - those are the
no-anchor windows where drift just grows linearly until the matcher
finds something to lock back onto.

**totals to cite.** 4 585 attempts → 479 accepted (≈10 %), 71 rejected
by gate (≈1.5 %), 43 had nothing to match (≈1 %), the rest tried and
failed quietly. accepted shifts: median 1.08 m, mean 1.47 m, max 5 m.



## how to use this in chapter 5

the canonical narrative arc is:

1. **02, 03** - global SLAM (mapping or atlas) cannot constrain lateral
   drift after obstacle bypass.
2. **27** - and worse, it can fail silently - the robot drove 25 m off
   the road into the trees while the stack believed it was on track.
3. **30** - first teach-and-repeat replaces trust SLAM with "trust
   dead reckoning + visual anchors." pipeline pose vs GT visibly
   diverges over 37 m.
4. **31, 35, 39** - the depth-only / gap-based / hybrid line. 31 stops
   at a sub-robot gap in the forest; 35 stops at the cone wall on the
   road; 39 never even gets to its A* mode. motivates moving to a
   real planner.
5. **41** - Nav2 with GT proves the planner works given perfect pose.
   41/43 waypoints reached with four solid barriers.
6. **49** - same Nav2 with raw VIO almost works on a clean loop. ATE
   0.534 m over 395 m, the orange and red lines visually overlap.
7. **55** - visual landmark matcher (PnP-RANSAC) is the key innovation;
   in this run drift goes 0.3 → 9.1 m over the route and three large
   wrong-cluster anchor jumps (10, 27, 19 m) are visible.
8. **56** - projection cap of 1 m on the next waypoint. nav line is
   smooth, no jumps, full 1 390 m loop closes within 8 m of spawn.
9. **58** - running landmark accumulator self-poisons. 197 jumps in
   the nav line, biggest 95 m, total nav path 5.6* the actual driven
   distance. caught and disabled.
10. **60** - precise finisher + STOP fix. cleanest south loop in the
    figure set, start and end markers sit on top of each other.
11. **62** - anchor sanity gate. visually similar to 60 but rejects
    confident-yet-wrong matches that the earlier runs were vulnerable to.
12. **64** - best-of-breed synthesis on paper, but a cautionary tale in
    practice: by skipping exp 62's sanity gate, the matcher publishes
    wrong-cluster matches that pull the pose by up to 5 m at a time.
    median fresh-anchor err 6.93 m vs exp 60's 0.39 m. recommend exp 60
    for deployed configuration, not exp 64.
13. **70** - IMU noise ablation: simulation noise model is not the
    load-bearing factor.
14. **73, 75** - strip-down ablations: 73 turns off the custom local
    pieces (lookahead, detour, finisher) and stalls 56 m short on
    the road; 75 turns off Nav2 entirely and stalls at WP 3 of route
    09_se_ne. both halves of the stack are required.
