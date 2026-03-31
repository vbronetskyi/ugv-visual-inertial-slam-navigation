# Exp 51 v7_physx_velocity - synth IMU from PhysX velocity API

## Hypothesis

v5 diagnostic showed synth IMU ax std 3.5 m/s² (measured) vs yaml 0.02 spec
- 175× too noisy because of position-double-differentiation. v6 worked
around this by bumping yaml `NoiseAcc: 1.5` so ORB-SLAM3 weights IMU
appropriately. A cleaner fix: use Isaac's rigid-body velocity API
(`_husky_rigid.get_linear_velocity()`) and single-differentiate to get
acceleration, then yaml can stay at the real Phidgets spec (0.02 / 0.005).

## Implementation iterations

### v7.1 - all PhysX API
Both `get_linear_velocity()` (for accel via single diff) and
`get_angular_velocity()` (for gyro direct). LPF maxlen 5.

| axis | v5 (pos-diff) | v7.1 |
|---|---|---|
| ax std | 3.90 | 1.03 |
| ay std | 1.26 | 1.30 |
| az std | 1.29 | 0.85 |
| gx std | **0.005** | **0.067** <- regressed |
| gy std | 0.005 | 0.073 |
| gz std | 0.052 | 0.034 |

Accel improved 4×. But **gyro regressed 13×** because PhysX angular
velocity has contact-solver jitter just like linear velocity - the clean
gyro in v3-v6 came from quaternion-diff.

### v7.2 - hybrid (PhysX vel for accel, quat-diff for gyro), LPF maxlen 20

| axis | v5 | v7.2 | Phidgets spec |
|---|---|---|---|
| ax std | 3.90 | **0.24** | 0.02 |
| ay std | 1.26 | 0.27 | 0.02 |
| az std | 1.29 | 0.20 | 0.02 |
| gx std | 0.005 | **0.005** x | 0.005 |
| gy std | 0.005 | 0.005 x | 0.005 |
| gz std | 0.052 | 0.066 | 0.005 |

**Accel 16× cleaner than v5**, gyro gx/gy match spec exactly.
gz still noisy (GT-derived yaw has wheel-slip contribution).

## The fatal flaw - 7 % path deficit

Test: integrate body-frame (ax, ay) through GT yaw to world frame, then
double-integrate to position. Compare integrated path to GT path.

| implementation | integrated path | GT path | ratio |
|---|---|---|---|
| v5 (pos-diff, LPF 11) | 82.2 m | 80.1 m | **1.026** x |
| v7.1 (vel-diff, LPF 5) | 18.1 m | 33.1 m | 0.547 ✗ |
| v7.2 (vel-diff, LPF 20) | 80.9 m | 94.9 m | 0.853 ✗ |

Extra LPF 1–61 on v7.2 changes ratio only 0.93 -> 0.89 - so LPF isn't
dominant. The issue is that **PhysX's `get_linear_velocity()` reports
approximately 7–15 % lower speed than the actual rate of position change**.
Presumably a contact-solver damping effect on the numerical wheel/ground
model.

## Consequence - worse VIO

Running VIO against v7.2 IMU:

| range | v6 err (mean/max) | v7.2 err (mean/max) |
|---|---|---|
| 0–50 m | 0.46 / 0.93 | 0.53 / 0.98 |
| 50–100 m | **1.35 / 1.94** | **2.82 / 9.54** <- worse |

v7.2 drifts faster than v6, because BA integrates 93 % of true distance
and concludes the robot is moving 7 % slower than it actually is -> the
integrated position lags GT.

## verdict

**v7 concept (clean IMU at source) is fundamentally right but PhysX's
velocity API has its own systematic bias** - querying `get_linear_velocity()`
doesn't give us the ground truth we needed. Double-differentiating GT
*position* (v5 approach) is noisier per-sample but correct in mean -
that's why v5's ratio is 1.026 while v7's is 0.85-0.93.

v6 (noisy IMU + yaml noise values matched to reality) remains the best
working answer: clean enough in the mean (ratio 1.026), ORB-SLAM3 weights
it correctly through yaml.

## Files

- `scripts/run_husky_forest_v7.py` - frozen snapshot with hybrid IMU
- `config/vio_th160.yaml` - Phidgets spec (0.02/0.005) - assumes clean IMU
- `logs/` - partial v7.1 and v7.2 runs

## next step

Return to v6 (proven working to 110 m at 0.65 m mean err), optionally add
RollingScaleCorrector on top if we want to push beyond 150 m.
