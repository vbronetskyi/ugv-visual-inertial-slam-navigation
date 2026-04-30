# exp 18: VIO offline with zigzag IMU initialization - success

Make ORB-SLAM3 RGBD-Inertial SLAM produce a usable trajectory by fixing
the IMU initialization problem from exp 17.

## результат

- RGBD-I beats RGBD-only

| Method | ATE | Scale | Notes |
|---|---|---|---|
| RGBD-only baseline | 0.49 m | - | best previous result |
| RGBD-I exp 17 (no init, Variant B Tbc) | 22.0 m | 0.150 | 6.7x scale error |
| RGBD-I exp 17 (no init, identity Tbc) | 8.5 m | 0.078 | 12.8x scale error |
| **RGBD-I exp 18 (zigzag init, identity Tbc)** | **0.116 m** | **0.993** | **4.2x BETTER than RGBD-only** |

VIO trajectory vs GT: 1377 frames (1500 input, +-150 sec), tracks 86m of
