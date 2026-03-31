# SNAPSHOT: key modifications for Exp 48 LIVE VIO roundtrip
# This is a REFERENCE copy of the relevant sections from run_husky_forest.py
# Actual script: /workspace/simulation/isaac/scripts/run_husky_forest.py

# === 1. Synthetic IMU generation (PhysX unusable for VIO) ===
# PhysX IMU has contact-solver oscillations (accel std 0.37 m/s^2).
# Generate IMU from GT pose with Phidgets 1042 noise profile:
#   gyro_std=0.005 rad/s, accel_std=0.02 m/s^2
# World velocity from position diff -> LPF -> body accel in body frame
# Flag: --synthetic-imu

# === 2. IMU write format ===
# /tmp/isaac_imu.txt (append mode, 200 Hz):
#   timestamp gx gy gz ax ay az qx qy qz qw
# rgbd_inertial_live reads this file for IMU preintegration

# === 3. Pure pursuit with lookahead ===
# Pick WP ~2m ahead on path (skip dense 0.5m WPs for smooth driving):
#   LOOKAHEAD = 2.0
#   while dist(robot, wp[idx]) < LOOKAHEAD: idx++
# Arrival threshold: 1.0m

# === 4. Pure pursuit speed tuning ===
#   max_speed = 0.25  # cmd (× 3.4 Husky scaling = 0.76 m/s actual)
#   if |err| > 0.7:  lin=0.12, ang=err*1.5 (sharp turn)
#   elif |err| > 0.2: lin=0.20, ang=err*1.2 (medium)
#   else: lin=max_speed, ang=err*1.0 (cruise)

# === 5. Critical speed: 0.76 m/s ===
# At >1 m/s ORB feature matching degrades (>5% scene displacement/frame)
# 0.76 m/s gives 0.08m frame shift -> 0% track failures (Exp 46 validated)

# === 6. Route: 797 WPs (798 with turnaround loop) ===
# Densified from south_anchors_fixed.json (99 outbound WPs, 1.2m clearance)
# Interpolated to 0.5m spacing: 397 outbound + 17 natural turnaround loop + 386 return
# Critical: obstacle data includes USD scene rocks/RockCol (not just gazebo_models)

# === Key Isaac settings (at top of script) ===
# settings.set("/app/runLoops/main/rateLimitFrequency", 200)  # 200Hz physics for IMU
# SCENE_USD = "/opt/husky_forest_scene.usd"
# HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
