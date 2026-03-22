#!/usr/bin/env python3
"""
IMU calibration test: dump raw sensor readings at 4 motion states.

Determines the actual IMU sensor frame mapping (which axis is up/forward/lateral)
by observing where gravity, yaw rate, and forward acceleration appear in raw output.

usage:
  /opt/isaac-sim-6.0.0/python.sh imu_cal_test.py
"""
import os
import sys
import math
import time
import numpy as np

ISAAC_SIM_PATH = os.environ.get("ISAAC_SIM_PATH", "/opt/isaac-sim-6.0.0")
ROS2_LIB = os.path.join(ISAAC_SIM_PATH, "exts/isaacsim.ros2.core/jazzy/lib")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = ROS2_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "samples_per_pixel_per_frame": 8,
                     "width": 320, "height": 240})

import omni, omni.kit.app, omni.kit.commands
import carb
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, UsdShade

manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.sensors.physics.nodes"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(20):
    app.update()

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

print("adding Husky...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(200):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"
IMU_LINK = f"{BASE_LINK}/imu_link"

# physics 200Hz TGS
_phys = stage.GetPrimAtPath("/World/PhysicsScene")
if _phys.IsValid():
    PhysxSchema.PhysxSceneAPI(_phys).GetTimeStepsPerSecondAttr().Set(200)
    PhysxSchema.PhysxSceneAPI(_phys).CreateSolverTypeAttr().Set("TGS")

# spawn position
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_translate = husky_xf.AddTranslateOp()
_rotate = husky_xf.AddRotateXYZOp()
SPAWN_X, SPAWN_Y, SPAWN_Z = -95.0, -6.0, 0.5
_translate.Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, SPAWN_Z))
_rotate.Set(Gf.Vec3f(0, 0, 0))

# wheel drives
wheel_r = 0.165
track = 0.555
_wheel_vel_attrs = []
for wn in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
    drive = UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"/World/Husky/Physics/{wn}"), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())

# IMU sensor
IMU_SENSOR_PATH = IMU_LINK + "/imu_sensor"
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor",
    parent=IMU_LINK,
    sensor_period=1.0 / 200.0,
    linear_acceleration_filter_size=60,
    angular_velocity_filter_size=30,
    orientation_filter_size=30,
)

# start physics
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

from isaacsim.sensors.physics import _sensor as _imu_mod
_imu_interface = _imu_mod.acquire_imu_sensor_interface()

_base_prim = stage.GetPrimAtPath(BASE_LINK)


def get_robot_pose():
    xf = UsdGeom.XformCache()
    tf = xf.GetLocalToWorldTransform(_base_prim)
    pos = tf.ExtractTranslation()
    rot = tf.ExtractRotationMatrix()
    yaw = math.atan2(rot[0][1], rot[0][0])
    return float(pos[0]), float(pos[1]), float(pos[2]), yaw


def measure(label, n=200, drive_left=0.0, drive_right=0.0, settle=50):
    """Drive wheels at given velocities and collect IMU samples."""
    for i, wa in enumerate(_wheel_vel_attrs):
        v = drive_left if i % 2 == 0 else drive_right
        wa.Set(math.degrees(v))

    # warmup
    for _ in range(settle):
        app.update()

    accs = []
    gyros = []
    poses = []
    for _ in range(n):
        app.update()
        r = _imu_interface.get_sensor_reading(IMU_SENSOR_PATH, read_gravity=True)
        if r.is_valid:
            accs.append([r.lin_acc_x, r.lin_acc_y, r.lin_acc_z])
            gyros.append([r.ang_vel_x, r.ang_vel_y, r.ang_vel_z])
            poses.append(get_robot_pose())

    accs = np.array(accs)
    gyros = np.array(gyros)
    poses = np.array(poses)

    print(f"\n=== {label} ===")
    print(f"  drive_left={drive_left:.2f} drive_right={drive_right:.2f}")
    print(f"  RAW IMU lin_acc (mean): "
          f"x={accs[:,0].mean():+.4f} y={accs[:,1].mean():+.4f} z={accs[:,2].mean():+.4f}")
    print(f"  RAW IMU ang_vel (mean): "
          f"x={gyros[:,0].mean():+.4f} y={gyros[:,1].mean():+.4f} z={gyros[:,2].mean():+.4f}")
    print(f"  RAW IMU lin_acc (std):  "
          f"x={accs[:,0].std():.4f} y={accs[:,1].std():.4f} z={accs[:,2].std():.4f}")
    print(f"  Robot pos: ({poses[:,0].mean():.2f}, {poses[:,1].mean():.2f}, {poses[:,2].mean():.2f})")
    print(f"  Robot yaw: mean={poses[:,3].mean():+.4f} std={poses[:,3].std():.4f}")
    print(f"  GT yaw rate: {(poses[-1,3] - poses[0,3]) / (n / 200.0):+.4f} rad/s")
    print(f"  GT linear motion: dx={poses[-1,0]-poses[0,0]:+.3f}m dy={poses[-1,1]-poses[0,1]:+.3f}m")

    return accs, gyros, poses


# test 1: stationary
# Determines which axis gravity reaction (+9.81) appears on
acc_s, gyro_s, _ = measure("STATIONARY", n=200, drive_left=0, drive_right=0, settle=200)

# test 2: yaw left (rotate CCW = positive yaw)
# left wheels back, right wheels forward
v_yaw = 1.0 / wheel_r  # 1 m/s wheel speed
acc_yl, gyro_yl, poses_yl = measure(
    "YAW LEFT (CCW, +yaw in world)", n=200,
    drive_left=-v_yaw, drive_right=v_yaw, settle=80)

# test 3: yaw right (CW = negative yaw)
acc_yr, gyro_yr, _ = measure(
    "YAW RIGHT (CW, -yaw in world)", n=200,
    drive_left=v_yaw, drive_right=-v_yaw, settle=80)

# Reset to spawn (stop wheels and re-spawn)
for wa in _wheel_vel_attrs:
    wa.Set(0)
for _ in range(100):
    app.update()
_translate.Set(Gf.Vec3d(SPAWN_X, SPAWN_Y, SPAWN_Z))
_rotate.Set(Gf.Vec3f(0, 0, 0))
for _ in range(100):
    app.update()

# test 4: forward acceleration
v_fwd = 1.0 / wheel_r
acc_f, gyro_f, _ = measure(
    "FORWARD DRIVE (+x in world)", n=200,
    drive_left=v_fwd, drive_right=v_fwd, settle=50)  # short settle to catch acceleration

# analysis
print("\n\n========== AXIS MAPPING ANALYSIS ==========")

# Gravity: which axis has +9.81 stationary?
g_axis = np.argmax(np.abs(acc_s.mean(axis=0)))
g_sign = np.sign(acc_s.mean(axis=0)[g_axis])
g_axis_name = ['x', 'y', 'z'][g_axis]
print(f"\nGravity (+9.81) is on RAW sensor {g_sign:+.0f} * {g_axis_name}")
print(f"  -> sensor {g_axis_name}-axis points {'UP' if g_sign > 0 else 'DOWN'} in world")

# Yaw: during left rotation, which gyro axis is positive?
gyro_yl_mean = gyro_yl.mean(axis=0)
gyro_yr_mean = gyro_yr.mean(axis=0)
y_diff = gyro_yl_mean - gyro_yr_mean  # left - right
y_axis = np.argmax(np.abs(y_diff))
y_sign = np.sign(y_diff[y_axis])
y_axis_name = ['x', 'y', 'z'][y_axis]
print(f"\nYaw rate (positive = CCW left turn) is on RAW sensor {y_sign:+.0f} * {y_axis_name}")
print(f"  Left-Right gyro diff: {y_diff}")
print(f"  -> sensor {y_axis_name}-axis is {'+UP (yaw axis)' if y_sign > 0 else '-UP (yaw axis flipped)'}")

# Forward: during forward drive, which acc axis has nonzero mean?
# (subtracting gravity component)
acc_f_mean = acc_f.mean(axis=0)
acc_f_excl_gravity = acc_f_mean - acc_s.mean(axis=0)
f_axis = np.argmax(np.abs(acc_f_excl_gravity))
f_sign = np.sign(acc_f_excl_gravity[f_axis])
f_axis_name = ['x', 'y', 'z'][f_axis]
print(f"\nForward accel (driving forward) is on RAW sensor {f_sign:+.0f} * {f_axis_name}")
print(f"  acc diff (forward - stationary): {acc_f_excl_gravity}")

# Summary
print(f"\n========== SUMMARY ==========")
print(f"sensor {g_axis_name} axis ({'+'if g_sign>0 else '-'}) = UP in world")
print(f"sensor {y_axis_name} axis ({'+'if y_sign>0 else '-'}) = YAW axis (along world Z)")
print(f"sensor {f_axis_name} axis ({'+'if f_sign>0 else '-'}) = FORWARD direction")

# Determine FLU mapping
print(f"\n========== FLU CONVERSION ==========")
print(f"To convert from sensor frame to FLU body frame:")
print(f"  FLU X (forward) = {'+' if f_sign>0 else '-'}sensor.{f_axis_name}")
print(f"  FLU Z (up)      = {'+' if g_sign>0 else '-'}sensor.{g_axis_name}")
remaining = [a for a in 'xyz' if a not in (f_axis_name, g_axis_name)]
if remaining:
    l_axis_name = remaining[0]
    print(f"  FLU Y (left)    = ?sensor.{l_axis_name}  (need to check sign)")
    # Left axis: cross product of up and forward = left
    # Or: yaw axis x forward = left for right-hand frame

# Print code template
print(f"\ndef _imu_to_flu(sx, sy, sz):")
print(f"    flu_x = {'+sx' if f_axis==0 and f_sign>0 else '-sx' if f_axis==0 else '+sy' if f_axis==1 and f_sign>0 else '-sy' if f_axis==1 else '+sz' if f_sign>0 else '-sz'}")
print(f"    flu_z = {'+sx' if g_axis==0 and g_sign>0 else '-sx' if g_axis==0 else '+sy' if g_axis==1 and g_sign>0 else '-sy' if g_axis==1 else '+sz' if g_sign>0 else '-sz'}")
print(f"    flu_y = ?  # check sign with cross product or another test")

timeline.stop()
app.close()
