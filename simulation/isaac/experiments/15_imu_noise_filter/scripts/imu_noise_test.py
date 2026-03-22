#!/usr/bin/env python3
"""
IMU noise test for Husky in Isaac Sim.

Measures PhysX IMU noise stationary and during motion.
Optionally applies compliant contacts to wheel materials.

usage:
  /opt/isaac-sim-6.0.0/python.sh imu_noise_test.py [--compliant]
"""
import os
import sys
import math
import time
import argparse
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--compliant", action="store_true",
                    help="apply compliant contacts to wheels")
parser.add_argument("--stiffness", type=float, default=1e6)
parser.add_argument("--damping", type=float, default=1e4)
parser.add_argument("--synthetic", action="store_true",
                    help="compute synthetic IMU from GT pose (Phidgets noise)")
parser.add_argument("--filter", type=int, default=20)
args, _ = parser.parse_known_args()

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

settings = carb.settings.get_settings()
settings.set("/persistent/omnigraph/updateToUsd", True)

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

# wheel friction material
_wf = UsdShade.Material.Define(stage, "/World/WheelFriction")
_wfp = UsdPhysics.MaterialAPI.Apply(_wf.GetPrim())
_wfp.CreateStaticFrictionAttr(1.0)
_wfp.CreateDynamicFrictionAttr(0.8)
_wfp.CreateRestitutionAttr(0.0)

# Apply compliant contacts to wheel material if requested
if args.compliant:
    print(f"\n=== APPLYING COMPLIANT CONTACTS ===")
    print(f"  stiffness={args.stiffness}, damping={args.damping}")
    physx_mat = PhysxSchema.PhysxMaterialAPI.Apply(_wf.GetPrim())
    physx_mat.CreateCompliantContactStiffnessAttr().Set(args.stiffness)
    physx_mat.CreateCompliantContactDampingAttr().Set(args.damping)
    print(f"  applied to /World/WheelFriction")

# Walk stage to find wheel collision prims
print("\nSearching for wheel collisions...")
wheel_cols = []
for prim in stage.Traverse():
    pn = str(prim.GetPath())
    if "wheel" in pn.lower() and "collision" in pn.lower():
        print(f"  found: {pn}  type={prim.GetTypeName()}")
        wheel_cols.append(prim)

for col in wheel_cols:
    if col.IsValid():
        # The collision is "guide" purpose - change to default so PhysX uses it
        UsdGeom.Imageable(col).GetPurposeAttr().Set("default")
        UsdShade.MaterialBindingAPI.Apply(col).Bind(_wf, materialPurpose="physics")
        print(f"  bound material to {col.GetPath()}")
        if args.compliant:
            physx_col_mat = PhysxSchema.PhysxMaterialAPI.Apply(_wf.GetPrim())
            physx_col_mat.CreateCompliantContactStiffnessAttr().Set(args.stiffness)
            physx_col_mat.CreateCompliantContactDampingAttr().Set(args.damping)

# spawn position
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_translate = husky_xf.AddTranslateOp()
_rotate = husky_xf.AddRotateXYZOp()
_translate.Set(Gf.Vec3d(-95.0, -6.0, 0.5))
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
    linear_acceleration_filter_size=args.filter,
    angular_velocity_filter_size=args.filter // 2,
    orientation_filter_size=args.filter // 2,
)

# start physics
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

from isaacsim.sensors.physics import _sensor as _imu_mod
_imu_interface = _imu_mod.acquire_imu_sensor_interface()

# synthetic imu state (phidgets spatial 1042 noise model)
GYRO_NOISE_STD = 0.005
ACCEL_NOISE_STD = 0.02
GYRO_BIAS = np.array([0.001, -0.0008, 0.0012])
ACCEL_BIAS = np.array([0.005, -0.003, 0.004])

_syn_state = {
    'prev_pos': None, 'prev_vel': np.zeros(3),
    'prev_quat': None, 'prev_time': None,
}

def _quat_to_yaw(q):
    """quat (qx,qy,qz,qw) -> yaw."""
    qx, qy, qz, qw = q
    return math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

def _quat_to_R(q):
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])

def synthetic_imu(vel_world, ang_vel_world, quat, t):
    """Compute IMU from PhysX rigid body velocities (no position differentiation).

    Only differentiates velocity once -> acc, much smoother than 2x diff of pos.
    Returns (lin_acc_body, ang_vel_body) with realistic noise.
    """
    vel_world = np.asarray(vel_world, dtype=float)
    ang_vel_world = np.asarray(ang_vel_world, dtype=float)
    quat = np.asarray(quat, dtype=float)

    if _syn_state['prev_time'] is None:
        _syn_state['prev_time'] = t
        _syn_state['prev_vel'] = vel_world
        return None

    dt = t - _syn_state['prev_time']
    if dt < 1e-6:
        return None

    # Linear acceleration from velocity diff (single differentiation)
    lin_acc_world = (vel_world - _syn_state['prev_vel']) / dt
    # Gravity reaction (+9.81 z because IMU measures gravity reaction)
    lin_acc_world[2] += 9.81

    # Rotate to body frame
    R = _quat_to_R(quat)
    lin_acc_body = R.T @ lin_acc_world
    ang_vel_body = R.T @ ang_vel_world

    # Realistic noise + bias
    lin_acc_body += np.random.normal(0, ACCEL_NOISE_STD, 3) + ACCEL_BIAS
    ang_vel_body += np.random.normal(0, GYRO_NOISE_STD, 3) + GYRO_BIAS

    _syn_state['prev_time'] = t
    _syn_state['prev_vel'] = vel_world

    return lin_acc_body, ang_vel_body

def _flu(ax, ay, az):
    return az, -ay, ax

_base_prim_for_pose = stage.GetPrimAtPath(BASE_LINK)

# Get rigid body API for direct velocity reading (avoids differentiation jitter)
from omni.physx import get_physx_simulation_interface
from pxr import PhysxSchema as _PSchema
_rb_api = UsdPhysics.RigidBodyAPI.Get(stage, BASE_LINK)
_physx_rb = _PSchema.PhysxRigidBodyAPI.Get(stage, BASE_LINK)

def get_rigid_body_velocities():
    """Read velocities directly from PhysX rigid body API."""
    if _rb_api:
        v = _rb_api.GetVelocityAttr().Get()
        w = _rb_api.GetAngularVelocityAttr().Get()
        if v is not None and w is not None:
            return np.array([v[0], v[1], v[2]]), np.array([w[0], w[1], w[2]])
    return None, None

def measure(label, n=400, drive_speed=0.0):
    """Collect n IMU readings, optionally driving."""
    if drive_speed > 0:
        v_left = drive_speed / wheel_r
        v_right = drive_speed / wheel_r
        for i, wa in enumerate(_wheel_vel_attrs):
            wa.Set(math.degrees(v_left if i % 2 == 0 else v_right))
    else:
        for wa in _wheel_vel_attrs:
            wa.Set(0.0)

    # warmup
    for _ in range(50):
        app.update()

    # reset synthetic IMU state to skip warmup transients
    _syn_state['prev_time'] = None

    accs = []
    gyros = []
    sim_t = 0.0
    physics_dt = 1.0 / 200.0
    for _ in range(n):
        app.update()
        sim_t += physics_dt
        if args.synthetic:
            # Velocity from PhysX rigid body (avoids position differentiation)
            vel_w, ang_w = get_rigid_body_velocities()
            xf = UsdGeom.XformCache()
            tf = xf.GetLocalToWorldTransform(_base_prim_for_pose)
            rot_q = tf.ExtractRotationQuat()
            quat = (rot_q.GetImaginary()[0], rot_q.GetImaginary()[1],
                    rot_q.GetImaginary()[2], rot_q.GetReal())
            if vel_w is not None:
                res = synthetic_imu(vel_w, ang_w, quat, sim_t)
                if res is not None:
                    lin_acc, ang_vel = res
                    accs.append(lin_acc.tolist())
                    gyros.append(ang_vel.tolist())
        else:
            r = _imu_interface.get_sensor_reading(IMU_SENSOR_PATH, read_gravity=True)
            if r.is_valid:
                ax, ay, az = _flu(r.lin_acc_x, r.lin_acc_y, r.lin_acc_z)
                gx, gy, gz = _flu(r.ang_vel_x, r.ang_vel_y, r.ang_vel_z)
                accs.append([ax, ay, az])
                gyros.append([gx, gy, gz])

    accs = np.array(accs)
    gyros = np.array(gyros)
    print(f"\n--- {label} (n={len(accs)}) ---")
    print(f"  acc_mean: x={accs[:,0].mean():+.4f} y={accs[:,1].mean():+.4f} z={accs[:,2].mean():+.4f}")
    print(f"  acc_std:  x={accs[:,0].std():.4f} y={accs[:,1].std():.4f} z={accs[:,2].std():.4f}")
    print(f"  gyro_mean: x={gyros[:,0].mean():+.4f} y={gyros[:,1].mean():+.4f} z={gyros[:,2].mean():+.4f}")
    print(f"  gyro_std:  x={gyros[:,0].std():.4f} y={gyros[:,1].std():.4f} z={gyros[:,2].std():.4f}")
    return accs, gyros

print(f"\n========== IMU NOISE TEST ==========")
print(f"compliant={args.compliant} synthetic={args.synthetic}")

# 1. Stationary
acc_s, gyro_s = measure("STATIONARY", n=400, drive_speed=0.0)

# 2. Moving forward 0.5 m/s
acc_m, gyro_m = measure("MOVING 0.5 m/s", n=400, drive_speed=0.5)

# 3. Stop
for wa in _wheel_vel_attrs:
    wa.Set(0.0)

print(f"\n========== SUMMARY ==========")
print(f"compliant={args.compliant}")
print(f"STATIONARY acc_std mean: {acc_s.std(axis=0).mean():.4f} m/s²")
print(f"STATIONARY gyro_std mean: {gyro_s.std(axis=0).mean():.4f} rad/s")
print(f"MOVING     acc_std mean: {acc_m.std(axis=0).mean():.4f} m/s²")
print(f"MOVING     gyro_std mean: {gyro_m.std(axis=0).mean():.4f} rad/s")
print(f"NOISE INCREASE (move/stat): "
      f"acc {acc_m.std(axis=0).mean()/max(acc_s.std(axis=0).mean(), 1e-6):.1f}x, "
      f"gyro {gyro_m.std(axis=0).mean()/max(gyro_s.std(axis=0).mean(), 1e-6):.1f}x")

timeline.stop()
app.close()
