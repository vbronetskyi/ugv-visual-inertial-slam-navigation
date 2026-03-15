#!/usr/bin/env python3
import os
import sys
import math
import numpy as np

EXPERIENCE = '/usr/local/lib/python3.12/dist-packages/isaacsim/apps/isaacsim.exp.headless.minimal.kit'
HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
OUTPUT_USD = "/workspace/simulation/isaac/assets/husky_outdoor_scene.usd"

from isaacsim import SimulationApp
app = SimulationApp({"headless": True}, experience=EXPERIENCE)

import omni
import omni.kit.commands
from pxr import UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf, Vt, PhysicsSchemaTools

# enable extensions we need
manager = omni.kit.app.get_app().get_extension_manager()
# only enable what we need for scene building (not runtime sensor extensions)
# sensor/ros2 extensions have broken dep chains in pip install (omni.pip.cloud missing)
# they'll work at runtime when loading in full isaac sim
extensions = [
    "isaacsim.asset.importer.utils",
    "isaacsim.asset.importer.urdf",
    "isaacsim.asset.transformer",
    "isaacsim.asset.transformer.rules",
]
for ext in extensions:
    manager.set_extension_enabled_immediate(ext, True)

for _ in range(20):
    app.update()

# -- open husky usd --
print("opening husky usd...")
omni.usd.get_context().open_stage(HUSKY_USD)
for _ in range(20):
    app.update()

stage = omni.usd.get_context().get_stage()
root = stage.GetDefaultPrim()
robot_path = root.GetPath().pathString
print(f"  robot at: {robot_path}")

# remove root_joint - it's a FixedJoint that locks the robot to the world
root_joint_path = f"{robot_path}/Physics/root_joint"
rj = stage.GetPrimAtPath(root_joint_path)
if rj.IsValid():
    stage.RemovePrim(root_joint_path)
    # print(f">>> frame {i}/{n_frames}")
    print("  removed root_joint FixedJoint (was locking robot in place)")

# set robot spawn position - on the road, looking forward toward obstacles
base_link_path = f"{robot_path}/Geometry/base_link"
base_prim = stage.GetPrimAtPath(base_link_path)
if base_prim.IsValid():
    xf = UsdGeom.Xformable(base_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-20, 0, 0.3))  # drop from 0.3m, settles to ~0.14m
    print("  spawn: (-20, 0, 0.3) - on road, facing forward")


# 1. OUTDOOR ENVIRONMENT
print("\nbuilding outdoor environment...")

# -- physics scene (gravity, solver settings) --
physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
physics_scene.CreateGravityMagnitudeAttr(9.81)

# -- ground plane --
PhysicsSchemaTools.addGroundPlane(
    stage, "/World/GroundPlane", "Z", 200.0,
    Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.3, 0.45, 0.2)  # grass green
)
# friction for outdoor dirt/grass
ground_geom = stage.GetPrimAtPath("/World/GroundPlane/geom")
if ground_geom.IsValid():
    mat_api = UsdPhysics.MaterialAPI.Apply(ground_geom)
    mat_api.CreateStaticFrictionAttr(0.9)
    mat_api.CreateDynamicFrictionAttr(0.7)
    mat_api.CreateRestitutionAttr(0.1)

# -- dirt road (flat box along X axis) --
road = UsdGeom.Cube.Define(stage, "/World/Environment/Road")
road.CreateSizeAttr(1.0)
xform = UsdGeom.Xformable(road)
xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.005))
xform.AddScaleOp().Set(Gf.Vec3f(100, 3.0, 0.01))  # 100m long, 3m wide
road.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.45, 0.3)])  # dirt brown

# -- scatter obstacles as tree/rock proxies --
# random but deterministic placement for reproducibility
rng = np.random.RandomState(42)
obstacle_parent = UsdGeom.Xform.Define(stage, "/World/Environment/Obstacles")

for i in range(30):
    # place on both sides of the road, not on the road itself
    x = rng.uniform(-40, 40)
    y_side = rng.choice([-1, 1])
    y = y_side * rng.uniform(3, 25)  # at least 3m from road center
    height = rng.uniform(0.5, 3.0)
    width = rng.uniform(0.3, 1.5)

    obs_path = f"/World/Environment/Obstacles/obstacle_{i:02d}"
    obs = UsdGeom.Cube.Define(stage, obs_path)
    obs.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(obs)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, height / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(width, width, height))

    # green-ish for trees, grey for rocks
    if rng.random() > 0.3:
        color = Gf.Vec3f(0.15 + rng.uniform(0, 0.15), 0.35 + rng.uniform(0, 0.2), 0.1)
    else:
        color = Gf.Vec3f(0.5, 0.5, 0.5)
    obs.CreateDisplayColorAttr([color])

    # add collision
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(obs_path))

# -- some larger structures (village buildings) at the far end --
buildings_parent = UsdGeom.Xform.Define(stage, "/World/Environment/Buildings")
building_configs = [
    (30, 8, 4, 6, 3, Gf.Vec3f(0.8, 0.75, 0.65)),   # beige house
    (35, -10, 3, 5, 4, Gf.Vec3f(0.7, 0.3, 0.25)),   # red barn
    (25, 12, 5, 4, 2.5, Gf.Vec3f(0.9, 0.9, 0.85)),  # white cottage
]
for i, (bx, by, bw, bl, bh, color) in enumerate(building_configs):
    bld_path = f"/World/Environment/Buildings/building_{i:02d}"
    bld = UsdGeom.Cube.Define(stage, bld_path)
    bld.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(bld)
    xf.AddTranslateOp().Set(Gf.Vec3d(bx, by, bh / 2))
    xf.AddScaleOp().Set(Gf.Vec3f(bl, bw, bh))
    bld.CreateDisplayColorAttr([color])
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(bld_path))

# -- outdoor lighting --
sun = UsdLux.DistantLight.Define(stage, "/World/Environment/SunLight")
sun.CreateIntensityAttr(5000)
sun.CreateAngleAttr(0.53)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
sun_xf = UsdGeom.Xformable(sun)
sun_xf.AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))

# fill light from opposite side
fill = UsdLux.DistantLight.Define(stage, "/World/Environment/FillLight")
fill.CreateIntensityAttr(1500)
fill.CreateAngleAttr(1.0)
fill.CreateColorAttr(Gf.Vec3f(0.7, 0.8, 1.0))  # sky blue tint
fill_xf = UsdGeom.Xformable(fill)
fill_xf.AddRotateXYZOp().Set(Gf.Vec3f(-30, -160, 0))

# dome light for sky background - eliminates black sky in camera
dome = UsdLux.DomeLight.Define(stage, "/World/Environment/DomeLight")
dome.CreateIntensityAttr(1000)
dome.CreateColorAttr(Gf.Vec3f(0.53, 0.70, 0.92))  # clear sky blue
dome.CreateTextureFormatAttr("latlong")

print("  ground plane: 200m, grass material")
print("  road: 100m x 3m dirt path")
print("  obstacles: 30 tree/rock proxies")
print("  buildings: 3 village structures")
print("  lighting: sun + fill + dome sky")

for _ in range(10):
    app.update()


# 2. RGB-D CAMERA SENSOR
print("\nadding rgb-d camera sensor...")

# camera prim path - attach to the realsense link on the husky
camera_parent = f"{robot_path}/Geometry/base_link/top_plate_link/camera_realsense_bottom_screw_frame/camera_realsense_link"

# create a UsdGeom.Camera prim as child of the realsense link
cam_path = f"{camera_parent}/d435i_camera"
cam = UsdGeom.Camera.Define(stage, cam_path)
# d435i specs: 87° x 58° fov, 1280x720 default
cam.CreateFocalLengthAttr(1.93)  # ~87 deg hfov at 3.68mm sensor
cam.CreateHorizontalApertureAttr(3.68)
cam.CreateVerticalApertureAttr(2.45)  # ~58 deg vfov
cam.CreateClippingRangeAttr(Gf.Vec2f(0.105, 10.0))  # d435i: 0.105m - 10m
# USD camera: looks -Z, up +Y. we want: look +X (forward), up +Z (world up)
# rotateXYZ applies Rz * Ry * Rx. with (90, 0, -90):
#   view: Rx(90)(0,0,-1)=(0,1,0) -> Rz(-90)(0,1,0)=(1,0,0) = +X forward
#   up:   Rx(90)(0,1,0)=(0,0,1)  -> Rz(-90)(0,0,1)=(0,0,1) = +Z up
cam_xf = UsdGeom.Xformable(cam)
cam_xf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))

# print(f"DEBUG: ran {len(ran)} waypoints")
print(f"  camera prim: {cam_path}")
print("  fov: 87° x 58° (d435i specs)")
print("  range: 0.105m - 10m")

for _ in range(10):
    app.update()


# 3. IMU SENSOR
print("\nadding imu sensor...")

# imu goes on the imu_link
imu_parent = f"{robot_path}/Geometry/base_link/imu_link"

# create imu sensor prim using raw USD attributes (matching IsaacImuSensor schema)
# can't use the kit command because omni.replicator.core dep chain is broken in pip install
imu_path = f"{imu_parent}/imu_sensor"
imu_prim = stage.DefinePrim(imu_path, "Xform")

# set the IsaacImuSensor schema attributes directly
imu_prim.CreateAttribute("sensorPeriod", Sdf.ValueTypeNames.Float).Set(1.0 / 200.0)  # 200 Hz
imu_prim.CreateAttribute("linearAccelerationFilterWidth", Sdf.ValueTypeNames.Int).Set(10)
imu_prim.CreateAttribute("angularVelocityFilterWidth", Sdf.ValueTypeNames.Int).Set(10)
imu_prim.CreateAttribute("orientationFilterWidth", Sdf.ValueTypeNames.Int).Set(10)
# tag it so the sensor extension picks it up at runtime
imu_prim.CreateAttribute("isaac:sensor:type", Sdf.ValueTypeNames.String).Set("IsaacImuSensor")
print(f"  imu prim: {imu_path}")
print("  rate: 200 Hz (matching Phidgets spatial)")
print("  filter size: 10 samples")

for _ in range(10):
    app.update()


# 4. SAVE SCENE
print(f"\nsaving scene to: {OUTPUT_USD}")
stage.Export(OUTPUT_USD)


# 5. VERIFY SCENE CONTENTS
print("\n=== scene verification ===")

# count prims by type
type_counts = {}
for prim in stage.Traverse():
    t = prim.GetTypeName()
    type_counts[t] = type_counts.get(t, 0) + 1

for t in sorted(type_counts.keys()):
    if type_counts[t] > 0:
        print(f"  {t}: {type_counts[t]}")

# check key prims exist
key_prims = [
    (robot_path, "robot root"),
    (f"{robot_path}/Physics/front_left_wheel", "front left wheel joint"),
    (f"{robot_path}/Physics/front_right_wheel", "front right wheel joint"),
    (camera_parent, "camera link"),
    (cam_path, "d435i camera"),
    (imu_parent, "imu link"),
    ("/World/PhysicsScene", "physics scene"),
    ("/World/GroundPlane", "ground plane"),
    ("/World/Environment/Road", "road"),
    ("/World/Environment/SunLight", "sun light"),
]

print("\nkey prims:")
for path, desc in key_prims:
    prim = stage.GetPrimAtPath(path)
    status = "ok" if prim.IsValid() else "MISSING"
    print(f"  [{status}] {desc}: {path}")

print(f"\n=== outdoor scene ready ===")
print(f"output: {OUTPUT_USD}")

app.close()
print("done")
