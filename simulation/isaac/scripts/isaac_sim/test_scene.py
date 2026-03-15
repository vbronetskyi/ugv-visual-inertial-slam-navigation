#!/usr/bin/env python3
"""
validate the husky outdoor scene loads correctly and physics works
tests what we can without the full isaac sim runtime (sensor extensions)
"""
import sys

EXPERIENCE = "/usr/local/lib/python3.12/dist-packages/isaacsim/apps/isaacsim.exp.headless.minimal.kit"
SCENE_USD = "/workspace/simulation/isaac/assets/husky_outdoor_scene.usd"

from isaacsim import SimulationApp
app = SimulationApp({'headless': True}, experience=EXPERIENCE)

import omni
from pxr import UsdPhysics, UsdGeom, Gf

# load scene
print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(20):
    app.update()

stage = omni.usd.get_context().get_stage()
errors = []
passes = []


def check(name, condition):
    if condition:
        passes.append(name)
        print(f"  [PASS] {name}")
    else:
        errors.append(name)
        # print("DEBUG: isaac sim step")
        print(f"  [FAIL] {name}")


# -- test 1: scene loads --
check("scene loads", stage is not None)

# -- test 2: robot exists --
robot = stage.GetPrimAtPath("/husky")
check("robot prim exists", robot.IsValid())

# -- test 3: all 4 wheel joints --
for wheel in ["front_left", "front_right", "rear_left", "rear_right"]:
    path = f"/husky/Physics/{wheel}_wheel"
    prim = stage.GetPrimAtPath(path)
    check(f"{wheel} wheel joint", prim.IsValid() and prim.GetTypeName() == "PhysicsRevoluteJoint")

# -- test 4: camera prim --
cam_path = "/husky/Geometry/base_link/top_plate_link/camera_realsense_bottom_screw_frame/camera_realsense_link/d435i_camera"
cam = stage.GetPrimAtPath(cam_path)
check("d435i camera prim", cam.IsValid() and cam.GetTypeName() == "Camera")

# check camera properties
if cam.IsValid():
    focal = cam.GetAttribute("focalLength").Get()
    clip = cam.GetAttribute("clippingRange").Get()
    check("camera focal length set", focal is not None and focal > 0)
    check("camera clip range (d435i: 0.105-10m)", clip is not None and abs(clip[0] - 0.105) < 0.01)

# -- test 5: imu prim --
imu_path = "/husky/Geometry/base_link/imu_link/imu_sensor"
imu = stage.GetPrimAtPath(imu_path)
check("imu prim exists", imu.IsValid())

if imu.IsValid():
    period = imu.GetAttribute("sensorPeriod").Get()
    check("imu period ~200Hz", period is not None and abs(period - 1.0/200.0) < 0.001)
    filt = imu.GetAttribute("linearAccelerationFilterWidth").Get()
    check("imu filter width = 10", filt == 10)

# -- test 6: physics scene --
phys = stage.GetPrimAtPath("/World/PhysicsScene")
check("physics scene exists", phys.IsValid())

if phys.IsValid():
    scene_api = UsdPhysics.Scene(phys)
    gravity_dir = scene_api.GetGravityDirectionAttr().Get()
    gravity_mag = scene_api.GetGravityMagnitudeAttr().Get()
    check("gravity direction is -Z", gravity_dir == Gf.Vec3f(0, 0, -1))
    check("gravity magnitude = 9.81", abs(gravity_mag - 9.81) < 0.01)

# -- test 7: environment --
check("ground plane", stage.GetPrimAtPath("/World/GroundPlane").IsValid())
check("road", stage.GetPrimAtPath("/World/Environment/Road").IsValid())
check("sun light", stage.GetPrimAtPath("/World/Environment/SunLight").IsValid())
check("fill light", stage.GetPrimAtPath("/World/Environment/FillLight").IsValid())
check("dome light (sky)", stage.GetPrimAtPath("/World/Environment/DomeLight").IsValid())

# count obstacles
obstacle_count = 0
for prim in stage.Traverse():
    if "obstacle_" in prim.GetName():
        obstacle_count += 1
check(f"obstacles ({obstacle_count} found)", obstacle_count == 30)

# count buildings
building_count = 0
for prim in stage.Traverse():
    if "building_" in prim.GetName():
        building_count += 1
check(f"buildings ({building_count} found)", building_count == 3)

# -- test 8: run 100 physics steps --
print("\nrunning physics test (100 steps)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

for i in range(100):
    app.update()

# check robot hasn't fallen through ground (z should be near 0, not -inf)
robot_xform = UsdGeom.Xformable(stage.GetPrimAtPath("/husky/Geometry/base_link"))
# get world transform
xform_cache = UsdGeom.XformCache()
world_transform = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/husky/Geometry/base_link"))
pos = world_transform.ExtractTranslation()
print(f"  robot position after 100 steps: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
check("robot above ground (z > -1)", pos[2] > -1.0)

timeline.stop()

# -- summary --
# print("DEBUG: isaac sim step")
print(f"\n{'='*50}")
print(f"PASSED: {len(passes)}/{len(passes) + len(errors)}")
if errors:
    print(f"FAILED: {len(errors)}")
    for e in errors:
        print(f"  - {e}")
    app.close()
    sys.exit(1)
else:
    print("all tests passed")

app.close()
print("done")
