#!/usr/bin/env python3
"""
build a forest-to-village outdoor scene using local primitives
no S3 dependencies - everything renders reliably with RayTracedLighting

scene layout (200m x 100m):
  x=-50 to -10: dense forest (trees, rocks, shrubs)
  x=-10 to +10: open field / dirt road transition
  x=+10 to +50: sparse forest + 3 village buildings
  robot spawns at (-40, 0, 0.3) facing +X down the road

usage: /opt/isaac-sim-6.0.0/python.sh build_forest_scene.py
"""
import os
import sys
import numpy as np
import time

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": 'RayTracedLighting'})

import omni
import omni.kit.app
from pxr import UsdGeom, UsdLux, UsdShade, UsdPhysics, PhysxSchema, Gf, Sdf, PhysicsSchemaTools

SCENE_OUT = "/workspace/simulation/isaac/assets/husky_forest_scene.usd"
HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"

omni.usd.get_context().new_stage()
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# physics
phys = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
phys.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
phys.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/PhysicsScene")).CreateTimeStepsPerSecondAttr(60)

rng = np.random.RandomState(42)


# GROUND
print("building ground...")
PhysicsSchemaTools.addGroundPlane(
    stage, "/World/Ground", "Z", 200.0,
    Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.22, 0.38, 0.12)  # forest green
)

# dirt road - flat box along X
road = UsdGeom.Cube.Define(stage, "/World/Terrain/Road")
road.CreateSizeAttr(1.0)
road.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.42, 0.28)])
rxf = UsdGeom.Xformable(road)
rxf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.005))
rxf.AddScaleOp().Set(Gf.Vec3f(120, 3.0, 0.01))

# road edges
for idx, side in enumerate([-1, 1]):
    edge = UsdGeom.Cube.Define(stage, f"/World/Terrain/RoadEdge_{idx}")
    edge.CreateSizeAttr(1.0)
    edge.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.28, 0.18)])
    exf = UsdGeom.Xformable(edge)
    exf.AddTranslateOp().Set(Gf.Vec3d(0, side * 1.8, 0.003))
    exf.AddScaleOp().Set(Gf.Vec3f(120, 0.3, 0.01))


# LIGHTING
print("adding lighting...")
# dome sky - photorealistic forest HDRI from Poly Haven (CC0)
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1500)
hdri_path = "/workspace/simulation/isaac/assets/hdri/forest_slope_2k.hdr"
dome.CreateTextureFileAttr(hdri_path)
dome.CreateTextureFormatAttr("latlong")

# sun
sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
sun.CreateIntensityAttr(5000)
sun.CreateAngleAttr(0.53)
sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
sxf = UsdGeom.Xformable(sun)
sxf.AddRotateXYZOp().Set(Gf.Vec3f(-50, 25, 0))

# fill
fill = UsdLux.DistantLight.Define(stage, "/World/FillLight")
fill.CreateIntensityAttr(1500)
fill.CreateColorAttr(Gf.Vec3f(0.7, 0.8, 1.0))
fxf = UsdGeom.Xformable(fill)
fxf.AddRotateXYZOp().Set(Gf.Vec3f(-30, -160, 0))


# TREE HELPER - cylinder trunk + cone/sphere canopy
def make_tree(parent_path, x, y, trunk_h, trunk_r, canopy_h, canopy_r, tree_type="conifer"):
    """conifer = cone canopy, deciduous = sphere canopy"""
    trunk_color = Gf.Vec3f(0.35 + rng.uniform(-0.05, 0.05),
                           0.22 + rng.uniform(-0.03, 0.03),
                           0.12 + rng.uniform(-0.02, 0.02))
    canopy_color = Gf.Vec3f(0.12 + rng.uniform(-0.04, 0.04),
                            0.35 + rng.uniform(-0.1, 0.1),
                            0.08 + rng.uniform(-0.03, 0.03))

    # trunk
    trunk = UsdGeom.Cylinder.Define(stage, f"{parent_path}/trunk")
    trunk.CreateHeightAttr(trunk_h)
    trunk.CreateRadiusAttr(trunk_r)
    trunk.CreateDisplayColorAttr([trunk_color])
    trunk.CreateAxisAttr("Z")
    txf = UsdGeom.Xformable(trunk)
    txf.AddTranslateOp().Set(Gf.Vec3d(x, y, trunk_h / 2))

    # canopy
    if tree_type == "conifer":
        canopy = UsdGeom.Cone.Define(stage, f"{parent_path}/canopy")
        canopy.CreateHeightAttr(canopy_h)
        canopy.CreateRadiusAttr(canopy_r)
        canopy.CreateAxisAttr("Z")
    else:
        canopy = UsdGeom.Sphere.Define(stage, f"{parent_path}/canopy")
        canopy.CreateRadiusAttr(canopy_r)
    canopy.CreateDisplayColorAttr([canopy_color])
    cxf = UsdGeom.Xformable(canopy)
    cxf.AddTranslateOp().Set(Gf.Vec3d(x, y, trunk_h + canopy_h * 0.4))

    # collision on trunk
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{parent_path}/trunk"))


# FOREST: dense zone (x=-50 to -10)
print("planting dense forest...")
tree_idx = 0
for _ in range(40):
    x = rng.uniform(-50, -10)
    y = rng.uniform(-30, 30)
    # keep off the road (|y| > 3)
    if abs(y) < 4:
        y = 4 * np.sign(y) + rng.uniform(0, 2) * np.sign(y)
    trunk_h = rng.uniform(3, 8)
    trunk_r = rng.uniform(0.1, 0.3)
    canopy_r = rng.uniform(1.5, 4)
    canopy_h = rng.uniform(3, 6)
    ttype = rng.choice(["conifer", "deciduous"])
    make_tree(f"/World/Forest/tree_{tree_idx:03d}", x, y, trunk_h, trunk_r, canopy_h, canopy_r, ttype)
    tree_idx += 1


# SPARSE FOREST: transition + village zone (x=-10 to +50)
print("planting sparse forest + village zone...")
for _ in range(20):
    x = rng.uniform(-10, 50)
    y = rng.uniform(-30, 30)
    if abs(y) < 4:
        y = 4 * np.sign(y) + rng.uniform(0, 3) * np.sign(y)
    trunk_h = rng.uniform(4, 10)
    trunk_r = rng.uniform(0.15, 0.35)
    canopy_r = rng.uniform(2, 5)
    canopy_h = rng.uniform(4, 7)
    ttype = rng.choice(["conifer", "deciduous"])
    make_tree(f"/World/Forest/tree_{tree_idx:03d}", x, y, trunk_h, trunk_r, canopy_h, canopy_r, ttype)
    tree_idx += 1

print(f"  total trees: {tree_idx}")


# ROCKS
print("placing rocks...")
for i in range(20):
    x = rng.uniform(-50, 40)
    y = rng.uniform(-25, 25)
    if abs(y) < 3:
        continue
    rock = UsdGeom.Sphere.Define(stage, f"/World/Rocks/rock_{i:02d}")
    r = rng.uniform(0.2, 0.8)
    rock.CreateRadiusAttr(r)
    rock.CreateDisplayColorAttr([Gf.Vec3f(0.45 + rng.uniform(-0.1, 0.1),
                                          0.42 + rng.uniform(-0.1, 0.1),
                                          0.38 + rng.uniform(-0.1, 0.1))])
    rxf = UsdGeom.Xformable(rock)
    rxf.AddTranslateOp().Set(Gf.Vec3d(x, y, r * 0.5))
    rxf.AddScaleOp().Set(Gf.Vec3f(1.0, rng.uniform(0.7, 1.3), rng.uniform(0.5, 0.8)))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"/World/Rocks/rock_{i:02d}"))


# VILLAGE BUILDINGS (x=30 to 45)
print("building village...")
buildings = [
    ("cabin",   35,  10, 6, 5, 3.5, Gf.Vec3f(0.55, 0.35, 0.2)),   # wooden cabin
    ("barn",    40, -12, 8, 6, 4.5, Gf.Vec3f(0.6, 0.2, 0.15)),    # red barn
    ("cottage", 30,  15, 5, 4, 3.0, Gf.Vec3f(0.85, 0.82, 0.75)),  # white cottage
    ("shed",    45,   5, 3, 3, 2.5, Gf.Vec3f(0.45, 0.4, 0.3)),    # tool shed
]
for name, bx, by, bw, bl, bh, color in buildings:
    # main body
    body = UsdGeom.Cube.Define(stage, f"/World/Village/{name}/body")
    body.CreateSizeAttr(1.0)
    body.CreateDisplayColorAttr([color])
    bxf = UsdGeom.Xformable(body)
    bxf.AddTranslateOp().Set(Gf.Vec3d(bx, by, bh / 2))
    bxf.AddScaleOp().Set(Gf.Vec3f(bw, bl, bh))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"/World/Village/{name}/body"))

    # roof (darker, slightly wider)
    roof = UsdGeom.Cube.Define(stage, f"/World/Village/{name}/roof")
    roof.CreateSizeAttr(1.0)
    roof_color = Gf.Vec3f(color[0] * 0.5, color[1] * 0.4, color[2] * 0.3)
    roof.CreateDisplayColorAttr([roof_color])
    roofxf = UsdGeom.Xformable(roof)
    roofxf.AddTranslateOp().Set(Gf.Vec3d(bx, by, bh + 0.3))
    roofxf.AddScaleOp().Set(Gf.Vec3f(bw + 0.4, bl + 0.4, 0.6))


# FALLEN LOGS (visual clutter for SLAM features)
print("adding fallen logs...")
for i in range(8):
    x = rng.uniform(-45, 35)
    y = rng.uniform(-20, 20)
    if abs(y) < 4:
        continue
    log = UsdGeom.Cylinder.Define(stage, f"/World/Logs/log_{i:02d}")
    log.CreateHeightAttr(rng.uniform(2, 5))
    log.CreateRadiusAttr(rng.uniform(0.1, 0.25))
    log.CreateAxisAttr("Y")  # lying on side
    log.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.2, 0.1)])
    lxf = UsdGeom.Xformable(log)
    lxf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.1))
    angle = rng.uniform(0, 180)
    lxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, angle))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"/World/Logs/log_{i:02d}"))


# CAMERA (standalone, for testing - robot has its own)
cam = UsdGeom.Camera.Define(stage, "/World/ForwardCamera")
cam.CreateFocalLengthAttr(18.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 500.0))
cxf = UsdGeom.Xformable(cam)
cxf.AddTranslateOp().Set(Gf.Vec3d(-40, 0, 1.5))
cxf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))  # look +X, up +Z

# topdown
top = UsdGeom.Camera.Define(stage, "/World/TopDownCamera")
top.CreateFocalLengthAttr(10.0)
top.CreateHorizontalApertureAttr(36.0)
top.CreateClippingRangeAttr(Gf.Vec2f(1.0, 500.0))
txf = UsdGeom.Xformable(top)
txf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 80))


# SAVE, START, CAPTURE
# print(f">>> frame {i}/{n_frames}")
print(f"\nsaving scene: {SCENE_OUT}")
stage.Export(SCENE_OUT)

print("starting simulation...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

from isaacsim.sensors.camera import Camera
from PIL import Image

# print(f"DEBUG state={state} pose={pose}")
print("\ncapturing forward view...")
c_fwd = Camera("/World/ForwardCamera", name="fwd", frequency=30, resolution=(640, 480))
c_fwd.initialize()
for _ in range(120):
    app.update()

rgb = c_fwd.get_rgb()
m = np.mean(rgb) if rgb is not None else -1
print(f"  forward: mean={m:.0f}")
if rgb is not None and m > 5:
    Image.fromarray(rgb[:, :, :3]).save("/tmp/custom_forest_test.png")
    print("  SAVED /tmp/custom_forest_test.png")

print("\ncapturing topdown view...")
c_top = Camera("/World/TopDownCamera", name="top", frequency=10, resolution=(800, 600))
c_top.initialize()
for _ in range(60):
    app.update()

rgb_t = c_top.get_rgb()
if rgb_t is not None and np.mean(rgb_t) > 5:
    Image.fromarray(rgb_t[:, :, :3]).save("/tmp/custom_forest_topdown.png")
    print("  SAVED /tmp/custom_forest_topdown.png")

# Hz
count = 0
t0 = time.time()
while time.time() - t0 < 5:
    app.update()
    if c_fwd.get_rgb() is not None:
        count += 1
hz = count / (time.time() - t0)
print(f"\nrender rate: {hz:.1f} Hz")

timeline.stop()
app.close()
print("done")
