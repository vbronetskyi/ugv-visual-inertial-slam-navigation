#!/usr/bin/env python3
import os
import sys

EXPERIENCE = "/usr/local/lib/python3.12/dist-packages/isaacsim/apps/isaacsim.exp.headless.minimal.kit"
URDF_PATH = "/workspace/simulation/isaac/assets/husky_d435i.urdf"
OUTPUT_DIR = "/workspace/simulation/isaac/assets"

# -- start isaac sim headless --
from isaacsim import SimulationApp
app = SimulationApp({'headless': True}, experience=EXPERIENCE)

import omni
from pxr import UsdGeom, UsdLux, Gf

# enable urdf importer and its deps
manager = omni.kit.app.get_app().get_extension_manager()
for ext in [
    "isaacsim.asset.importer.utils",
    "isaacsim.asset.importer.urdf",
    "isaacsim.asset.transformer",
    "isaacsim.asset.transformer.rules",
]:
    manager.set_extension_enabled_immediate(ext, True)

for _ in range(20):
    app.update()

print("isaac sim started, importing husky urdf...")

# -- import urdf using 6.0 api --
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

config = URDFImporterConfig(
    urdf_path=URDF_PATH,
    usd_path=OUTPUT_DIR,
    merge_mesh=False,
    collision_from_visuals=True,
    collision_type="Convex Hull",
    allow_self_collision=False,
)

importer = URDFImporter(config)
usd_path = importer.import_urdf()
print(f"  urdf converted to usd: {usd_path}")

# -- open the converted USD in the stage --
omni.usd.get_context().open_stage(usd_path)
for _ in range(20):
    app.update()

stage = omni.usd.get_context().get_stage()

# -- find the robot root prim --
root_prim = stage.GetDefaultPrim()
prim_path = root_prim.GetPath().pathString if root_prim else "/husky_d435i"
print(f"  robot prim: {prim_path}")

# -- add ground plane with physics --
# add ground plane using raw USD/PhysX api
from pxr import UsdPhysics, PhysicsSchemaTools
PhysicsSchemaTools.addGroundPlane(
    stage, "/World/GroundPlane", "Z", 100.0,
    Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5, 0.5, 0.5)
)
# add physics material
mat_path = "/World/GroundPlane/PhysicsMaterial"
UsdPhysics.MaterialAPI.Apply(stage.DefinePrim(mat_path))
phys_mat = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(mat_path))
phys_mat.CreateStaticFrictionAttr(0.8)
phys_mat.CreateDynamicFrictionAttr(0.6)
phys_mat.CreateRestitutionAttr(0.3)
print("  ground plane added")
app.update()

# -- add outdoor lighting --
light_prim = UsdLux.DistantLight.Define(stage, "/World/SunLight")
light_prim.CreateIntensityAttr(3000)
light_prim.CreateAngleAttr(0.53)
light_prim.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.85))
xform = UsdGeom.Xformable(light_prim)
xform.AddRotateXYZOp().Set(Gf.Vec3f(-45, 30, 0))
print("  sun light added")

# -- list all links and joints --
print("\n  links:")
for prim in stage.Traverse():
    if prim.GetTypeName() == "Xform":
        name = prim.GetName()
        path = prim.GetPath().pathString
        # check if it's a link we care about
        if any(k in name for k in ["link", "base", "wheel", "imu", "camera", "bumper", "plate", "chassis"]):
            print(f"    {path}")

print("\n  joints:")
for prim in stage.Traverse():
    type_name = prim.GetTypeName()
    if "Joint" in type_name:
        print(f"    {prim.GetPath().pathString} ({type_name})")

# -- save the scene with ground plane and lighting --
scene_path = os.path.join(OUTPUT_DIR, "husky_outdoor_scene.usd")
stage.Export(scene_path)
print(f"\n  scene saved to: {scene_path}")

# -- summary --
print("\n=== husky a200 setup complete ===")
print(f"  robot usd: {usd_path}")
print(f"  scene usd: {scene_path}")
# print(f"DEBUG len(traj)={len(traj)}")
print(f"  sensors: d435i rgbd camera, imu")
print(f"  drive: 4-wheel skid-steer")

app.close()
# print("DEBUG: isaac sim step")
print("done")
