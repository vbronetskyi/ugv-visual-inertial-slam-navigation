#!/usr/bin/env python3
import os, sys, argparse, time, math, json
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=300.0)
parser.add_argument("--route", type=str, default=None,
                    choices=["road", "north", "south", "road_nav2", "warmup", "north_forest", "09_se_ne"],
                    help="predefined route: road (S-curve), north (forest), south (forest), warmup (straight), north_forest (deep N-forest teach, exp 71), 09_se_ne (corner-to-corner teach)")
parser.add_argument("--vio-warmup", action="store_true",
                    help="prepend zigzag warmup waypoints for VIO IMU initialization")
parser.add_argument("--obstacles", action="store_true",
                    help="spawn cones+tent obstacles on route")
parser.add_argument("--synthetic-imu", action="store_true",
                    help="generate synthetic IMU from GT pose (Phidgets 1042 noise) instead of PhysX sensor. "
                         "Avoids PhysX contact-solver micro-oscillations that break ORB-SLAM3 VIO.")
parser.add_argument("--nav2-bridge", action="store_true",
                    help="run as Nav2 bridge (no autonomous driving, wait for cmd_vel)")
parser.add_argument("--spawn-x", type=float, default=-95.0,
                    help="spawn X position (default: -95.0)")
parser.add_argument("--spawn-y", type=float, default=-6.0,
                    help="spawn Y position (default: -6.0)")
parser.add_argument("--spawn-yaw", type=float, default=0.0,
                    help="spawn yaw in radians (default: 0.0)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({
    "headless": True,
    "max_bounces": 4,                       # more bounces = light penetrates tree canopy
    "max_specular_transmission_bounces": 1, # was 3
    "max_volume_bounces": 1,                # was 15
    "samples_per_pixel_per_frame": 48,      # better color accuracy
    "width": 640,                           # was 1280
    "height": 480,                          # was 720
})

import omni, omni.kit.app, omni.kit.commands
import omni.graph.core as og
import usdrt.Sdf
import carb
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, PhysicsSchemaTools, Usd

# GPU optimizations
settings = carb.settings.get_settings()
settings.set("/rtx/raytracing/backfaceCulling", False)
settings.set("/rtx/directLighting/backfaceCulling", False)
# post-processing off (not needed for SLAM)
settings.set("/rtx/post/motionblur/enabled", False)
settings.set("/rtx/post/dof/enabled", False)
settings.set("/rtx/post/bloom/enabled", False)
settings.set("/rtx/post/lensFlares/enabled", False)
settings.set("/rtx/directLighting/sampledLighting/enabled", False)
# reflections off, indirect diffuse on (light through canopy)
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/indirectDiffuse/enabled", True)
# fabric off, PhysX needs direct USD sync for articulation control
settings.set("/persistent/omnigraph/updateToUsd", True)
settings.set("/persistent/omnihydra/useSceneGraphInstancing", False)
# render at 200fps = 1 physics step per app.update() -> real 200Hz IMU
settings.set("/app/runLoops/main/rateLimitFrequency", 200)
settings.set("/app/runLoops/main/rateLimitEnabled", True)

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"
RENDERS = "/workspace/simulation/isaac/assets/renders/maps"

# enable ros2
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()
print(f"ros2: {manager.get_enabled_extension_id('isaacsim.ros2.bridge')}")

# load forest scene
print(f"loading forest scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

# Shrub fix (scene-audit):
# /World/ShrubCol/sc_* colliders were hardcoded at z=0.20 in the baked USD.
# Real terrain varies ±0.5 m -> shrubs visibly floated up to 70 cm above ground.
# Two corrections at runtime:
#   1. Shrub visual: place base on real terrain (z = terrain_h(x,y))
#   2. Collider: keep at shrub CENTER (z = terrain_h + 0.3 m) with small
#      radius 0.15 m - only the woody stem is impassable; foliage is drive-thru
def _terrain_height_early(x, y):
    import math as _m
    RWPS = [(-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
            (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
            (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
            (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
            (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5)]
    def _ry(x):
        if x <= RWPS[0][0]: return RWPS[0][1]
        if x >= RWPS[-1][0]: return RWPS[-1][1]
        for i in range(len(RWPS)-1):
            if RWPS[i][0] <= x <= RWPS[i+1][0]:
                tt = (x - RWPS[i][0]) / (RWPS[i+1][0] - RWPS[i][0])
                return RWPS[i][1] + tt * (RWPS[i+1][1] - RWPS[i][1])
        return 0
    h = 0.5*_m.sin(x*0.018+0.5)*_m.cos(y*0.022+1.2) + 0.35*_m.sin(x*0.035+2.1)*_m.sin(y*0.03+0.7) + 0.18*_m.sin(x*0.07+3.3)*_m.cos(y*0.065+2.5) + 0.12*_m.cos(x*0.11+1.0)*_m.sin(y*0.09+4.0) + 0.06*_m.sin(x*0.5+0.7)*_m.cos(y*0.43+2.1) + 0.04*_m.cos(x*0.7+3.5)*_m.sin(y*0.6+0.4) + 0.03*_m.sin(x*1.0+1.2)*_m.cos(y*0.83+3.8)
    rd = abs(y - _ry(x))
    if rd < 4.0: h *= (rd/4.0)**2
    if rd < 2.0: h -= 0.06*(1.0 - rd/2.0)
    return max(h, -0.5)


_shrub_root = stage.GetPrimAtPath("/World/ShrubCol")
if _shrub_root.IsValid():
    _shrub_src_candidates = [
        "/World/Roadside/rsp_0011/mesh/veg_shrub_sm_02_inst",
        "/World/Roadside/rsp_0006/mesh/veg_shrub_gen_05_inst",
        "/World/Roadside/rsp_0002/mesh/veg_shrub_gen_07_inst",
    ]
    _src = next((p for p in _shrub_src_candidates
                 if stage.GetPrimAtPath(p).IsValid()), None)
    UsdGeom.Xform.Define(stage, "/World/Shrubs")
    _n_vis = 0
    _n_fixed = 0
    for _c in _shrub_root.GetChildren():
        if not _src:
            continue
        _t = UsdGeom.Xformable(_c).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        _x, _y = float(_t[0]), float(_t[1])
        _gz = _terrain_height_early(_x, _y)
        # (1) collider at bush CENTER - terrain + 0.3 m, shrunken radius
        _xf = UsdGeom.Xformable(_c)
        try: _xf.ClearXformOpOrder()
        except Exception: pass
        _xf.AddTranslateOp().Set(Gf.Vec3d(_x, _y, _gz + 0.3))
        # Shrink sphere collider if the prim is a Sphere
        try:
            _sph = UsdGeom.Sphere(_c)
            if _sph:
                _sph.CreateRadiusAttr().Set(0.15)
        except Exception:
            pass
        _n_fixed += 1
        # (2) visual on ground at terrain height
        _vis_path = f"/World/Shrubs/{_c.GetName()}_vis"
        _vis = UsdGeom.Xform.Define(stage, _vis_path)
        _vis.AddTranslateOp().Set(Gf.Vec3d(_x, _y, _gz))
        _vis.GetPrim().GetReferences().AddInternalReference(_src)
        _n_vis += 1
    print(f"  shrubs: {_n_vis} visuals on terrain, {_n_fixed} colliders at bush-center r=0.15m")

# /World/Cover is empty in the baked scene.  Scatter ferns/grass/leaves around
# every tree at runtime - purpose=render (no collision, driveable) - so ORB
# has extra visual features on forest floor.  Deterministic seed so teach and
# repeat scatter in the same positions.
_cover_root = stage.GetPrimAtPath("/World/Cover")
if _cover_root and _cover_root.IsValid():
    _tree_root = stage.GetPrimAtPath("/World/Trees")
    if _tree_root and _tree_root.IsValid():
        import random as _rnd
        _rng = _rnd.Random(17)
        _ASSETS = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
        _DS = f"{_ASSETS}/NVIDIA/dsready_content/nv_content/common_assets/props_vegetation"
        _COVER_ASSETS = [
            f"{_DS}/veg_grass_clump_01/veg_grass_clump_01.usd",
            f"{_DS}/veg_grass_clump_02/veg_grass_clump_02.usd",
            f"{_DS}/sct_debris_leaves_dry_01/sct_debris_leaves_dry_01.usd",
            f"{_DS}/sct_debris_leaves_dry_02/sct_debris_leaves_dry_02.usd",
        ]
        _cov_idx = 0
        _trees = list(_tree_root.GetChildren())
        for _tree in _trees:
            _tt = UsdGeom.Xformable(_tree).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            _tx, _ty = float(_tt[0]), float(_tt[1])
            for _ in range(3):  # 3 cover items per tree ⇒ ~400 over 130 trees
                _dx = _rng.uniform(-5, 5)
                _dy = _rng.uniform(-5, 5)
                _cx, _cy = _tx + _dx, _ty + _dy
                if -100 < _cx < 80 and -50 < _cy < 10:
                    _cgz = _terrain_height_early(_cx, _cy)
                    _asset = _COVER_ASSETS[_cov_idx % len(_COVER_ASSETS)]
                    _cov_path = f"/World/Cover/cov_{_cov_idx:04d}"
                    _cov_xf = UsdGeom.Xform.Define(stage, _cov_path)
                    _cov_xf.AddTranslateOp().Set(Gf.Vec3d(_cx, _cy, _cgz))
                    _cov_xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, _rng.uniform(0, 360)))
                    UsdGeom.Imageable(_cov_xf.GetPrim()).CreatePurposeAttr("render")
                    _cov_xf.GetPrim().GetReferences().AddReference(_asset)
                    _cov_idx += 1
        print(f"  cover: {_cov_idx} ferns/grass/leaves scattered (render-only, no collision)")

# RoadsideTrees fix: rtcol_* (Cylinder trunk colliders) are hardcoded at
# z=2.0 absolute in the baked USD.  With terrain varying ±0.5 m, the
# bottom of the cylinder floats or buries.  Re-translate so the cylinder
# sits on real terrain (center z = terrain_z + height/2 = terrain_z + 2.0).
_rtcol_root = stage.GetPrimAtPath("/World/RoadsideTrees")
if _rtcol_root and _rtcol_root.IsValid():
    _n_rt_fix = 0
    _rt_children = list(_rtcol_root.GetChildren())
    for _c in _rt_children:
        if not _c.GetName().startswith("rtcol_"):
            continue
        _t = UsdGeom.Xformable(_c).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()).ExtractTranslation()
        _x, _y, _z = float(_t[0]), float(_t[1]), float(_t[2])
        _gz = _terrain_height_early(_x, _y)
        _new_z = _gz + 2.0  # cylinder height=4m, center at terrain + 2
        if abs(_z - _new_z) > 0.1:
            _xf = UsdGeom.Xformable(_c)
            try: _xf.ClearXformOpOrder()
            except Exception: pass
            _xf.AddTranslateOp().Set(Gf.Vec3d(_x, _y, _new_z))
            _n_rt_fix += 1
    # print("DEBUG: entering main loop")
    print(f"  roadside_trees: z-fixed {_n_rt_fix}/{sum(1 for c in _rt_children if c.GetName().startswith('rtcol_'))} cylinders to terrain+2m")

# Spawn obstacles if requested
if args.obstacles:
    _route_for_obs = args.route or "road"
    from spawn_obstacles import spawn_obstacles as _spawn_obs
    _n_obs = _spawn_obs(stage, _route_for_obs)
    print(f"  spawned {_n_obs} obstacles on {_route_for_obs}")
    for _ in range(30):
        app.update()

# load husky with PhysX articulation (floating base, wheel drive)
print("adding Husky A200...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(300):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"
IMU_PATH = "/World/Husky/Geometry/base_link/imu_link"
WHEELS = ["front_left_wheel_joint", "front_right_wheel_joint",
          "rear_left_wheel_joint", "rear_right_wheel_joint"]

# 200Hz physics with TGS solver (reduces contact oscillations for cleaner IMU)
_phys_scene = stage.GetPrimAtPath("/World/PhysicsScene")
if _phys_scene.IsValid():
    PhysxSchema.PhysxSceneAPI(_phys_scene).GetTimeStepsPerSecondAttr().Set(200)
    PhysxSchema.PhysxSceneAPI(_phys_scene).CreateSolverTypeAttr().Set("TGS")
    print("  physics: 200Hz, TGS solver")

# root_joint removed from USD (physics.usda) to enable floating base
_root_joint = stage.GetPrimAtPath("/World/Husky/Physics/root_joint")
if _root_joint.IsValid():
    print("  WARNING: root_joint still exists, base may be fixed")

# IMU sensor frame in this USD: UBR (Up-Backward-Right)
# Verified by imu_cal_test.py:
#   - Stationary: gravity (+9.81) on raw sensor +X => sensor X = UP
#   - Yaw left (CCW): ang_vel positive on raw sensor +X => sensor X is yaw axis (UP)
#   - Forward drive: lin_acc on raw sensor -Y => sensor Y = -FORWARD = BACKWARD
#   - Right-hand rule: UP × FORWARD = LEFT, +X × -Y = -Z, so sensor Z = -LEFT = RIGHT
#
# ORB-SLAM3 / our convention: body FLU (X=Forward, Y=Left, Z=Up)
def _imu_urf_to_flu(sx, sy, sz):
    # FIXME: breaks if matcher silent >30s, need a fallback
    """Convert IMU vector from sensor UBR frame to body FLU frame.
    Function name kept for backward compatibility but actual conversion is UBR->FLU.
       flu_x (forward) = -sy (negate backward)
       flu_y (left)    = -sz (negate right)
       flu_z (up)      = +sx
    """
    return (-sy, -sz, sx)

# IMU sensor on imu_link
IMU_SENSOR_PATH = IMU_PATH + "/imu_sensor"
_imu_ok, _imu_prim = omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor",
    parent=IMU_PATH,
    sensor_period=1.0 / 200.0,
    linear_acceleration_filter_size=20,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
print(f"  IMU sensor: {IMU_SENSOR_PATH}")

# wheel friction material
from pxr import UsdShade
_wf_mat = UsdShade.Material.Define(stage, "/World/WheelFriction")
_wf_phys = UsdPhysics.MaterialAPI.Apply(_wf_mat.GetPrim())
_wf_phys.CreateStaticFrictionAttr(1.0)
_wf_phys.CreateDynamicFrictionAttr(0.8)
_wf_phys.CreateRestitutionAttr(0.0)
for _wl in ["front_left_wheel_link", "front_right_wheel_link",
            "rear_left_wheel_link", "rear_right_wheel_link"]:
    _col = stage.GetPrimAtPath(f"{BASE_LINK}/{_wl}/collision")
    if _col.IsValid():
        UsdShade.MaterialBindingAPI.Apply(_col).Bind(_wf_mat, materialPurpose="physics")
print("  wheel friction: static=1.0, dynamic=0.8")

# camera (independent prim, follows robot)
CAM_RGB = "/World/HuskyCamera"
cam = UsdGeom.Camera.Define(stage, CAM_RGB)
cam.CreateFocalLengthAttr(1.93)
cam.CreateHorizontalApertureAttr(3.86)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
_cam_xf = UsdGeom.Xformable(cam)
_cam_transform_op = _cam_xf.AddTransformOp()
CAM_FWD = 0.5
CAM_UP = 0.48

def _make_cam_matrix(x, y, z, yaw, pitch=0):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    # Base rotation (yaw only): row0=(sy,-cy,0) row1=(0,0,1) row2=(-cy,-sy,0)
    # Add pitch: rotate around camera's local X axis (row0)
    # row1 rotated by pitch: row1*cos(p) + row2*sin(p)
    # row2 rotated by pitch: -row1*sin(p) + row2*cos(p)
    r0x, r0y, r0z = sy, -cy, 0
    r1x, r1y, r1z = 0*cp + (-cy)*sp, 0*cp + (-sy)*sp, 1*cp + 0*sp
    r2x, r2y, r2z = 0*(-sp) + (-cy)*cp, 0*(-sp) + (-sy)*cp, 1*(-sp) + 0*cp
    return Gf.Matrix4d(
        r0x, r0y, r0z, 0,
        r1x, r1y, r1z, 0,
        r2x, r2y, r2z, 0,
          x,   y,   z, 1
    )

_cam_transform_op.Set(_make_cam_matrix(-95 + CAM_FWD, -6, 0.2 + CAM_UP, 0))
# print("DEBUG: entering main loop")
print(f"  camera: {CAM_RGB}")

# verify prim paths
for name, path in [("base_link", BASE_LINK), ("camera", CAM_RGB), ("imu", IMU_PATH)]:
    print(f"  {name}: {stage.GetPrimAtPath(path).IsValid()} ({path})")

# RigidPrim for direct velocity reading (used by --synthetic-imu)
from isaacsim.core.prims import SingleRigidPrim as _SingleRigidPrim
_husky_rigid = _SingleRigidPrim(prim_path=BASE_LINK)
_base_link_prim = stage.GetPrimAtPath(BASE_LINK)

# spawn position
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_husky_translate_op = husky_xf.AddTranslateOp()
_husky_rotate_op = husky_xf.AddRotateXYZOp()
_spawn_x, _spawn_y = args.spawn_x, args.spawn_y
_spawn_yaw_deg = math.degrees(args.spawn_yaw)
_husky_translate_op.Set(Gf.Vec3d(_spawn_x, _spawn_y, 10.0))  # high placeholder - must stay above terrain peaks
_husky_rotate_op.Set(Gf.Vec3f(0, 0, _spawn_yaw_deg))
print(f"  spawn: ({_spawn_x}, {_spawn_y}, yaw={args.spawn_yaw:.2f}rad)")

# ros2 graph - extra pumping to let extensions register
# (large stage modifications above can delay ext loading past edit() call)
for _ in range(60):
    app.update()
print("\ncreating ros2 graph...")
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("CreateCamRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PubIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("CreateCamRP.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_RGB)]),
            ("CreateCamRP.inputs:width", 640),
            ("CreateCamRP.inputs:height", 480),
            ("RGBPub.inputs:topicName", "camera/color/image_raw"),
            ("RGBPub.inputs:type", "rgb"),
            ("RGBPub.inputs:frameId", "camera_link"),
            ("DepthPub.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPub.inputs:type", "depth"),
            ("DepthPub.inputs:frameId", "camera_link"),
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(IMU_SENSOR_PATH)]),
            ("ReadIMU.inputs:readGravity", True),
            ("PubIMU.inputs:topicName", "imu/data"),
            ("PubIMU.inputs:frameId", "imu_link"),
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(BASE_LINK)]),
            ("PubOdom.inputs:topicName", "odom"),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_link"),
            ("PubTF.inputs:topicName", "tf_isaac"),  # avoid conflict with tf_wall_clock_relay
            ("PubTF.inputs:targetPrims", [usdrt.Sdf.Path(BASE_LINK)]),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateCamRP.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PubTF.inputs:execIn"),
            ("CreateCamRP.outputs:execOut", "RGBPub.inputs:execIn"),
            ("CreateCamRP.outputs:execOut", "DepthPub.inputs:execIn"),
            ("CreateCamRP.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
            ("CreateCamRP.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            ("ReadIMU.outputs:execOut", "PubIMU.inputs:execIn"),
            ("ReadIMU.outputs:angVel", "PubIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PubIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PubIMU.inputs:orientation"),
            ("ReadSimTime.outputs:simulationTime", "PubIMU.inputs:timeStamp"),
            ("ComputeOdom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("ComputeOdom.outputs:position", "PubOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PubOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("ReadSimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ],
    },
)
print("  ros2 graph created")

# wheel drive parameters (velocity control: stiffness=0, high damping)
print("  configuring wheel drives...")
_wheel_vel_attrs = []
for wname in ["front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"]:
    wp = f"/World/Husky/Physics/{wname}"
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(wp), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())
        print(f"    {wname}: damping=100000, maxForce=500")

# start simulation
print(f"\nstarting simulation ({args.duration}s)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

print("warming up...")
for _ in range(300):
    app.update()

# camera render product for recording + web UI
import omni.replicator.core as rep
rp_fwd = rep.create.render_product(CAM_RGB, (640, 480))
ann_fwd = rep.AnnotatorRegistry.get_annotator("rgb")
ann_fwd.attach([rp_fwd])
ann_depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
ann_depth.attach([rp_fwd])

for _ in range(200):
    app.update()
print("  camera render product ready")

# IMU sensor interface
from isaacsim.sensors.physics import _sensor as _imu_mod
_imu_interface = _imu_mod.acquire_imu_sensor_interface()

# no articulation -- base_link is a regular rigid body now
# wheels controlled via USD DriveAPI, pose via XformCache
_base_link_prim = stage.GetPrimAtPath(BASE_LINK)
print(f"  base_link: {_base_link_prim.IsValid()}")

# _wheel_vel_attrs already set up earlier (DriveAPI target velocity)

def _get_husky_pose():
    """robot pose from PhysX via XformCache (no articulation)"""
    xf_cache = UsdGeom.XformCache()
    world_tf = xf_cache.GetLocalToWorldTransform(_base_link_prim)
    pos = world_tf.ExtractTranslation()
    rot = world_tf.ExtractRotationMatrix()
    fwd_x, fwd_y = rot[0][0], rot[0][1]
    yaw = math.atan2(fwd_y, fwd_x)
    return (np.array([pos[0], pos[1], pos[2]]),
            np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]))

def _get_husky_full_quat():
    """Full quaternion (qx,qy,qz,qw) from rotation matrix - for synth IMU only.
    USD GfMatrix3d uses ROW-VECTOR convention (p' = p * M), scipy uses
    COLUMN-VECTOR (p' = M * p), so we TRANSPOSE before from_matrix()."""
    xf_cache = UsdGeom.XformCache()
    world_tf = xf_cache.GetLocalToWorldTransform(_base_link_prim)
    rot = world_tf.ExtractRotationMatrix()
    from scipy.spatial.transform import Rotation as _R
    R = np.array([[float(rot[i][j]) for j in range(3)] for i in range(3)]).T
    return _R.from_matrix(R).as_quat()  # (qx, qy, qz, qw)

p0 = _get_husky_pose()[0]
print(f"  position: ({p0[0]:.1f}, {p0[1]:.1f}, {p0[2]:.2f})")

# collision list (matches thinning from convert_gazebo_to_isaac.py)
_obstacles = []
with open("/tmp/gazebo_models.json") as _f:
    _models = json.load(_f)

_RWPS = [
    (-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
    (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
    (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
    (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
    (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5),
]
def _road_y(x):
    if x <= _RWPS[0][0]: return _RWPS[0][1]
    if x >= _RWPS[-1][0]: return _RWPS[-1][1]
    for i in range(len(_RWPS)-1):
        if _RWPS[i][0] <= x <= _RWPS[i+1][0]:
            t = (x - _RWPS[i][0]) / (_RWPS[i+1][0] - _RWPS[i][0])
            return _RWPS[i][1] + t * (_RWPS[i+1][1] - _RWPS[i][1])
    return 0

# terrain height function (must match convert_gazebo_to_isaac.py)
def _terrain_height(x, y):
    h = 0.0
    h += 0.5 * math.sin(x * 0.018 + 0.5) * math.cos(y * 0.022 + 1.2)
    h += 0.35 * math.sin(x * 0.035 + 2.1) * math.sin(y * 0.03 + 0.7)
    h += 0.18 * math.sin(x * 0.07 + 3.3) * math.cos(y * 0.065 + 2.5)
    h += 0.12 * math.cos(x * 0.11 + 1.0) * math.sin(y * 0.09 + 4.0)
    # small bumps (forest floor)
    h += 0.06 * math.sin(x * 0.5 + 0.7) * math.cos(y * 0.43 + 2.1)
    h += 0.04 * math.cos(x * 0.7 + 3.5) * math.sin(y * 0.6 + 0.4)
    h += 0.03 * math.sin(x * 1.0 + 1.2) * math.cos(y * 0.83 + 3.8)
    road_dist = abs(y - _road_y(x))
    if road_dist < 4.0:
        h *= (road_dist / 4.0) ** 2
    if road_dist < 2.0:
        h -= 0.06 * (1.0 - road_dist / 2.0)
    return max(h, -0.5)

# spawn above terrain, PhysX will settle the robot during warmup
_spawn_z = _terrain_height(_spawn_x, _spawn_y) + 0.5
_husky_translate_op.Set(Gf.Vec3d(_spawn_x, _spawn_y, _spawn_z))
_husky_rotate_op.Set(Gf.Vec3f(0, 0, _spawn_yaw_deg))
# Also use the PhysX rigid-body API to teleport - the earlier Xformable .Set
# alone doesn't re-sync the PhysX actor pose once articulation was initialised.
try:
    import numpy as _np
    _q = _np.array([math.cos(args.spawn_yaw/2), 0.0, 0.0, math.sin(args.spawn_yaw/2)])  # [w,x,y,z]
    _husky_rigid.set_world_pose(position=_np.array([_spawn_x, _spawn_y, _spawn_z]), orientation=_q)
    _husky_rigid.set_linear_velocity(_np.zeros(3))
    _husky_rigid.set_angular_velocity(_np.zeros(3))
    print(f"  PhysX rigid teleported to ({_spawn_x}, {_spawn_y}, {_spawn_z:.2f})")
except Exception as _e:
    print(f"  [warn] PhysX teleport failed: {_e}")
_cam_transform_op.Set(_make_cam_matrix(_spawn_x + CAM_FWD * math.cos(args.spawn_yaw), _spawn_y + CAM_FWD * math.sin(args.spawn_yaw), _spawn_z + CAM_UP, args.spawn_yaw))
print(f"  spawn z={_spawn_z:.2f} at ({_spawn_x}, {_spawn_y})")

# replicate thinning: remove 5 near each west corner, skip every 3rd
_all_trees = [m for m in _models if m["type"] in ("pine", "oak")]
_west_sw = sorted(_all_trees, key=lambda m: math.hypot(m["x"]+120, m["y"]+80))
_west_nw = sorted(_all_trees, key=lambda m: math.hypot(m["x"]+120, m["y"]-80))
_remove = set(t["name"] for t in _west_sw[:5]) | set(t["name"] for t in _west_nw[:5])
_skip = 0
for m in _all_trees:
    if m["name"] in _remove:
        continue
    _skip += 1
    if _skip % 3 == 0:
        continue
    _obstacles.append((m["x"], m["y"], 0.7))

# rocks (with road shift)
for m in _models:
    if m["type"] == "rock":
        rx, ry = m["x"], m["y"]
        if abs(ry - _road_y(rx)) < 5.0:
            ry = _road_y(rx) + 8.0 * (1 if ry >= _road_y(rx) else -1)
        _obstacles.append((rx, ry, 0.8))
    elif m["type"] == "house":
        _obstacles.append((m["x"], m["y"], 6.0))
    elif m["type"] == "barrel":
        _obstacles.append((m["x"], m["y"], 0.5))
    elif m["type"] in ("fallen_oak", "fallen_pine"):
        # fallen tree scaled 1.5-2x in scene, trunk ~12-16m long
        yaw = m.get("yaw", 0)
        for d in [-7, -5, -3, -1, 0, 1, 3, 5, 7]:
            _obstacles.append((m["x"] + d * math.cos(yaw), m["y"] + d * math.sin(yaw), 0.6))
    elif m["type"] == "shrub":
        _obstacles.append((m["x"], m["y"], 0.1))
print(f"  {len(_obstacles)} collision obstacles (trees + shrubs + fallen logs + rocks + houses)")

# verify ros2 topics
import rclpy
from sensor_msgs.msg import Image as RosImage, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("husky_sim")

# cmd_vel subscriber
latest_cmd = [Twist()]
def cmd_cb(msg):
    # FIXME: spawn-x/y hardcoded, sync with run_repeat.sh
    latest_cmd[0] = msg
node.create_subscription(Twist, "/cmd_vel", cmd_cb, 10)

# topic counter
topic_counts = {}
def make_cb(t):
    def cb(msg):
        topic_counts[t] = topic_counts.get(t, 0) + 1
    return cb
node.create_subscription(RosImage, "/camera/color/image_raw", make_cb("cam"), 10)
node.create_subscription(Imu, "/imu/data", make_cb("imu"), 10)
node.create_subscription(Odometry, "/odom", make_cb("odom"), 10)

print("\nmeasuring topics (5s)...")
t0 = time.time()
while time.time() - t0 < 5:
    app.update()
    rclpy.spin_once(node, timeout_sec=0.001)

elapsed = time.time() - t0
print("ros2 topics:")
for t in sorted(topic_counts):
    print(f"  {t}: {topic_counts[t]} msgs, {topic_counts[t]/elapsed:.1f} Hz")

p1 = _get_husky_pose()[0]
print(f"\nrobot: ({p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.2f})")
print(f"running for {args.duration}s... (ctrl+c to stop)\n")

# road waypoints for path following (match convert_gazebo_to_isaac.py)
ROAD_WPS = [
    (-100, -7.0), (-95, -6.0), (-90, -4.5), (-85, -2.8), (-80, -1.5),
    (-75, -0.8), (-70, -0.5), (-65, -1.0), (-60, -2.2), (-55, -3.8),
    (-50, -5.0), (-45, -5.5), (-40, -5.2), (-35, -4.0), (-30, -2.5),
    (-25, -1.0), (-20, 0.2), (-15, 1.2), (-10, 1.8), (-5, 2.0),
    (0, 1.5), (5, 0.5), (10, -0.8), (15, -2.2), (20, -3.5),
    (25, -4.2), (30, -4.0), (35, -3.0), (40, -1.8), (45, -0.8),
    (50, -0.5), (55, -1.0), (60, -2.0), (65, -3.2), (70, -4.5), (75, -5.0),
]

def find_road_path(robot_x, robot_y, goal_x, goal_y):
    # find closest wp to robot
    def closest_wp(x, y):
        best_i, best_d = 0, 1e9
        for i, (wx, wy) in enumerate(ROAD_WPS):
            d = math.hypot(wx - x, wy - y)
            if d < best_d:
                best_i, best_d = i, d
        return best_i
    ri = closest_wp(robot_x, robot_y)
    gi = closest_wp(goal_x, goal_y)
    # return waypoints in order from robot to goal
    if ri <= gi:
        return [ROAD_WPS[i] for i in range(ri, gi + 1)]
    else:
        return [ROAD_WPS[i] for i in range(ri, gi - 1, -1)]

# routes from A* pathfinding (start -> houses -> return)
_ROUTES_FILE = "/tmp/slam_routes.json"
try:
    with open(_ROUTES_FILE) as _rf:
        _ROUTES = json.load(_rf)
    NORTH_ROUTE = [tuple(p) for p in _ROUTES["north"]]
    SOUTH_ROUTE = [tuple(p) for p in _ROUTES["south"]]
    ROAD_ROUTE = [tuple(p) for p in _ROUTES["road"]]
    print(f"  loaded routes: north={len(NORTH_ROUTE)}, south={len(SOUTH_ROUTE)}, road={len(ROAD_ROUTE)} wps")
except FileNotFoundError:
    print(f"  WARNING: {_ROUTES_FILE} not found, routes unavailable")
    NORTH_ROUTE = SOUTH_ROUTE = ROAD_ROUTE = []

current_path = []
path_idx = 0

# auto-navigate predefined route
_auto_route = None
_auto_idx = 1  # start from waypoint 1 (waypoint 0 is spawn)

# Warmup waypoints - first 12 road S-curve waypoints (proven to work on exp18)
# Smooth S-curve from spawn east, gives VIO IMU init time on natural road motion
WARMUP_WAYPOINTS = [
    (-90, -4.5), (-85, -2.8), (-80, -1.5), (-75, -0.8),
    (-70, -0.5), (-65, -1.0), (-60, -2.2), (-55, -3.8),
    (-50, -5.0), (-45, -5.5), (-40, -5.2), (-35, -4.0),
]

if args.route == "north":
    _auto_route = list(NORTH_ROUTE)
    print(f"  AUTO ROUTE: north forest ({len(_auto_route)} waypoints)")
elif args.route == "south":
    _auto_route = list(SOUTH_ROUTE)
    print(f"  AUTO ROUTE: south forest ({len(_auto_route)} waypoints)")
elif args.route == "road":
    _auto_route = list(ROAD_ROUTE)
    print(f"  AUTO ROUTE: road ({len(_auto_route)} waypoints)")
elif args.route == "road_nav2":
    import json as _json
    with open(_ROUTES_FILE) as _rf:
        _all_routes = _json.load(_rf)
    _auto_route = list(_all_routes.get("road_nav2", []))
    print(f"  AUTO ROUTE: road_nav2 ({len(_auto_route)} waypoints)")
elif args.route == "north_forest":
    import json as _json
    with open(_ROUTES_FILE) as _rf:
        _all_routes = _json.load(_rf)
    _auto_route = list(_all_routes.get("north_forest", []))
    print(f"  AUTO ROUTE: north_forest ({len(_auto_route)} waypoints)")
elif args.route == "09_se_ne":
    import json as _json
    with open(_ROUTES_FILE) as _rf:
        _all_routes = _json.load(_rf)
    _auto_route = list(_all_routes.get("09_se_ne", []))
    print(f"  AUTO ROUTE: 09_se_ne ({len(_auto_route)} waypoints)")
elif args.route == "warmup":
    # Standalone aggressive zigzag for VIO IMU init testing
    _auto_route = list(WARMUP_WAYPOINTS)
    print(f"  AUTO ROUTE: warmup aggressive zigzag ({len(_auto_route)} waypoints)")

# Prepend VIO warmup to forest routes if --vio-warmup flag is set
if args.vio_warmup and _auto_route is not None and args.route in ("north", "south", "road"):
    # Warmup ends at the last WARMUP_WAYPOINTS point.
    # Skip initial route waypoints that are behind or at the warmup-end X,
    # so robot continues forward smoothly instead of doubling back to spawn.
    import math as _math
    warmup_end_x = WARMUP_WAYPOINTS[-1][0]
    warmup_end_y = WARMUP_WAYPOINTS[-1][1]
    SKIP_MARGIN = 2.0  # skip any wp whose x is < warmup_end_x + margin
    route_tail = _auto_route
    skipped = 0
    while route_tail and route_tail[0][0] < warmup_end_x + SKIP_MARGIN:
        route_tail = route_tail[1:]
        skipped += 1
    _auto_route = list(WARMUP_WAYPOINTS) + route_tail
    # print(f"DEBUG turnaround fire? {fired}")
    print(f"  PREPENDED VIO WARMUP ({len(WARMUP_WAYPOINTS)} wp), "
          f"skipped {skipped} initial route wp behind warmup -> "
          f"total {len(_auto_route)} waypoints")

if args.nav2_bridge:
    _auto_route = None
    print("  NAV2 BRIDGE MODE: no auto-route, waiting for /cmd_vel")

sim_time = 5.0
wheel_r = 0.165   # husky wheel radius
track = 0.555     # husky track width
goal_x, goal_y = None, None

# recording setup
import time as _time
_rec_dir = f"/root/bags/husky_real/isaac_slam_{int(_time.time())}"
os.makedirs(f"{_rec_dir}/camera_rgb", exist_ok=True)
os.makedirs(f"{_rec_dir}/camera_depth", exist_ok=True)
_gt_file = open(f"{_rec_dir}/groundtruth.csv", "w")
_gt_file.write("timestamp,x,y,z,yaw\n")
# TUM-format GT with camera position (for ORB-SLAM3 evaluation)
_gt_tum_file = open(f"{_rec_dir}/groundtruth_tum.txt", "w")
_odom_file = open(f"{_rec_dir}/odom.csv", "w")
_odom_file.write("timestamp,x,y,z,qx,qy,qz,qw,lin_vel,ang_vel\n")
_imu_file = open(f"{_rec_dir}/imu.csv", "w")
_imu_file.write("timestamp,ax,ay,az,gx,gy,gz,qx,qy,qz,qw\n")
_recording = True
rng = np.random.RandomState(42)

# Synthetic IMU state (for --synthetic-imu mode)
_synth_prev_pos = None        # world-frame position
_synth_prev_vel = None        # world-frame velocity
_synth_prev_quat = None       # (qx, qy, qz, qw)
_synth_prev_omega = None      # previous angular velocity (for LPF)
_synth_prev_time = None
# Phidgets Spatial 1042 noise specs
# SKIP_RADIUS = 4.0  # 3.0 too tight, missed behind-obstacle WPs
_SYNTH_GYRO_STD = 0.005       # rad/s white noise
# MATCH_RATIO = 0.7  # 0.75 was default, bit too loose
_SYNTH_ACCEL_STD = 0.02       # m/s^2 white noise
# Constant biases for this run
_SYNTH_GYRO_BIAS = rng.normal(0, 0.001, 3)
_SYNTH_ACCEL_BIAS = rng.normal(0, 0.005, 3)
if args.synthetic_imu:
    print(f"  SYNTHETIC IMU enabled (Phidgets 1042: gyro_std={_SYNTH_GYRO_STD}, accel_std={_SYNTH_ACCEL_STD})")
    print(f"    bias(gyro)={_SYNTH_GYRO_BIAS.round(4)}  bias(accel)={_SYNTH_ACCEL_BIAS.round(4)}")

from collections import deque
_synth_prev_pos = None
_synth_prev_vel_world = None
_synth_accel_buf = deque(maxlen=11)
_synth_prev_quat = None
_synth_prev_time = None
_synth_prev_omega = None
# Exp 51 v2 IMU fix: standstill detection. PhysX contact-solver jitter causes
# ~0.1mm position noise per 5ms step; double-differentiation amplifies this
# to ±1.1 m/s² phantom accel. A real IMU on a stationary robot reads pure
# gravity + sensor noise, NOT position jitter - so we detect standstill and
# bypass the derivative chain.
# v7 alternative (PhysX velocity API for accel) had lower noise but 7%
# systematic path deficit -> VIO drifted worse than v6. Position-double-diff
# is noisier per-sample but mean-energy correct (ratio 1.026 vs v7's 0.93),
# which is what ORB-SLAM3 BA actually needs. Use with vio_th160.yaml
# `NoiseAcc: 1.5` so BA weights noise correctly.
_synth_pos_hist = deque(maxlen=20)  # 100ms @ 200Hz
_SYNTH_STAND_THRESH = 0.015          # 15mm over window

def _compute_synth_imu(pos_arg, quat_xyzw, pos_for_vel, t):
    """Synthetic IMU: world velocity from position diff, accel from vel diff, LPF.
    When GT position is stationary (<15mm drift over 100ms), output pure gravity
    in body frame + Phidgets sensor noise - avoids PhysX jitter being amplified
    into phantom acceleration that ORB-SLAM3 integrates into fake motion."""
    global _synth_prev_pos, _synth_prev_vel_world, _synth_prev_quat, _synth_prev_time, _synth_prev_omega
    from scipy.spatial.transform import Rotation as _R

    pos = np.asarray(pos_arg, dtype=float)
    quat = np.asarray(quat_xyzw, dtype=float)

    if _synth_prev_time is None:
        _synth_prev_pos = pos
        _synth_prev_vel_world = np.zeros(3)
        _synth_prev_quat = quat
        _synth_prev_time = t
        _synth_prev_omega = np.zeros(3)
        _synth_pos_hist.append(pos.copy())
        return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

    dt = t - _synth_prev_time
    if dt < 1e-6:
        return None

    R_now = _R.from_quat(quat)
    R_prev = _R.from_quat(_synth_prev_quat)
    omega_body = (R_prev.inv() * R_now).as_rotvec() / dt
    omega_body = 0.4 * omega_body + 0.6 * _synth_prev_omega

    _synth_pos_hist.append(pos.copy())
    is_stationary = False
    if len(_synth_pos_hist) >= _synth_pos_hist.maxlen:
        hist = np.asarray(_synth_pos_hist)
        max_disp = float(np.max(np.linalg.norm(hist - hist[0], axis=1)))
        is_stationary = max_disp < _SYNTH_STAND_THRESH

    vel_world = (pos - _synth_prev_pos) / dt
    raw_accel_world = (vel_world - _synth_prev_vel_world) / dt
    _synth_accel_buf.append(raw_accel_world)
    smooth_accel = np.mean(list(_synth_accel_buf), axis=0)

    if is_stationary:
        accel_body = R_now.inv().apply(np.array([0., 0., 9.81]))
        accel_body = accel_body + rng.normal(0, _SYNTH_ACCEL_STD, 3) + _SYNTH_ACCEL_BIAS
    else:
        accel_world_g = smooth_accel + np.array([0., 0., 9.81])
        accel_body = R_now.inv().apply(accel_world_g)
        # exp 70 Run A: consistent Phidgets 1042 noise + bias in motion too
        # (master omitted this - ORB-SLAM3 BA weights NoiseAcc from yaml;
        # mismatch between regimes is a likely source of straight-line scale drift)
        accel_body = accel_body + rng.normal(0, _SYNTH_ACCEL_STD, 3) + _SYNTH_ACCEL_BIAS

    omega_noisy = omega_body + rng.normal(0, _SYNTH_GYRO_STD, 3) + _SYNTH_GYRO_BIAS

    _synth_prev_pos = pos
    _synth_prev_vel_world = vel_world
    _synth_prev_quat = quat
    _synth_prev_time = t
    _synth_prev_omega = omega_body

    return (float(accel_body[0]), float(accel_body[1]), float(accel_body[2]),
            float(omega_noisy[0]), float(omega_noisy[1]), float(omega_noisy[2]))
_rec_img_count = 0
print(f"  SLAM recording to {_rec_dir}")

try:
    # NOTE: zigzag init phase removed (exp 21 finding):
    # - DriveAPI commands didn't actually move the robot (forest uses ArticulationAPI)
    # - Result: 12s of "fake" zigzag where IMU recorded ±2 m/s² wheel vibrations
    #   while GT showed robot static. ORB-SLAM3 built initial map from these
    #   stationary keyframes -> bad triangulation -> +0.13m to +6m extra ATE.
    # - Init phase only helps VIO (which doesn't work on forest anyway).
    # Recording starts driving the route immediately.

    # With render at 200fps, each app.update() = 1 physics step = 1/200s
    # IMU: every step (200Hz), Camera: every 20th step (10Hz), GT/odom: every step
    _step_dt = 1.0 / 200.0
    _step_count = 0

    while sim_time < args.duration:
        app.update()
        sim_time += _step_dt
        _step_count += 1

        # IMU every step (200Hz)
        if _recording:
            pp = _get_husky_pose()
            _qw = float(pp[1][0]); _qx = float(pp[1][1])
            _qy = float(pp[1][2]); _qz = float(pp[1][3])
            _valid = False
            if args.synthetic_imu:
                _pos = np.array([float(pp[0][0]), float(pp[0][1]), float(pp[0][2])])
                _yaw_q = np.array([float(pp[1][1]), float(pp[1][2]), float(pp[1][3]), float(pp[1][0])])  # scipy (x,y,z,w)
                # World-frame velocity from position diff (proven accurate in odom)
                _synth = _compute_synth_imu(_pos, _yaw_q, _pos, sim_time)  # pass pos as 3rd arg, compute vel internally
                if _synth is not None:
                    _ax, _ay, _az, _gx, _gy, _gz = _synth
                    _valid = True
            else:
                _imu_reading = _imu_interface.get_sensor_reading(IMU_SENSOR_PATH, read_gravity=True)
                _valid = _imu_reading.is_valid
                if _valid:
                    _ax, _ay, _az = _imu_urf_to_flu(
                        _imu_reading.lin_acc_x, _imu_reading.lin_acc_y, _imu_reading.lin_acc_z)
                    _gx, _gy, _gz = _imu_urf_to_flu(
                        _imu_reading.ang_vel_x, _imu_reading.ang_vel_y, _imu_reading.ang_vel_z)
            if _valid:
                _imu_file.write(f"{sim_time:.4f},"
                    f"{_ax:.6f},{_ay:.6f},{_az:.6f},"
                    f"{_gx:.6f},{_gy:.6f},{_gz:.6f},"
                    f"{_qx:.6f},{_qy:.6f},{_qz:.6f},{_qw:.6f}\n")
                # Write IMU for live VIO (rgbd_inertial_live) - append with timestamp
                # Format: timestamp gx gy gz ax ay az qx qy qz qw
                with open("/tmp/isaac_imu.txt", "a") as _imuf:
                    _imuf.write(f"{sim_time:.4f} {_gx:.6f} {_gy:.6f} {_gz:.6f} "
                                f"{_ax:.6f} {_ay:.6f} {_az:.6f} "
                                f"{_qx:.6f} {_qy:.6f} {_qz:.6f} {_qw:.6f}\n")

        # Navigation + camera + GT only every 20th step (10Hz)
        if _step_count % 20 != 0:
            continue

        rclpy.spin_once(node, timeout_sec=0.001)

        # check signal file to remove obstacles at runtime
        if os.path.exists("/tmp/isaac_remove_obstacles.txt"):
            try:
                from spawn_obstacles import remove_obstacles
                remove_obstacles(stage)
                os.remove("/tmp/isaac_remove_obstacles.txt")
                print("  OBSTACLES REMOVED via signal file")
            except Exception as e:
                print(f"  obstacle removal failed: {e}")

        # Signal: clear pure-pursuit route (used when Nav2 takes over)
        if _auto_route is not None and os.path.exists("/tmp/isaac_clear_route.txt"):
            _auto_route = None
            goal_x, goal_y = None, None
            try: os.remove("/tmp/isaac_clear_route.txt")
            except: pass
            print("  PURE PURSUIT DISABLED - awaiting /cmd_vel")
            _get_husky_pose._pp_disabled = True

        # auto-route: advance to next waypoint when arrived
        if _auto_route is not None and goal_x is None and _auto_idx < len(_auto_route):
            # Pure pursuit lookahead: pick WP ~2m ahead along path (not just next by index)
            pp_ = _get_husky_pose()[0]
            _rx0, _ry0 = float(pp_[0]), float(pp_[1])
            LOOKAHEAD = 2.0
            _best = _auto_idx
            for _i in range(_auto_idx, min(_auto_idx + 10, len(_auto_route))):
                _wx, _wy = _auto_route[_i]
                if math.hypot(_wx - _rx0, _wy - _ry0) >= LOOKAHEAD:
                    _best = _i
                    break
                _best = _i
            _auto_idx = _best
            goal_x, goal_y = _auto_route[_auto_idx]
            print(f"  ROUTE [{_auto_idx}/{len(_auto_route)-1}]: ({goal_x:.1f}, {goal_y:.1f})")
        elif _auto_route is not None and goal_x is None and _auto_idx >= len(_auto_route):
            print(f"\n  ROUTE COMPLETE! Recorded {_rec_img_count} frames.")
            break  # stop simulation when route is done

        # read goal from file (written by web UI) - only when no auto-route
        # SKIP in nav2-bridge mode: Nav2 controls via /cmd_vel only
        if _auto_route is None and not getattr(args, 'nav2_bridge', False):
            try:
                with open("/tmp/isaac_goal.txt", "r") as f:
                    goal_txt = f.read().strip()
                if goal_txt == "stop":
                    goal_x, goal_y = None, None
                elif goal_txt == "reverse":
                    # drive backwards for 3s via DriveAPI wheel velocities
                    goal_x, goal_y = None, None
                    rev_speed = math.degrees(-5.0)  # rad/s backwards
                    for _ in range(180):  # 3 seconds at 60Hz
                        for wa in _wheel_vel_attrs:
                            wa.Set(rev_speed)
                        app.update()
                        pp = _get_husky_pose()
                        _ry = math.atan2(2*(float(pp[1][0])*float(pp[1][3])), 1-2*(float(pp[1][3])**2))
                        _cam_transform_op.Set(_make_cam_matrix(
                            float(pp[0][0]) + CAM_FWD*math.cos(_ry),
                            float(pp[0][1]) + CAM_FWD*math.sin(_ry),
                            float(pp[0][2]) + CAM_UP, _ry))
                    for wa in _wheel_vel_attrs:
                        wa.Set(0.0)
                    try:
                        os.remove("/tmp/isaac_goal.txt")
                    except:
                        pass
                    # print(f"DEBUG wp_idx={wp_idx} pose={pose}")
                    print("  REVERSED ~3s")
                elif goal_txt == "reset":
                    goal_x, goal_y = None, None
                    # teleport robot back to spawn (set root xform + stop wheels)
                    _husky_translate_op.Set(Gf.Vec3d(-95, -6, _spawn_z))
                    _husky_rotate_op.Set(Gf.Vec3f(0, 0, 0))
                    for _wa in _wheel_vel_attrs:
                        _wa.Set(0.0)
                    try:
                        os.remove("/tmp/isaac_goal.txt")
                    except:
                        pass
                    print("  RESET to spawn (-95, -6)")
                elif goal_txt:
                    parts = goal_txt.split()
                    new_x, new_y = float(parts[0]), float(parts[1])
                    if goal_x is None or math.hypot(new_x - goal_x, new_y - goal_y) > 0.5:
                        print(f"  NEW GOAL: ({new_x:.1f}, {new_y:.1f})")
                    goal_x, goal_y = new_x, new_y
            except FileNotFoundError:
                pass
            except:
                pass

        # also accept cmd_vel from ROS2
        cmd = latest_cmd[0]

        if goal_x is not None:
            pp = _get_husky_pose()[0]
            rx, ry = float(pp[0]), float(pp[1])
            dx, dy = goal_x - rx, goal_y - ry
            dist = math.hypot(dx, dy)

            if dist < 1.0:  # tight tracking with dense 0.5m WPs
                vels = np.array([0.0, 0.0, 0.0, 0.0])
                print(f"  ARRIVED at ({goal_x:.1f}, {goal_y:.1f}), actual=({rx:.1f},{ry:.1f})")
                goal_x, goal_y = None, None
                _get_husky_pose._stuck_ref = (rx, ry, sim_time)
                if _auto_route is not None:
                    _auto_idx += 1
                else:
                    try:
                        os.remove("/tmp/isaac_goal.txt")
                    except:
                        pass
            else:
                desired_yaw = math.atan2(dy, dx)
                rot = _get_husky_pose()[1]
                qw, qx, qy, qz = float(rot[0]), float(rot[1]), float(rot[2]), float(rot[3])
                current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                err = desired_yaw - current_yaw
                while err > math.pi: err -= 2 * math.pi
                while err < -math.pi: err += 2 * math.pi

                # Pure pursuit speed - scaled for VIO camera matching
                # cmd 0.25 × Husky 3.4× scaling = ~0.85 m/s actual
                max_speed = 0.25
                if abs(err) > 0.5:
                    # Large error: slow and sharp turn
                    lin_v, ang_v = 0.10, max(-0.5, min(0.5, err * 1.8))
                elif abs(err) > 0.15:
                    # Medium error: moderate speed
                    lin_v, ang_v = 0.18, max(-0.35, min(0.35, err * 1.5))
                else:
                    lin_v, ang_v = max_speed, max(-0.2, min(0.2, err * 1.2))

                v_left = (lin_v - ang_v * track / 2) / wheel_r
                v_right = (lin_v + ang_v * track / 2) / wheel_r
                vels = np.array([v_left, v_right, v_left, v_right])
        elif abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01:
            v_left = (cmd.linear.x - cmd.angular.z * track / 2) / wheel_r
            v_right = (cmd.linear.x + cmd.angular.z * track / 2) / wheel_r
            vels = np.array([v_left, v_right, v_left, v_right])
            # Log Nav2 commanding (every 2s sim time)
            if not hasattr(_get_husky_pose, '_last_nav2_log') or sim_time - _get_husky_pose._last_nav2_log > 2.0:
                print(f"  [NAV2] v={cmd.linear.x:.2f} w={cmd.angular.z:.2f}")
                _get_husky_pose._last_nav2_log = sim_time
        else:
            vels = np.array([0.0, 0.0, 0.0, 0.0])

        # set wheel velocity targets via USD DriveAPI (no articulation)
        for i, _wa in enumerate(_wheel_vel_attrs):
            v = float(vels[0]) if i % 2 == 0 else float(vels[1])  # left=0,2 right=1,3
            _wa.Set(math.degrees(v))

        # read pose, sync camera
        pp = _get_husky_pose()
        rx, ry, rz = float(pp[0][0]), float(pp[0][1]), float(pp[0][2])
        qw, qx, qy, qz = float(pp[1][0]), float(pp[1][1]), float(pp[1][2]), float(pp[1][3])
        current_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        # Write GT pose for tf_wall_clock_relay
        with open("/tmp/isaac_pose.txt", "w") as _pf:
            _pf.write(f"{rx} {ry} {rz} {qx} {qy} {qz} {qw}\n")
        # Dense trajectory log (every 10Hz navigation tick)
        if not hasattr(_get_husky_pose, '_traj_f'):
            _get_husky_pose._traj_f = open("/tmp/isaac_trajectory.csv", "w")
            _get_husky_pose._traj_f.write("t,x,y,yaw\n")
        _get_husky_pose._traj_f.write(f"{sim_time:.2f},{rx:.4f},{ry:.4f},{current_yaw:.4f}\n")
        _get_husky_pose._traj_f.flush()
        # sync camera with robot
        cam_x = rx + CAM_FWD * math.cos(current_yaw)
        cam_y = ry + CAM_FWD * math.sin(current_yaw)
        cam_z = rz + CAM_UP
        fwd_d = 0.5
        z_front = _terrain_height(rx + fwd_d * math.cos(current_yaw), ry + fwd_d * math.sin(current_yaw))
        z_back = _terrain_height(rx - fwd_d * math.cos(current_yaw), ry - fwd_d * math.sin(current_yaw))
        pitch = math.atan2(z_front - z_back, 2 * fwd_d)
        _cam_transform_op.Set(_make_cam_matrix(cam_x, cam_y, cam_z, current_yaw, pitch))

        # save camera every logical frame (already 10Hz from step_count % 20)
        frame_n = _step_count // 20
        if True:  # every logical frame = 10Hz
            try:
                d_fwd = ann_fwd.get_data() if 'ann_fwd' in dir() else None
                if d_fwd is not None and np.mean(d_fwd[:, :, :3]) > 1:
                    from PIL import Image as PILImg
                    img = PILImg.fromarray(d_fwd[:, :, :3]).resize((640, 480))
                    try:
                        img.save("/tmp/isaac_cam_fwd.tmp.jpg", quality=75)
                        os.replace("/tmp/isaac_cam_fwd.tmp.jpg", "/tmp/isaac_cam_fwd.jpg")
                    except (FileNotFoundError, OSError):
                        pass
                    # save to recording dir
                    if _recording:
                        img.save(f"{_rec_dir}/camera_rgb/{sim_time:.4f}.jpg", quality=90)
                        d_depth = ann_depth.get_data() if 'ann_depth' in dir() else None
                        if d_depth is not None:
                            dc = np.nan_to_num(d_depth, nan=0.0, posinf=0.0, neginf=0.0)
                            depth_mm = (dc * 1000).astype(np.uint16)
                            PILImg.fromarray(depth_mm).save(f"{_rec_dir}/camera_depth/{sim_time:.4f}.png")
                        _rec_img_count += 1
            except Exception:
                pass
        # position for web UI
        if frame_n % 10 == 0:
            pp = _get_husky_pose()[0]
            try:
                with open("/tmp/isaac_robot_pos.txt", "w") as f:
                    f.write(f"{float(pp[0]):.3f} {float(pp[1]):.3f} {float(pp[2]):.3f}")
            except:
                pass

        # GT + odometry + IMU recording
        if _recording and frame_n % 1 == 0:
            pp = _get_husky_pose()
            rx, ry, rz = float(pp[0][0]), float(pp[0][1]), float(pp[0][2])
            qw, qx, qy, qz = float(pp[1][0]), float(pp[1][1]), float(pp[1][2]), float(pp[1][3])
            cur_yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
            ts = sim_time

            # ground truth
            _gt_file.write(f"{ts:.4f},{rx:.6f},{ry:.6f},{rz:.6f},{cur_yaw:.6f}\n")
            cam_gt_x = rx + CAM_FWD * math.cos(cur_yaw)
            cam_gt_y = ry + CAM_FWD * math.sin(cur_yaw)
            cam_gt_z = rz + CAM_UP
            _gt_tum_file.write(f"{ts:.4f} {cam_gt_x:.6f} {cam_gt_y:.6f} {cam_gt_z:.6f} 0.000000 0.000000 {math.sin(cur_yaw/2):.6f} {math.cos(cur_yaw/2):.6f}\n")

            # wheel odometry
            dt = _step_dt * 20
            if hasattr(_get_husky_pose, '_prev'):
                px, py, pyaw = _get_husky_pose._prev
                vx = (rx - px) / dt
                vy = (ry - py) / dt
                vyaw = cur_yaw - pyaw
                if vyaw > math.pi: vyaw -= 2 * math.pi
                if vyaw < -math.pi: vyaw += 2 * math.pi
                vyaw /= dt
                vx += rng.uniform(-0.01, 0.01)
                vy += rng.uniform(-0.01, 0.01)
                vyaw += rng.uniform(-0.002, 0.002)
                lin_v = math.hypot(vx, vy)
                _odom_file.write(f"{ts:.4f},{rx:.6f},{ry:.6f},{rz:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{lin_v:.6f},{vyaw:.6f}\n")
            _get_husky_pose._prev = (rx, ry, cur_yaw)

        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < 0.02:
            pp = _get_husky_pose()[0]
            print(f"  t={sim_time:.0f}s robot=({pp[0]:.1f}, {pp[1]:.1f}, {pp[2]:.2f})")

except KeyboardInterrupt:
    print("\nstopped")

if _recording:
    _gt_file.close()
    _gt_tum_file.close()
    _odom_file.close()
    _imu_file.close()
    print(f"\nSLAM recording saved to {_rec_dir}")
    print(f"  {_rec_img_count} images, GT+odom+IMU at 60Hz")

node.destroy_node()
rclpy.shutdown()
timeline.stop()
app.close()
print("done")