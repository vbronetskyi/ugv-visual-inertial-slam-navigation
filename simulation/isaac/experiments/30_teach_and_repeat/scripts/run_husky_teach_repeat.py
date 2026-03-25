#!/usr/bin/env python3
"""
Teach-and-repeat navigation for Husky in Isaac Sim.
Phase 3: route following without Nav2.

Pipeline:
  Isaac camera -> anchor_localizer -> route_follower -> wheel velocities

usage:
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /opt/isaac-sim-6.0.0/python.sh scripts/run_husky_teach_repeat.py --route south --direction outbound --duration 600
"""
import os
import sys
import math
import time
import json
import argparse

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--route", type=str, default="south")
parser.add_argument("--direction", type=str, default="outbound",
                    choices=["outbound", "return"])
parser.add_argument("--duration", type=float, default=600.0)
parser.add_argument("--lookahead", type=float, default=8.0)
args, _ = parser.parse_known_args()

# =====================================================================
# Isaac Sim setup (from run_husky_forest.py)
# =====================================================================
from isaacsim import SimulationApp
app = SimulationApp({
    "headless": True,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 1,
    "max_volume_bounces": 1,
    "samples_per_pixel_per_frame": 48,
    "width": 640,
    "height": 480,
})

import omni
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import usdrt.Sdf
import carb
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

settings = carb.settings.get_settings()
settings.set("/rtx/raytracing/backfaceCulling", False)
settings.set("/rtx/directLighting/backfaceCulling", False)
settings.set("/rtx/post/motionblur/enabled", False)
settings.set("/rtx/post/dof/enabled", False)
settings.set("/rtx/post/bloom/enabled", False)
settings.set("/rtx/post/lensFlares/enabled", False)
settings.set("/rtx/directLighting/sampledLighting/enabled", False)
settings.set("/rtx/reflections/enabled", False)
settings.set("/rtx/indirectDiffuse/enabled", True)
settings.set("/persistent/omnigraph/updateToUsd", True)
settings.set("/persistent/omnihydra/useSceneGraphInstancing", False)

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

print(f"loading forest scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

print("adding Husky A200...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(300):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"
CAM_RGB = "/World/HuskyCamera"
CAM_FWD = 0.5
CAM_UP = 0.48

_phys_scene = stage.GetPrimAtPath("/World/PhysicsScene")
if _phys_scene.IsValid():
    PhysxSchema.PhysxSceneAPI(_phys_scene).GetTimeStepsPerSecondAttr().Set(200)
    PhysxSchema.PhysxSceneAPI(_phys_scene).CreateSolverTypeAttr().Set("TGS")

cam = UsdGeom.Camera.Define(stage, CAM_RGB)
cam.CreateFocalLengthAttr(1.93)
cam.CreateHorizontalApertureAttr(3.86)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
_cam_xf = UsdGeom.Xformable(cam)
_cam_transform_op = _cam_xf.AddTransformOp()

def _make_cam_matrix(x, y, z, yaw, pitch=0):
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    r0x, r0y, r0z = sy, -cy, 0
    r1x, r1y, r1z = (-cy) * sp, (-sy) * sp, cp
    r2x, r2y, r2z = (-cy) * cp, (-sy) * cp, -sp
    return Gf.Matrix4d(r0x, r0y, r0z, 0, r1x, r1y, r1z, 0,
                       r2x, r2y, r2z, 0, x, y, z, 1)

# Wheel friction
from pxr import UsdShade
_wf_mat = UsdShade.Material.Define(stage, "/World/WheelFriction")
_wf_phys = UsdPhysics.MaterialAPI.Apply(_wf_mat.GetPrim())
_wf_phys.CreateStaticFrictionAttr(1.0)
_wf_phys.CreateDynamicFrictionAttr(0.8)
_wf_phys.CreateRestitutionAttr(0.0)
for wl in ["front_left_wheel_link", "front_right_wheel_link",
           "rear_left_wheel_link", "rear_right_wheel_link"]:
    _col = stage.GetPrimAtPath(f"{BASE_LINK}/{wl}/collision")
    if _col.IsValid():
        UsdShade.MaterialBindingAPI.Apply(_col).Bind(_wf_mat, materialPurpose="physics")

# Wheel drives
_wheel_vel_attrs = []
for wname in ["front_left_wheel", "front_right_wheel",
              "rear_left_wheel", "rear_right_wheel"]:
    wp = f"/World/Husky/Physics/{wname}"
    drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(wp), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())

# Robot pose
_base_link_prim = stage.GetPrimAtPath(BASE_LINK)

def _get_husky_pose():
    xf_cache = UsdGeom.XformCache()
    world_tf = xf_cache.GetLocalToWorldTransform(_base_link_prim)
    pos = world_tf.ExtractTranslation()
    rot = world_tf.ExtractRotationMatrix()
    yaw = math.atan2(rot[0][1], rot[0][0])
    return float(pos[0]), float(pos[1]), float(pos[2]), yaw

# Spawn
ROUTE_MEMORY = f"/workspace/simulation/isaac/route_memory/{args.route}"
with open(f"{ROUTE_MEMORY}/anchors.json") as f:
    all_anchors = json.load(f)

if args.direction == "outbound":
    max_x_idx = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
    nav_anchors = all_anchors[: max_x_idx + 1]
else:
    max_x_idx = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
    nav_anchors = all_anchors[max_x_idx:]

spawn_x = nav_anchors[0]["x"]
spawn_y = nav_anchors[0]["y"]
spawn_yaw = nav_anchors[0]["yaw"]

# Terrain height (from run_husky_forest.py)
_RWPS = [
    (-100, -7), (-95, -6), (-90, -4.5), (-85, -2.8), (-80, -1.5),
    (-75, -0.8), (-70, -0.5), (-65, -1), (-60, -2.2), (-55, -3.8),
    (-50, -5), (-45, -5.5), (-40, -5.2), (-35, -4), (-30, -2.5),
    (-25, -1), (-20, 0.2), (-15, 1.2), (-10, 1.8), (-5, 2),
    (0, 1.5), (5, 0.5), (10, -0.8), (15, -2.2), (20, -3.5),
    (25, -4.2), (30, -4), (35, -3), (40, -1.8), (45, -0.8),
    (50, -0.5), (55, -1), (60, -2), (65, -3.2), (70, -4.5), (75, -5),
]

def _road_y(x):
    if x <= _RWPS[0][0]:
        return _RWPS[0][1]
    if x >= _RWPS[-1][0]:
        return _RWPS[-1][1]
    for i in range(len(_RWPS) - 1):
        if _RWPS[i][0] <= x <= _RWPS[i + 1][0]:
            t = (x - _RWPS[i][0]) / (_RWPS[i + 1][0] - _RWPS[i][0])
            return _RWPS[i][1] + t * (_RWPS[i + 1][1] - _RWPS[i][1])
    return 0

def _terrain_height(x, y):
    h = 0.0
    h += 0.5 * math.sin(x * 0.018 + 0.5) * math.cos(y * 0.022 + 1.2)
    h += 0.35 * math.sin(x * 0.035 + 2.1) * math.sin(y * 0.03 + 0.7)
    h += 0.18 * math.sin(x * 0.07 + 3.3) * math.cos(y * 0.065 + 2.5)
    h += 0.12 * math.cos(x * 0.11 + 1.0) * math.sin(y * 0.09 + 4.0)
    h += 0.06 * math.sin(x * 0.5 + 0.7) * math.cos(y * 0.43 + 2.1)
    h += 0.04 * math.cos(x * 0.7 + 3.5) * math.sin(y * 0.6 + 0.4)
    h += 0.03 * math.sin(x * 1.0 + 1.2) * math.cos(y * 0.83 + 3.8)
    road_dist = abs(y - _road_y(x))
    if road_dist < 4.0:
        h *= (road_dist / 4.0) ** 2
    if road_dist < 2.0:
        h -= 0.06 * (1.0 - road_dist / 2.0)
    return max(h, -0.5)

spawn_z = _terrain_height(spawn_x, spawn_y) + 0.5
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_husky_translate_op = husky_xf.AddTranslateOp()
_husky_rotate_op = husky_xf.AddRotateXYZOp()
_husky_translate_op.Set(Gf.Vec3d(spawn_x, spawn_y, spawn_z))
_husky_rotate_op.Set(Gf.Vec3f(0, 0, math.degrees(spawn_yaw)))
_cam_transform_op.Set(_make_cam_matrix(
    spawn_x + CAM_FWD * math.cos(spawn_yaw),
    spawn_y + CAM_FWD * math.sin(spawn_yaw),
    spawn_z + CAM_UP, spawn_yaw))
print(f"  spawn: ({spawn_x:.1f}, {spawn_y:.1f}, {spawn_z:.2f}) yaw={math.degrees(spawn_yaw):.0f}")

# Start simulation
print(f"\nstarting simulation ({args.duration}s)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

# Camera render product
import omni.replicator.core as rep
rp_fwd = rep.create.render_product(CAM_RGB, (640, 480))
ann_fwd = rep.AnnotatorRegistry.get_annotator("rgb")
ann_fwd.attach([rp_fwd])
for _ in range(200):
    app.update()
print("  camera ready")

p0 = _get_husky_pose()
print(f"  robot at ({p0[0]:.1f}, {p0[1]:.1f}, {p0[2]:.2f}) yaw={math.degrees(p0[3]):.0f}")

# =====================================================================
# Teach-and-repeat pipeline
# =====================================================================
from anchor_localizer import AnchorLocalizer
from route_follower import RouteFollower

localizer = AnchorLocalizer(ROUTE_MEMORY, direction=args.direction)
follower = RouteFollower(nav_anchors, lookahead_distance=args.lookahead)
follower.initialize_from_anchor(0)

WHEEL_R = 0.165
TRACK = 0.555
LOCALIZE_EVERY = 6  # every 6 physics frames = ~10Hz camera / 6 = ~1.7 Hz matching

sim_time = 5.0
dt = 1.0 / 60.0
frame_count = 0
prev_lin = 0.0
prev_ang = 0.0

log_path = f"/workspace/simulation/isaac/experiments/30_teach_and_repeat/logs/" \
           f"teach_repeat_{args.route}_{args.direction}_{int(time.time())}.csv"
log_f = open(log_path, "w")
log_f.write("time,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,"
            "anchor_id,confidence,cmd_lin,cmd_ang,progress_m\n")

print(f"\n{'='*60}")
print(f"TEACH-AND-REPEAT: route={args.route} direction={args.direction}")
print(f"  anchors: {len(nav_anchors)}")
print(f"  lookahead: {args.lookahead}m")
print(f"  log: {log_path}")
print(f"{'='*60}\n")

try:
    while sim_time < args.duration and not follower.finished:
        app.update()
        sim_time += dt
        frame_count += 1

        # 1. Odometry dead-reckoning
        follower.update_odometry(prev_lin, prev_ang, dt)

        # 2. Get camera frame + localize (every N frames)
        anchor_id = localizer.current_anchor_id
        confidence = localizer.confidence
        if frame_count % LOCALIZE_EVERY == 0:
            try:
                d_fwd = ann_fwd.get_data()
                if d_fwd is not None and d_fwd.shape[0] > 10:
                    rgb = d_fwd[:, :, :3].copy()
                    import cv2
                    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
                    anchor_id, confidence, _ = localizer.localize(gray)
                    follower.correct_from_anchor(anchor_id, confidence)
            except Exception:
                pass

        # 3. Compute cmd_vel
        cmd_lin, cmd_ang = follower.compute_cmd_vel()

        # 4. Send to wheels (differential drive)
        v_left = (cmd_lin - cmd_ang * TRACK / 2) / WHEEL_R
        v_right = (cmd_lin + cmd_ang * TRACK / 2) / WHEEL_R
        # rad/s to deg/s for USD DriveAPI
        for i, vel in enumerate([v_left, v_right, v_left, v_right]):
            _wheel_vel_attrs[i].Set(math.degrees(vel))

        prev_lin = cmd_lin
        prev_ang = cmd_ang

        # 5. Update camera position
        rx, ry, rz, ryaw = _get_husky_pose()
        fwd_d = 0.5
        z_front = _terrain_height(rx + fwd_d * math.cos(ryaw),
                                  ry + fwd_d * math.sin(ryaw))
        z_back = _terrain_height(rx - fwd_d * math.cos(ryaw),
                                 ry - fwd_d * math.sin(ryaw))
        pitch = math.atan2(z_front - z_back, 2 * fwd_d)
        cam_x = rx + CAM_FWD * math.cos(ryaw)
        cam_y = ry + CAM_FWD * math.sin(ryaw)
        cam_z = rz + CAM_UP
        _cam_transform_op.Set(_make_cam_matrix(cam_x, cam_y, cam_z, ryaw, pitch))

        # 6. Log every second
        if frame_count % 60 == 0:
            stats = follower.get_stats()
            log_f.write(
                f"{sim_time:.2f},{rx:.4f},{ry:.4f},{ryaw:.4f},"
                f"{follower.odom_x:.4f},{follower.odom_y:.4f},"
                f"{follower.odom_yaw:.4f},"
                f"{anchor_id},{confidence:.3f},"
                f"{cmd_lin:.3f},{cmd_ang:.3f},"
                f"{stats['progress_m']:.2f}\n"
            )
            log_f.flush()

        # 7. Print every 10s
        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < dt:
            stats = follower.get_stats()
            print(
                f"  t={sim_time:.0f}s | GT=({rx:.1f},{ry:.1f}) | "
                f"anchor={stats['anchor_idx']}/{len(nav_anchors)-1} | "
                f"conf={confidence:.2f} | "
                f"progress={stats['progress_m']:.0f}/{stats['total_m']:.0f}m | "
                f"cmd=({cmd_lin:.2f},{cmd_ang:.2f})"
            )

except KeyboardInterrupt:
    print("\nstopped by user")

log_f.close()

# Final report
rx, ry, rz, ryaw = _get_husky_pose()
stats = follower.get_stats()
print(f"\n{'='*60}")
print(f"RESULT:")
print(f"  Duration: {sim_time:.0f}s")
print(f"  Final GT: ({rx:.1f}, {ry:.1f})")
print(f"  Anchor: {stats['anchor_idx']}/{len(nav_anchors)-1}")
print(f"  Progress: {stats['progress_m']:.0f}/{stats['total_m']:.0f}m")
print(f"  Finished: {stats['finished']}")
print(f"  Log: {log_path}")
print(f"{'='*60}")

timeline.stop()
app.close()
