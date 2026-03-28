#!/usr/bin/env python3
"""
Teach-and-repeat in a single Isaac Sim session.
Phase 1 (teach): drive via GT waypoints, record anchors in-memory.
Phase 2 (repeat): teleport to start, drive via visual anchor matching.

Same rendering session -> ORB features should match well.

usage:
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /opt/isaac-sim-6.0.0/python.sh scripts/run_husky_teach_then_repeat.py --route south --duration 800
"""
import os
import sys
import math
import time
import json
import argparse

import numpy as np
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("--route", type=str, default="south")
parser.add_argument("--direction", type=str, default="outbound",
                    choices=["outbound", "return"])
parser.add_argument("--duration", type=float, default=900.0,
                    help="total sim time (teach + repeat)")
parser.add_argument("--lookahead", type=float, default=8.0)
parser.add_argument("--skip-teach", action="store_true",
                    help="skip teach phase, use existing anchors from route_memory")
parser.add_argument("--obstacles", action="store_true",
                    help="spawn cones+tent obstacles on route")
parser.add_argument("--use-grid", action="store_true",
                    help="use LocalObstacleGrid for accumulated detection")
parser.add_argument("--use-planner", action="store_true",
                    help="use A* grid planner (implies --use-grid)")
args, _ = parser.parse_known_args()
if args.use_planner:
    args.use_grid = True  # planner needs the grid

# =====================================================================
# Isaac Sim setup (identical to run_husky_teach_repeat.py)
# =====================================================================
from isaacsim import SimulationApp
app = SimulationApp({
    "headless": True,
    "max_bounces": 4,
    "max_specular_transmission_bounces": 1,
    "max_volume_bounces": 1,
    "samples_per_pixel_per_frame": 8,  # reduced for speed (was 48)
    "width": 640,
    "height": 480,
})

import omni
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import usdrt.Sdf
import carb
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf

settings = carb.settings.get_settings()
for k, v in [
    ("/rtx/raytracing/backfaceCulling", False),
    ("/rtx/directLighting/backfaceCulling", False),
    ("/rtx/post/motionblur/enabled", False),
    ("/rtx/post/dof/enabled", False),
    ("/rtx/post/bloom/enabled", False),
    ("/rtx/post/lensFlares/enabled", False),
    ("/rtx/directLighting/sampledLighting/enabled", False),
    ("/rtx/reflections/enabled", False),
    ("/rtx/indirectDiffuse/enabled", True),
    ("/persistent/omnigraph/updateToUsd", True),
    ("/persistent/omnihydra/useSceneGraphInstancing", False),
]:
    settings.set(k, v)

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

print("loading forest scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

# Scene already pre-built by convert_gazebo_to_isaac.py with:
# - Trees thinned 33%, no ferns, reduced cover, debris instead of fallen
# - No runtime deactivation needed (was causing 6+ min delays)
print("scene pre-built (no runtime thinning needed)")

# Thin gazebo_models.json for collision list (match scene)
_gm = json.load(open("/tmp/gazebo_models_dense.json"))
_thinned = []
_tree_idx = 0
for m in _gm:
    if m["type"] in ("pine", "oak"):
        if _tree_idx % 3 != 0:  # keep 2/3 of trees
            _thinned.append(m)
        _tree_idx += 1
    elif m["type"] == "shrub":
        _thinned.append(m)  # keep all shrubs (short, not blocking)
    elif "fallen" in m.get("type", ""):
        pass  # remove fallen from collision (deactivated in USD)
    else:
        _thinned.append(m)  # keep rocks, houses, barrels
with open("/tmp/gazebo_models.json", "w") as f:
    json.dump(_thinned, f)
print(f"  collision models: {len(_gm)} -> {len(_thinned)}")

for _ in range(30):
    app.update()

if args.obstacles:
    import sys
    sys.path.insert(0, os.path.dirname(__file__))
    from spawn_obstacles import spawn_obstacles
    n_obs = spawn_obstacles(stage, args.route)
    print(f"  spawned {n_obs} obstacles on {args.route} route")
    for _ in range(30):
        app.update()

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
    return Gf.Matrix4d(
        sy, -cy, 0, 0,
        (-cy)*sp, (-sy)*sp, cp, 0,
        (-cy)*cp, (-sy)*cp, -sp, 0,
        x, y, z, 1,
    )

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

_wheel_vel_attrs = []
for wname in ["front_left_wheel", "front_right_wheel",
              "rear_left_wheel", "rear_right_wheel"]:
    drive = UsdPhysics.DriveAPI.Get(
        stage.GetPrimAtPath(f"/World/Husky/Physics/{wname}"), "angular")
    if drive:
        drive.GetDampingAttr().Set(100000.0)
        drive.GetStiffnessAttr().Set(0.0)
        drive.GetMaxForceAttr().Set(500.0)
        _wheel_vel_attrs.append(drive.GetTargetVelocityAttr())

# IMU sensor
IMU_PATH = "/World/Husky/Geometry/base_link/imu_link"
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
print(f"  IMU: {IMU_SENSOR_PATH}")

from isaacsim.sensors.physics import _sensor as _imu_mod
_imu_interface = _imu_mod.acquire_imu_sensor_interface()

def _imu_urf_to_flu(sx, sy, sz):
    return (-sy, -sz, sx)

_base_link_prim = stage.GetPrimAtPath(BASE_LINK)

def _get_husky_pose():
    xf_cache = UsdGeom.XformCache()
    world_tf = xf_cache.GetLocalToWorldTransform(_base_link_prim)
    pos = world_tf.ExtractTranslation()
    rot = world_tf.ExtractRotationMatrix()
    yaw = math.atan2(rot[0][1], rot[0][0])
    return float(pos[0]), float(pos[1]), float(pos[2]), yaw

# Load waypoints for teach phase
ROUTE_MEMORY = f"/workspace/simulation/isaac/route_memory/{args.route}"
with open(f"{ROUTE_MEMORY}/anchors.json") as f:
    file_anchors = json.load(f)
if args.direction == "outbound":
    max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
    teach_waypoints = [(a["x"], a["y"]) for a in file_anchors[: max_x_idx + 1]]
else:
    max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
    teach_waypoints = [(a["x"], a["y"]) for a in file_anchors[max_x_idx:]]

# Terrain height
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

def _terrain_height(x, y):
    h = (0.5*math.sin(x*0.018+0.5)*math.cos(y*0.022+1.2)
       + 0.35*math.sin(x*0.035+2.1)*math.sin(y*0.03+0.7)
       + 0.18*math.sin(x*0.07+3.3)*math.cos(y*0.065+2.5)
       + 0.12*math.cos(x*0.11+1.0)*math.sin(y*0.09+4.0)
       + 0.06*math.sin(x*0.5+0.7)*math.cos(y*0.43+2.1)
       + 0.04*math.cos(x*0.7+3.5)*math.sin(y*0.6+0.4)
       + 0.03*math.sin(x*1.0+1.2)*math.cos(y*0.83+3.8))
    rd = abs(y - _road_y(x))
    if rd < 4.0: h *= (rd/4.0)**2
    if rd < 2.0: h -= 0.06*(1.0-rd/2.0)
    return max(h, -0.5)

# Spawn at first waypoint
spawn_x, spawn_y = teach_waypoints[0]
spawn_yaw = math.atan2(teach_waypoints[1][1]-spawn_y,
                        teach_waypoints[1][0]-spawn_x)
spawn_z = _terrain_height(spawn_x, spawn_y) + 0.5

husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_husky_translate_op = husky_xf.AddTranslateOp()
_husky_rotate_op = husky_xf.AddRotateXYZOp()
_husky_translate_op.Set(Gf.Vec3d(spawn_x, spawn_y, spawn_z))
_husky_rotate_op.Set(Gf.Vec3f(0, 0, math.degrees(spawn_yaw)))
_cam_transform_op.Set(_make_cam_matrix(
    spawn_x + CAM_FWD*math.cos(spawn_yaw),
    spawn_y + CAM_FWD*math.sin(spawn_yaw),
    spawn_z + CAM_UP, spawn_yaw))
print(f"  spawn: ({spawn_x:.1f}, {spawn_y:.1f}) yaw={math.degrees(spawn_yaw):.0f}")

print(f"\nstarting simulation ({args.duration}s)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(300):
    app.update()

import omni.replicator.core as rep
rp_fwd = rep.create.render_product(CAM_RGB, (640, 480))
ann_fwd = rep.AnnotatorRegistry.get_annotator("rgb")
ann_fwd.attach([rp_fwd])
ann_depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
ann_depth.attach([rp_fwd])
for _ in range(200):
    app.update()
print("  camera + depth ready")

WHEEL_R = 0.165
TRACK = 0.555
dt = 1.0 / 60.0

def send_wheels(lin_v, ang_v):
    vl = (lin_v - ang_v * TRACK / 2) / WHEEL_R
    vr = (lin_v + ang_v * TRACK / 2) / WHEEL_R
    for i, vel in enumerate([vl, vr, vl, vr]):
        _wheel_vel_attrs[i].Set(math.degrees(vel))

def stop_wheels():
    for attr in _wheel_vel_attrs:
        attr.Set(0.0)

def update_camera():
    rx, ry, rz, ryaw = _get_husky_pose()
    fd = 0.5
    zf = _terrain_height(rx + fd*math.cos(ryaw), ry + fd*math.sin(ryaw))
    zb = _terrain_height(rx - fd*math.cos(ryaw), ry - fd*math.sin(ryaw))
    pitch = math.atan2(zf - zb, 2*fd)
    _cam_transform_op.Set(_make_cam_matrix(
        rx + CAM_FWD*math.cos(ryaw), ry + CAM_FWD*math.sin(ryaw),
        rz + CAM_UP, ryaw, pitch))
    return rx, ry, rz, ryaw

def get_rgb():
    try:
        d = ann_fwd.get_data()
        if d is not None and d.shape[0] > 10:
            return d[:, :, :3].copy()
    except Exception:
        pass
    return None

def get_depth():
    try:
        d = ann_depth.get_data()
        if d is not None and d.shape[0] > 10:
            return np.nan_to_num(d, nan=0.0, posinf=10.0, neginf=0.0)
    except Exception:
        pass
    return None


class DepthAvoidance:
    def __init__(self):
        self.OBSTACLE_DIST = 2.0
        self.CRITICAL_DIST = 1.0
        self.AVOIDANCE_STRENGTH = 0.5
        self.SLOWDOWN_DIST = 3.0
        H, W = 480, 640
        y1, y2 = H // 3, H * 2 // 3
        self.sectors = {
            "far_left":  (0, W // 5, y1, y2),
            "left":      (W // 5, W * 2 // 5, y1, y2),
            "center":    (W * 2 // 5, W * 3 // 5, y1, y2),
            "right":     (W * 3 // 5, W * 4 // 5, y1, y2),
            "far_right": (W * 4 // 5, W, y1, y2),
        }

    def sector_dist(self, depth, name):
        x1, x2, y1, y2 = self.sectors[name]
        s = depth[y1:y2, x1:x2]
        valid = s[(s > 0.1) & (s < 10.0)]
        if len(valid) < 10:
            return 10.0
        return float(np.percentile(valid, 20))

    def compute(self, depth, route_ang=0.0):
        """route_ang: angular cmd from pure pursuit (positive=left turn).
        Used to bias avoidance in the direction the route wants to go."""
        if depth is None:
            return 1.0, 0.0, False
        d = {n: self.sector_dist(depth, n) for n in self.sectors}

        # Never fully block - always creep forward (min 0.15 m/s via speed scale)
        if d["center"] < self.CRITICAL_DIST:
            speed = 0.15 / 0.8  # will be multiplied by MAX_LINEAR_VEL
        elif d["center"] < self.SLOWDOWN_DIST:
            speed = max(0.3, d["center"] / self.SLOWDOWN_DIST)
        else:
            speed = 1.0

        ang = 0.0
        for side, sign in [("left", -1), ("far_left", -1),
                           ("right", 1), ("far_right", 1)]:
            if d[side] < self.OBSTACLE_DIST:
                urgency = (self.OBSTACLE_DIST - d[side]) / self.OBSTACLE_DIST
                ang += sign * self.AVOIDANCE_STRENGTH * urgency

        if d["center"] < self.OBSTACLE_DIST:
            # Bias toward route direction when center blocked
            if abs(route_ang) > 0.05:
                ang += math.copysign(self.AVOIDANCE_STRENGTH * 0.8, route_ang)
            elif d["left"] + d["far_left"] > d["right"] + d["far_right"]:
                ang += self.AVOIDANCE_STRENGTH * 0.8
            else:
                ang -= self.AVOIDANCE_STRENGTH * 0.8

        return speed, ang, False


if args.skip_teach:
    print(f"\n{'='*60}")
    print(f"SKIP TEACH - using {len(file_anchors)} anchors from route_memory")
    print(f"{'='*60}")
    if args.direction == "outbound":
        max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
        anchors = file_anchors[: max_x_idx + 1]
    else:
        max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
        anchors = file_anchors[max_x_idx:]
    anchor_descs = {}
    anchor_depth_profiles = {}
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    orb = cv2.ORB_create(nfeatures=1000)
    for a in anchors:
        aid = a["id"]
        desc_path = f"{ROUTE_MEMORY}/rgb/{aid:04d}_desc.npy"
        dp_path = f"{ROUTE_MEMORY}/rgb/{aid:04d}_depth_profile.npy"
        if os.path.exists(desc_path):
            anchor_descs[aid] = np.load(desc_path)
        if os.path.exists(dp_path):
            anchor_depth_profiles[aid] = np.load(dp_path)
    print(f"  {len(anchors)} anchors, {len(anchor_descs)} desc, {len(anchor_depth_profiles)} dp")
    s_accum = anchors[-1]["s"] if anchors else 0

# =====================================================================
# PHASE 1: TEACH (skip if --skip-teach)
# =====================================================================
if not args.skip_teach:
    print(f"\n{'='*60}")
    print(f"PHASE 1: TEACH - driving {len(teach_waypoints)} waypoints")
    print(f"{'='*60}\n")

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
orb = cv2.ORB_create(nfeatures=1000)
sim_time = 5.0

anchors = []
anchor_descs = {}
anchor_depth_profiles = {}
last_ax = last_ay = None
s_accum = 0.0
prev_gx = prev_gy = None
wp_idx = 0
teach_start = sim_time

if args.skip_teach:
    # Load existing anchors from route_memory instead of driving teach
    if args.direction == "outbound":
        max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
        anchors = file_anchors[: max_x_idx + 1]
    else:
        max_x_idx = max(range(len(file_anchors)), key=lambda i: file_anchors[i]["x"])
        anchors = file_anchors[max_x_idx:]
    for a in anchors:
        aid = a["id"]
        dp = f"{ROUTE_MEMORY}/rgb/{aid:04d}_desc.npy"
        if os.path.exists(dp):
            anchor_descs[aid] = np.load(dp)
    s_accum = anchors[-1]["s"] if anchors else 0
    print(f"  Loaded {len(anchors)} anchors, {len(anchor_descs)} descriptors")
    wp_idx = len(teach_waypoints)  # skip while loop

while sim_time < args.duration / 2 and wp_idx < len(teach_waypoints):
    app.update()
    sim_time += dt

    rx, ry, rz, ryaw = update_camera()

    # Pure pursuit to current waypoint
    wx, wy = teach_waypoints[wp_idx]
    ddx, ddy = wx - rx, wy - ry
    dist = math.hypot(ddx, ddy)

    if dist < 2.5:
        wp_idx += 1
        if wp_idx >= len(teach_waypoints):
            break
        continue

    target_yaw = math.atan2(ddy, ddx)
    herr = target_yaw - ryaw
    while herr > math.pi: herr -= 2*math.pi
    while herr < -math.pi: herr += 2*math.pi

    if abs(herr) > 0.7:
        lin_v, ang_v = 0.4, max(-1.2, min(1.2, herr*2.0))
    elif abs(herr) > 0.2:
        lin_v, ang_v = 0.8, max(-1.0, min(1.0, herr*1.5))
    else:
        lin_v, ang_v = 1.0, max(-0.6, min(0.6, herr*1.2))
    send_wheels(lin_v, ang_v)

    # Track distance
    if prev_gx is not None:
        s_accum += math.hypot(rx - prev_gx, ry - prev_gy)
    prev_gx, prev_gy = rx, ry

    # Record anchor every 2m
    if last_ax is None or math.hypot(rx - last_ax, ry - last_ay) >= 2.0:
        rgb = get_rgb()
        if rgb is not None:
            gray = clahe.apply(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
            kps, desc = orb.detectAndCompute(gray, None)
            aid = len(anchors)
            anchors.append({
                "id": aid, "s": round(s_accum, 2),
                "x": round(rx, 4), "y": round(ry, 4),
                "z": round(rz, 4), "yaw": round(ryaw, 4),
                "n_features": len(kps) if kps else 0,
                "direction": "outbound",
            })
            if desc is not None:
                anchor_descs[aid] = desc
            # Save to disk for inspection
            _rm = f"/workspace/simulation/isaac/route_memory/{args.route}"
            os.makedirs(f"{_rm}/rgb", exist_ok=True)
            os.makedirs(f"{_rm}/depth", exist_ok=True)
            from PIL import Image as _PILImg
            _PILImg.fromarray(rgb).save(f"{_rm}/rgb/{aid:04d}.jpg", quality=90)
            if desc is not None:
                np.save(f"{_rm}/rgb/{aid:04d}_desc.npy", desc)
                kp_coords = np.array([(k.pt[0],k.pt[1],k.size,k.angle) for k in kps], dtype=np.float32)
                np.save(f"{_rm}/rgb/{aid:04d}_kp.npy", kp_coords)
            # Save depth profile for robust matching
            depth_img = get_depth()
            if depth_img is not None:
                h, w = depth_img.shape[:2]
                strip = depth_img[h//3:h*2//3, :]
                n_bins = 32
                bw = w // n_bins
                dp = np.zeros(n_bins)
                for bi in range(n_bins):
                    col = strip[:, bi*bw:(bi+1)*bw]
                    valid = col[(col > 0.3) & (col < 10.0) & np.isfinite(col)]
                    dp[bi] = float(np.median(valid)) if len(valid) > 3 else 5.0
                anchor_depth_profiles[aid] = dp
            last_ax, last_ay = rx, ry
            if aid % 10 == 0:
                print(f"  teach: anchor {aid} s={s_accum:.0f}m "
                      f"({rx:.1f},{ry:.1f}) feat={len(kps) if kps else 0}")

    teach_dur = sim_time - teach_start
    print(f"\nTeach complete: {len(anchors)} anchors, {s_accum:.0f}m, {teach_dur:.0f}s")

    # Save anchors.json to disk
    _rm = f"/workspace/simulation/isaac/route_memory/{args.route}"
    with open(f"{_rm}/anchors.json", "w") as _af:
        json.dump(anchors, _af, indent=2)
    print(f"  Saved {len(anchors)} anchors to {_rm}/anchors.json")

# =====================================================================
# TELEPORT BACK TO START
# =====================================================================
print("\nTeleporting to start...")
stop_wheels()
for _ in range(60):
    app.update()

# Start repeat from anchor 6 - first clear zone (0 objects within 3m)
REPEAT_START_ANCHOR = 0 if args.route == "road" else min(6, len(anchors) - 1)
sx, sy = anchors[REPEAT_START_ANCHOR]["x"], anchors[REPEAT_START_ANCHOR]["y"]
syaw = anchors[REPEAT_START_ANCHOR]["yaw"]
sz = _terrain_height(sx, sy) + 0.5

# Stop physics, move robot, restart - PhysX ignores xform during play
timeline.stop()
for _ in range(30):
    app.update()

_husky_translate_op.Set(Gf.Vec3d(sx, sy, sz))
_husky_rotate_op.Set(Gf.Vec3f(0, 0, math.degrees(syaw)))
_cam_transform_op.Set(_make_cam_matrix(
    sx + CAM_FWD * math.cos(syaw), sy + CAM_FWD * math.sin(syaw),
    sz + CAM_UP, syaw))
for _ in range(30):
    app.update()

timeline.play()
for _ in range(300):
    app.update()
    stop_wheels()

p = _get_husky_pose()
teleport_err = math.hypot(p[0] - sx, p[1] - sy)
print(f"  robot at ({p[0]:.1f}, {p[1]:.1f}) - target ({sx:.1f}, {sy:.1f}) err={teleport_err:.1f}m")
if teleport_err > 5.0:
    print(f"  WARNING: teleport error {teleport_err:.1f}m > 5m!")

# Initialize encoder prev from actual GT position after teleport
enc_prev_x, enc_prev_y, enc_prev_yaw = p[0], p[1], p[3]

# =====================================================================
# PHASE 2: REPEAT - odometry-primary + visual correction
# =====================================================================
print(f"\n{'='*60}")
print(f"PHASE 2: REPEAT - {len(anchors)} anchors, odometry-primary")
print(f"{'='*60}\n")

from route_follower import RouteFollower

# Odometry-primary state
odom_s = 0.0
odom_anchor_idx = 0
vis_conf = 0.0

def find_anchor_by_s(s_val):
    best_i, best_d = 0, float("inf")
    lo = max(0, odom_anchor_idx - 2)
    hi = min(len(anchors), odom_anchor_idx + 10)
    for i in range(lo, hi):
        d = abs(anchors[i]["s"] - s_val)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i

# Robust anchor localizer - in-memory (no file I/O)
from robust_anchor_localizer import RobustAnchorLocalizer
robust_loc = RobustAnchorLocalizer.__new__(RobustAnchorLocalizer)
robust_loc.anchor_dir = "in-memory"
robust_loc.anchors = anchors
robust_loc.descriptors = anchor_descs
robust_loc.anchor_depths = anchor_depth_profiles
robust_loc.orb = orb
robust_loc.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
robust_loc.clahe = clahe
robust_loc.current_anchor_id = REPEAT_START_ANCHOR
robust_loc.confirmed_anchor_id = REPEAT_START_ANCHOR
robust_loc.confidence = 0.0
robust_loc.SEQUENCE_LENGTH = 5
robust_loc.SEQUENCE_AGREEMENT = 3
robust_loc.frame_votes = []
robust_loc.pending_anchor_id = None
robust_loc.pending_count = 0
robust_loc.CONFIRM_FRAMES = 3
robust_loc.MAX_CORRECTION = 3
robust_loc.GOOD_RATIO = 0.75
robust_loc.WINDOW_BACK = 3
robust_loc.WINDOW_FWD = 10
prev_confirmed = REPEAT_START_ANCHOR
print(f"  RobustAnchorLocalizer initialized (in-memory, {len(anchor_descs)} desc, "
      f"{len(anchor_depth_profiles)} depth profiles)")

follower = RouteFollower(anchors, lookahead_distance=args.lookahead)
follower.initialize_from_anchor(REPEAT_START_ANCHOR)
odom_s = anchors[REPEAT_START_ANCHOR]["s"]
odom_anchor_idx = REPEAT_START_ANCHOR

VISUAL_EVERY = 30  # every 30 frames = ~2Hz; visual is soft correction only
frame_count = 0
prev_lin = 0.0
repeat_start = sim_time

_obs_tag = "obs" if args.obstacles else "noobs"
_exp_dir = ("38_fast_planner" if args.use_planner
            else "36_obstacle_grid" if args.use_grid
            else "35_road_obstacle_avoidance")
log_path = (f"/workspace/simulation/isaac/experiments/{_exp_dir}/logs/"
            f"road_{_obs_tag}_{int(time.time())}.csv")
log_f = open(log_path, "w")
log_f.write("time,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,anchor_id,"
            "odom_s,vis_conf,cmd_lin,cmd_ang\n")

# Encoder-based odometry state (initialized after teleport from GT)
odom_yaw = p[3]  # from teleport GT
odom_x = p[0]
odom_y = p[1]
gyro_yaw = p[3]

# Previous GT for encoder simulation (pose diff)
enc_prev_x = enc_prev_y = enc_prev_yaw = None
ENCODER_NOISE = 0.005  # 0.5% - matches Husky A200 (78000 ticks/m)
enc_rng = np.random.RandomState(123)

# Gap navigator
from gap_navigator import GapNavigator
from traversability_filter import TraversabilityFilter
gap_nav = GapNavigator(robot_width=0.67, safety_margin=0.3)
trav_filter = TraversabilityFilter()
DEPTH_EVERY = 3
last_nav_lin = 0.0
last_nav_ang = 0.0
last_nav_mode = "ROUTE_TRACKING"
last_nav_debug = {}

# Obstacle grid (accumulated detection)
obstacle_grid = None
GRID_SECTORS = 80
if args.use_grid:
    from local_obstacle_grid import LocalObstacleGrid
    obstacle_grid = LocalObstacleGrid(size=20.0, resolution=0.2)
    print("  LocalObstacleGrid enabled (20m, 0.2m res, 100x100)")

grid_planner = None
if args.use_planner:
    from fast_grid_planner import FastGridPlanner
    grid_planner = FastGridPlanner(grid_size=100, resolution=0.2)
    print("  FastGridPlanner (A* cached) enabled")

# Stuck detection
stuck_enc_integral = 0.0
stuck_last_s = 0.0
stuck_check_time = sim_time
stuck_frames = 0
STUCK_THRESHOLD = 240  # 4 seconds - more patient
stuck_recovery_phase = 0
stuck_recovery_timer = 0
total_stuck_skips = 0

# Anchor correction
ANCHOR_POS_ALPHA = 0.3
ANCHOR_YAW_ALPHA = 0.15

try:
    while sim_time < args.duration and not follower.finished:
        app.update()
        sim_time += dt
        frame_count += 1

        # Update camera (GT needed for rendering + encoder simulation)
        rx, ry, rz, ryaw = update_camera()

        # 1. Encoder odometry: PhysX pose diff (simulates wheel encoders)
        if enc_prev_x is not None:
            enc_dx = rx - enc_prev_x
            enc_dy = ry - enc_prev_y
            enc_lin = math.hypot(enc_dx, enc_dy)
            enc_ang = ryaw - enc_prev_yaw
            enc_ang = math.atan2(math.sin(enc_ang), math.cos(enc_ang))

            # Add encoder noise (0.5%)
            enc_lin *= (1.0 + enc_rng.normal(0, ENCODER_NOISE))
            enc_ang *= (1.0 + enc_rng.normal(0, ENCODER_NOISE * 2))

            # FIX 1: Route progress only from forward motion, not turns
            # Skip distance when mostly turning (angular > 0.3 rad/frame, linear < 5mm)
            if enc_lin > 0.005 and abs(enc_ang) < 0.05:
                odom_s += enc_lin
            elif enc_lin > 0.01:
                # Moving but also turning - count partial
                odom_s += enc_lin * 0.5
            odom_anchor_idx = find_anchor_by_s(odom_s)

            # IMU gyro heading
            imu_reading = _imu_interface.get_sensor_reading(
                IMU_SENSOR_PATH, read_gravity=True)
            if imu_reading.is_valid:
                _, _, gz = _imu_urf_to_flu(
                    imu_reading.ang_vel_x,
                    imu_reading.ang_vel_y,
                    imu_reading.ang_vel_z)
                if abs(gz) > 0.03:
                    gyro_yaw += gz * dt

            # Heading fusion: 70% encoder, 30% gyro
            enc_yaw = odom_yaw + enc_ang
            odom_yaw = 0.7 * enc_yaw + 0.3 * gyro_yaw
            odom_yaw = math.atan2(math.sin(odom_yaw), math.cos(odom_yaw))
            gyro_yaw = odom_yaw

            # Position from encoder displacement + fused heading
            odom_x += enc_lin * math.cos(odom_yaw)
            odom_y += enc_lin * math.sin(odom_yaw)

            # Stuck detection from encoder
            stuck_enc_integral += enc_lin
        else:
            odom_anchor_idx = 0

        enc_prev_x, enc_prev_y, enc_prev_yaw = rx, ry, ryaw

        # 2. Robust visual matching (sequence + depth + conservative)
        if frame_count % VISUAL_EVERY == 0:
            rgb = get_rgb()
            depth_for_match = get_depth()
            if rgb is not None:
                loc_id, vis_conf, _ = robust_loc.localize(rgb, depth_for_match)

                # Soft odom_s correction from confirmed anchor
                if vis_conf > 0.3:
                    vis_s = anchors[robust_loc.confirmed_anchor_id]["s"]
                    corr = (vis_s - odom_s) * 0.2
                    odom_s += max(-3.0, min(3.0, corr))
                odom_anchor_idx = find_anchor_by_s(odom_s)

            # Position correction ONLY when confirmed changes + high confidence
            if robust_loc.confirmed_anchor_id != prev_confirmed and vis_conf > 0.6:
                ca = anchors[robust_loc.confirmed_anchor_id]
                alpha = min(vis_conf * 0.25, 0.2)
                odom_x = (1 - alpha) * odom_x + alpha * ca["x"]
                odom_y = (1 - alpha) * odom_y + alpha * ca["y"]
                yd = ca["yaw"] - odom_yaw
                yd = math.atan2(math.sin(yd), math.cos(yd))
                odom_yaw += 0.08 * yd
                gyro_yaw = odom_yaw
                prev_confirmed = robust_loc.confirmed_anchor_id

            # Low-confidence safe mode - only reduce speed slightly,
            # not halve it (vis is unreliable cross-session anyway)
            if vis_conf < 0.4:
                gap_nav.MAX_LINEAR = 0.55
                gap_nav.SLOW_SAFE_LINEAR = 0.2
                follower.lookahead_dist = 5.0
            else:
                gap_nav.MAX_LINEAR = 0.7
                gap_nav.SLOW_SAFE_LINEAR = 0.25
                follower.lookahead_dist = 8.0

        # 3. Set follower from encoder odom (for lookahead target)
        follower.current_anchor_idx = odom_anchor_idx
        follower.odom_x = odom_x
        follower.odom_y = odom_y
        follower.odom_yaw = odom_yaw

        # 4. Route desired heading (from follower lookahead)
        la = follower.get_adaptive_lookahead()
        tx, ty, _ = follower.get_lookahead_point(la)
        # Find nearest anchor to GT position for correct lookahead
        _gt_anchor = min(range(len(anchors)),
                         key=lambda i: math.hypot(anchors[i]["x"]-rx, anchors[i]["y"]-ry))
        _gt_la = min(_gt_anchor + 4, len(anchors) - 1)  # lookahead ~8m
        _gt_tx, _gt_ty = anchors[_gt_la]["x"], anchors[_gt_la]["y"]
        desired_heading = math.atan2(_gt_ty - ry, _gt_tx - rx)
        heading_err = desired_heading - ryaw
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))
        _turning_to_route = False

        # 5. GAP NAVIGATOR - primary cmd_vel (skip 1s for depth settle)
        if frame_count > 60 and frame_count % DEPTH_EVERY == 0:
            depth_raw = get_depth()
            if depth_raw is not None:
                # Traversability filter: remove sparse vegetation from depth
                depth_img = trav_filter.filter(depth_raw)

                if obstacle_grid is not None:
                    # Grid mode: accumulate depth into grid
                    obstacle_grid.update(rx, ry, ryaw, depth_img)
                    gs = obstacle_grid.get_stats()

                if grid_planner is not None:
                    # A* planner mode: plan path on grid to lookahead anchor
                    target_dx = _gt_tx - rx
                    target_dy = _gt_ty - ry
                    plan_dx, plan_dy, path_ok, path_len = \
                        grid_planner.plan(obstacle_grid.grid, target_dx, target_dy)

                    if path_ok:
                        plan_angle = math.atan2(plan_dy, plan_dx)
                        plan_herr = plan_angle - ryaw
                        plan_herr = math.atan2(math.sin(plan_herr),
                                               math.cos(plan_herr))
                        last_nav_ang = float(np.clip(
                            plan_herr * 1.5, -0.5, 0.5))
                        # Speed: maintain high speed, slow only for sharp turns
                        herr_abs = abs(plan_herr)
                        if herr_abs < 0.3:
                            last_nav_lin = 0.6
                        elif herr_abs < 0.7:
                            last_nav_lin = 0.4
                        else:
                            last_nav_lin = 0.2
                        last_nav_mode = "PLAN"
                    else:
                        last_nav_lin = 0.0
                        last_nav_ang = 0.3
                        last_nav_mode = "NO_PATH"

                    pdi = grid_planner.get_debug_info()
                    last_nav_debug = {
                        "mode": last_nav_mode,
                        "path_len": round(path_len, 1),
                        "grid_obs": gs["obstacle_cells"],
                        "grid_pct": gs["obstacle_pct"],
                        "impassable": pdi.get("impassable", 0),
                        "plan_ms": pdi.get("plan_ms", 0),
                        "blocked_ratio": 0,
                        "total_gaps": 0, "valid_gaps": 0,
                    }

                elif obstacle_grid is not None:
                    # Grid profile mode (exp 36): use gap nav on grid profile
                    acc_profile = obstacle_grid.get_obstacle_profile(
                        ryaw, num_sectors=GRID_SECTORS)
                    last_nav_lin, last_nav_ang, last_nav_mode, last_nav_debug = \
                        gap_nav.compute_cmd_vel_from_profile(
                            acc_profile, heading_err)
                    last_nav_debug["grid_obs"] = gs["obstacle_cells"]
                    last_nav_debug["grid_pct"] = gs["obstacle_pct"]
                    last_nav_debug["grid_max"] = gs["max_hits"]
                else:
                    # Single-frame mode (original)
                    last_nav_lin, last_nav_ang, last_nav_mode, last_nav_debug = \
                        gap_nav.compute_cmd_vel(depth_img, heading_err)

                # Track filter effect
                last_nav_debug["trav"] = trav_filter.stats(depth_raw, depth_img)

        if frame_count <= 60:
            # First 1s: stop, let depth settle
            cmd_lin = 0.0
            cmd_ang = 0.0
        elif _turning_to_route:
            # Robot turned away from route - rotate toward route heading first
            cmd_lin = 0.0
            cmd_ang = float(np.clip(heading_err * 1.5, -0.5, 0.5))
        else:
            cmd_lin = last_nav_lin
            cmd_ang = last_nav_ang

        # 6. Stuck detection: encoder not moving
        if sim_time - stuck_check_time > 3.0:
            if stuck_enc_integral < 0.3:
                stuck_frames += 60
            else:
                stuck_frames = max(0, stuck_frames - 30)
            stuck_enc_integral = 0.0
            stuck_check_time = sim_time

        if stuck_frames > STUCK_THRESHOLD and stuck_recovery_phase == 0:
            stuck_recovery_phase = 1
            stuck_recovery_timer = 0
            print(f"  STUCK at ({rx:.1f},{ry:.1f}) a={odom_anchor_idx}")

        if stuck_recovery_phase > 0:
            stuck_recovery_timer += 1
            if stuck_recovery_phase == 1:
                # Back up for 3s (was 2s) - need more clearance from obstacles
                cmd_lin, cmd_ang = -0.4, 0.0
                if stuck_recovery_timer > 180:  # 3s at 60fps
                    stuck_recovery_phase = 2
                    stuck_recovery_timer = 0
                    # Pick turn direction: alternate sides each stuck event
                    stuck_recovery_phase = 2
            elif stuck_recovery_phase == 2:
                # Turn for 1.5s at higher rate (was 1s at 0.5)
                # Alternate direction based on stuck skip count
                turn_dir = 0.6 if total_stuck_skips % 2 == 0 else -0.6
                cmd_lin, cmd_ang = 0.0, turn_dir
                if stuck_recovery_timer > 90:  # 1.5s
                    stuck_recovery_phase = 3
                    stuck_recovery_timer = 0
            elif stuck_recovery_phase == 3:
                odom_s += 4.0
                odom_anchor_idx = find_anchor_by_s(odom_s)
                stuck_recovery_phase = 0
                stuck_frames = 0
                total_stuck_skips += 1
                print(f"  SKIP #{total_stuck_skips}: a={odom_anchor_idx} s={odom_s:.0f}m")
                cmd_lin, cmd_ang = last_nav_lin, last_nav_ang

        # Check route completion
        if odom_anchor_idx >= len(anchors) - 2:
            last_a = anchors[-1]
            if math.hypot(odom_x - last_a["x"], odom_y - last_a["y"]) < 5.0:
                follower.finished = True

        send_wheels(cmd_lin, cmd_ang)
        prev_lin = cmd_lin

        # 7. Log every second
        if frame_count % 60 == 0:
            log_f.write(
                f"{sim_time:.2f},{rx:.4f},{ry:.4f},{ryaw:.4f},"
                f"{odom_x:.4f},{odom_y:.4f},{odom_yaw:.4f},{odom_anchor_idx},"
                f"{odom_s:.2f},{vis_conf:.3f},"
                f"{cmd_lin:.3f},{cmd_ang:.3f}\n")
            log_f.flush()

        # 8. Save camera preview every 10s
        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < dt:
            _preview = get_rgb()
            if _preview is not None:
                try:
                    from PIL import Image as _PILImg
                    _PILImg.fromarray(_preview).save("/tmp/isaac_cam_fwd.tmp.jpg", quality=75)
                    os.replace("/tmp/isaac_cam_fwd.tmp.jpg", "/tmp/isaac_cam_fwd.jpg")
                except Exception:
                    pass
            odom_err = math.hypot(rx - odom_x, ry - odom_y)
            d = last_nav_debug
            gap_info = (f"{last_nav_mode} gaps={d.get('total_gaps',0)}/"
                        f"{d.get('valid_gaps',0)} blk={d.get('blocked_ratio',0):.0%}")
            if "sel_ang" in d:
                gap_info += f" sel={math.degrees(d['sel_ang']):+.0f}deg"
            rs = robust_loc.get_stats()
            print(
                f"  t={sim_time:.0f}s | GT=({rx:.1f},{ry:.1f}) "
                f"odom=({odom_x:.1f},{odom_y:.1f}) | "
                f"a={odom_anchor_idx}/{len(anchors)-1} "
                f"conf={rs['confirmed_id']} | "
                f"s={odom_s:.0f}m | err={odom_err:.1f}m | "
                f"vis={vis_conf:.2f} {gap_info} {d.get('trav','')}"
                + (f" grid={d.get('grid_obs',0)}cells/{d.get('grid_pct',0)}%"
                   if obstacle_grid else "")
                + (f" path={d.get('path_len',0)}m imp={d.get('impassable',0)}"
                   f" {d.get('plan_ms',0)}ms"
                   if grid_planner else ""))

except KeyboardInterrupt:
    print("\nstopped")

log_f.close()

rx, ry, rz, ryaw = _get_husky_pose()
odom_err = math.hypot(rx - odom_x, ry - odom_y)
repeat_dur = sim_time - repeat_start
print(f"\n{'='*60}")
print(f"RESULT:")
print(f"  Mode: odom+IMU (no GT)")
print(f"  Teach: {len(anchors)} anchors, {s_accum:.0f}m")
print(f"  Repeat duration: {repeat_dur:.0f}s")
print(f"  Final GT: ({rx:.1f}, {ry:.1f})")
print(f"  Final odom: ({odom_x:.1f}, {odom_y:.1f})")
print(f"  Odom error: {odom_err:.1f}m")
print(f"  Anchor: {odom_anchor_idx}/{len(anchors)-1}")
print(f"  odom_s: {odom_s:.0f}/{anchors[-1]['s']:.0f}m")
print(f"  Stuck skips: {total_stuck_skips}")
print(f"  Finished: {follower.finished}")
print(f"  Log: {log_path}")
print(f"{'='*60}")

timeline.stop()
app.close()
