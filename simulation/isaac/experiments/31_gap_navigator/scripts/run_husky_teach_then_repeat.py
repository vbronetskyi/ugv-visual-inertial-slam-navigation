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
parser.add_argument("--duration", type=float, default=800.0,
                    help="total sim time (teach + repeat)")
parser.add_argument("--lookahead", type=float, default=8.0)
args, _ = parser.parse_known_args()

# =====================================================================
# Isaac Sim setup (identical to run_husky_teach_repeat.py)
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

# Thin forest - deactivate every 3rd tree/shrub + collision prim
print("thinning forest (deactivate every 3rd tree/shrub)...")
_thin_count = 0
for prefix, pattern in [("/World/Trees/tree_", "tree"),
                         ("/World/Cover/cover_", "cover"),
                         ("/World/TreeCollision/col_", "col")]:
    idx = 0
    while True:
        if pattern == "cover":
            path = f"{prefix}{idx:04d}"
        else:
            path = f"{prefix}{idx:03d}"
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            break
        if idx % 3 == 0:
            prim.SetActive(False)
            _thin_count += 1
        idx += 1
print(f"  deactivated {_thin_count} prims")

# Also thin gazebo_models.json for collision list
_gm = json.load(open("/tmp/gazebo_models_dense.json"))
_thinned = []
_tree_idx = 0
for m in _gm:
    if m["type"] in ("pine", "oak", "shrub"):
        if _tree_idx % 3 != 0:
            _thinned.append(m)
        _tree_idx += 1
    else:
        _thinned.append(m)
with open("/tmp/gazebo_models.json", "w") as f:
    json.dump(_thinned, f)
print(f"  collision models: {len(_gm)} -> {len(_thinned)}")

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


# =====================================================================
# PHASE 1: TEACH
# =====================================================================
print(f"\n{'='*60}")
print(f"PHASE 1: TEACH - driving {len(teach_waypoints)} waypoints")
print(f"{'='*60}\n")

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
orb = cv2.ORB_create(nfeatures=1000)

anchors = []
anchor_descs = {}
last_ax = last_ay = None
s_accum = 0.0
prev_gx = prev_gy = None

wp_idx = 0
sim_time = 5.0
teach_start = sim_time

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
            })
            if desc is not None:
                anchor_descs[aid] = desc
            last_ax, last_ay = rx, ry
            if aid % 10 == 0:
                print(f"  teach: anchor {aid} s={s_accum:.0f}m "
                      f"({rx:.1f},{ry:.1f}) feat={len(kps) if kps else 0}")

teach_dur = sim_time - teach_start
print(f"\nTeach complete: {len(anchors)} anchors, {s_accum:.0f}m, {teach_dur:.0f}s")

# =====================================================================
# TELEPORT BACK TO START
# =====================================================================
print("\nTeleporting to start...")
stop_wheels()
for _ in range(60):
    app.update()

# Start repeat from anchor 6 - first clear zone (0 objects within 3m)
REPEAT_START_ANCHOR = min(6, len(anchors) - 1)
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
odom_s = 0.0  # distance traveled (metres)
odom_anchor_idx = 0
vis_conf = 0.0

# Visual matching setup (for soft correction only)
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
VISUAL_CORRECTION_ALPHA = 0.3
VISUAL_MIN_CONF = 0.25

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

def visual_correction(rgb_frame):
    global odom_s, odom_anchor_idx, vis_conf
    gray = clahe.apply(cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY))
    kps, desc = orb.detectAndCompute(gray, None)
    if desc is None or len(kps) < 10:
        return

    s0 = max(0, odom_anchor_idx - 3)
    s1 = min(len(anchors), odom_anchor_idx + 5)
    best_id, best_score = odom_anchor_idx, 0
    for aid in range(s0, s1):
        if aid not in anchor_descs:
            continue
        try:
            ms = matcher.knnMatch(desc, anchor_descs[aid], k=2)
        except cv2.error:
            continue
        good = sum(1 for pair in ms
                   if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance)
        if good > best_score:
            best_score = good
            best_id = aid

    vis_conf = min(1.0, best_score / 30.0)
    if vis_conf > VISUAL_MIN_CONF:
        visual_s = anchors[best_id]["s"]
        correction = (visual_s - odom_s) * VISUAL_CORRECTION_ALPHA
        correction = max(-4.0, min(4.0, correction))
        odom_s += correction

follower = RouteFollower(anchors, lookahead_distance=args.lookahead)
follower.initialize_from_anchor(REPEAT_START_ANCHOR)
odom_s = anchors[REPEAT_START_ANCHOR]["s"]
odom_anchor_idx = REPEAT_START_ANCHOR

VISUAL_EVERY = 30  # every 30 frames = ~2Hz; visual is soft correction only
frame_count = 0
prev_lin = 0.0
repeat_start = sim_time

log_path = (f"/workspace/simulation/isaac/experiments/31_gap_navigator/logs/"
            f"gap_nav_{args.route}_{int(time.time())}.csv")
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
gap_nav = GapNavigator(robot_width=0.67, safety_margin=0.3)
DEPTH_EVERY = 3
last_nav_lin = 0.0
last_nav_ang = 0.0
last_nav_mode = "ROUTE_TRACKING"
last_nav_debug = {}

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

        # 2. Visual correction - FIX 2: only correct when confidence HIGH
        if frame_count % VISUAL_EVERY == 0:
            rgb = get_rgb()
            if rgb is not None:
                visual_correction(rgb)
                odom_anchor_idx = find_anchor_by_s(odom_s)

            # Anchor position correction ONLY if visual confidence > 0.6
            if vis_conf > 0.6:
                ca = anchors[odom_anchor_idx]
                alpha = min(vis_conf * 0.3, 0.25)
                odom_x = (1 - alpha) * odom_x + alpha * ca["x"]
                odom_y = (1 - alpha) * odom_y + alpha * ca["y"]
                yd = ca["yaw"] - odom_yaw
                yd = math.atan2(math.sin(yd), math.cos(yd))
                odom_yaw += 0.1 * yd
                gyro_yaw = odom_yaw

        # 3. Set follower from encoder odom (for lookahead target)
        follower.current_anchor_idx = odom_anchor_idx
        follower.odom_x = odom_x
        follower.odom_y = odom_y
        follower.odom_yaw = odom_yaw

        # 4. Route desired heading (from follower lookahead)
        la = follower.get_adaptive_lookahead()
        tx, ty, _ = follower.get_lookahead_point(la)
        desired_heading = math.atan2(ty - odom_y, tx - odom_x)
        heading_err = desired_heading - odom_yaw
        heading_err = math.atan2(math.sin(heading_err), math.cos(heading_err))

        # 5. GAP NAVIGATOR - primary cmd_vel (skip 1s for depth settle)
        if frame_count > 60 and frame_count % DEPTH_EVERY == 0:
            depth_img = get_depth()
            if depth_img is not None:
                last_nav_lin, last_nav_ang, last_nav_mode, last_nav_debug = \
                    gap_nav.compute_cmd_vel(depth_img, heading_err)

        if frame_count <= 60:
            # First 1s: stop, let depth settle
            cmd_lin = 0.0
            cmd_ang = 0.0
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
                cmd_lin, cmd_ang = -0.3, 0.0
                if stuck_recovery_timer > 120:
                    stuck_recovery_phase = 2
                    stuck_recovery_timer = 0
            elif stuck_recovery_phase == 2:
                cmd_lin, cmd_ang = 0.0, 0.5
                if stuck_recovery_timer > 60:
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

        # 8. Print every 10s
        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < dt:
            odom_err = math.hypot(rx - odom_x, ry - odom_y)
            d = last_nav_debug
            gap_info = (f"{last_nav_mode} gaps={d.get('total_gaps',0)}/"
                        f"{d.get('valid_gaps',0)} blk={d.get('blocked_ratio',0):.0%}")
            if "sel_ang" in d:
                gap_info += f" sel={math.degrees(d['sel_ang']):+.0f}deg"
            print(
                f"  t={sim_time:.0f}s | GT=({rx:.1f},{ry:.1f}) "
                f"odom=({odom_x:.1f},{odom_y:.1f}) | "
                f"a={odom_anchor_idx}/{len(anchors)-1} | "
                f"s={odom_s:.0f}m | err={odom_err:.1f}m | "
                f"{gap_info}")

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
