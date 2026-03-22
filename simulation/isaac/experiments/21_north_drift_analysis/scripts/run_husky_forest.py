#!/usr/bin/env python3
"""
Husky A200 in forest scene with PhysX articulation drive and ROS2.
Records RGB-D + IMU data for ORB-SLAM3 evaluation.

publishes: /camera/color/image_raw, /camera/depth/image_rect_raw, /imu/data, /odom, /tf
subscribes: /cmd_vel

usage:
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /opt/isaac-sim-6.0.0/python.sh run_husky_forest.py --route road --duration 600
"""
import os, sys, argparse, time, math, json
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=300.0)
parser.add_argument("--route", type=str, default=None,
                    choices=["road", "north", "south", "road_nav2"],
                    help="predefined route: road (S-curve), north (forest), south (forest)")
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

# IMU URF->FLU conversion (PhysX IMU is in URF: X=up, Y=right, Z=fwd)
# ORB-SLAM3 expects FLU: X=fwd, Y=left, Z=up (gravity on +Z)
def _imu_urf_to_flu(ux, uy, uz):
    """Convert IMU vector from URF to FLU."""
    return (uz, -uy, ux)  # flu_x=urf_z, flu_y=-urf_y, flu_z=urf_x

# IMU sensor on imu_link
IMU_SENSOR_PATH = IMU_PATH + "/imu_sensor"
_imu_ok, _imu_prim = omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor",
    parent=IMU_PATH,
    sensor_period=1.0 / 200.0,
    linear_acceleration_filter_size=60,
    angular_velocity_filter_size=30,
    orientation_filter_size=30,
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
    """camera matrix: look along yaw direction with pitch tilt, up = +Z"""
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
print(f"  camera: {CAM_RGB}")

# verify prim paths
for name, path in [("base_link", BASE_LINK), ("camera", CAM_RGB), ("imu", IMU_PATH)]:
    print(f"  {name}: {stage.GetPrimAtPath(path).IsValid()} ({path})")

# spawn position
husky_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Husky"))
husky_xf.ClearXformOpOrder()
_husky_translate_op = husky_xf.AddTranslateOp()
_husky_rotate_op = husky_xf.AddRotateXYZOp()
_husky_translate_op.Set(Gf.Vec3d(-95, -6, 0.2))  # updated after terrain_height defined
_husky_rotate_op.Set(Gf.Vec3f(0, 0, 0))
print("  spawn: (-95, -6, 0.2)")

# ros2 graph
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
            ("PubTF.inputs:topicName", "tf"),
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
_spawn_z = _terrain_height(-95, -6) + 0.5
_husky_translate_op.Set(Gf.Vec3d(-95, -6, _spawn_z))
_cam_transform_op.Set(_make_cam_matrix(-95 + CAM_FWD, -6, _spawn_z + CAM_UP, 0))
print(f"  spawn z={_spawn_z:.2f}")

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
        _obstacles.append((m["x"], m["y"], 0.4))
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
    """find sequence of road waypoints from robot to goal"""
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

    while sim_time < args.duration:
        app.update()
        rclpy.spin_once(node, timeout_sec=0.001)
        sim_time += 1.0 / 60.0

        # auto-route: advance to next waypoint when arrived
        if _auto_route is not None and goal_x is None and _auto_idx < len(_auto_route):
            goal_x, goal_y = _auto_route[_auto_idx]
            print(f"  ROUTE [{_auto_idx}/{len(_auto_route)-1}]: ({goal_x:.1f}, {goal_y:.1f})")
        elif _auto_route is not None and goal_x is None and _auto_idx >= len(_auto_route):
            print(f"\n  ROUTE COMPLETE! Recorded {_rec_img_count} frames.")
            break  # stop simulation when route is done

        # read goal from file (written by web UI) - only when no auto-route
        if _auto_route is None:
            try:
                with open("/tmp/isaac_goal.txt", "r") as f:
                    goal_txt = f.read().strip()
                if goal_txt == "stop":
                    goal_x, goal_y = None, None
                elif goal_txt == "reverse":
                    # drive backwards for 2s via Articulation API
                    goal_x, goal_y = None, None
                    rev_vel = np.zeros((1, _art_num_dof))
                    for idx in _wheel_dof_idx["left"] + _wheel_dof_idx["right"]:
                        rev_vel[0, idx] = -3.0  # rad/s backwards
                    for _ in range(120):  # 2 seconds at 60Hz
                        _husky_art.set_joint_velocity_targets(rev_vel)
                        app.update()
                        pp = _get_husky_pose()
                        _ry = math.atan2(2*(float(pp[1][0])*float(pp[1][3])), 1-2*(float(pp[1][3])**2))
                        _cam_transform_op.Set(_make_cam_matrix(
                            float(pp[0][0]) + CAM_FWD*math.cos(_ry),
                            float(pp[0][1]) + CAM_FWD*math.sin(_ry),
                            float(pp[0][2]) + CAM_UP, _ry))
                    _husky_art.set_joint_velocity_targets(np.zeros((1, _art_num_dof)))
                    try:
                        os.remove("/tmp/isaac_goal.txt")
                    except:
                        pass
                    print("  REVERSED ~2s")
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

            if dist < 2.5:  # wider arrival threshold for PhysX dynamics
                vels = np.array([0.0, 0.0, 0.0, 0.0])
                print(f"  ARRIVED at ({goal_x:.1f}, {goal_y:.1f}), actual=({rx:.1f},{ry:.1f})")
                goal_x, goal_y = None, None
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

                # Real Husky A200 typical operating speed: 0.8-1.0 m/s
                max_speed = 1.0
                if abs(err) > 0.7:
                    lin_v, ang_v = 0.4, max(-1.2, min(1.2, err * 2.0))
                elif abs(err) > 0.2:
                    lin_v, ang_v = 0.8, max(-1.0, min(1.0, err * 1.5))
                else:
                    lin_v, ang_v = max_speed, max(-0.6, min(0.6, err * 1.2))

                v_left = (lin_v - ang_v * track / 2) / wheel_r
                v_right = (lin_v + ang_v * track / 2) / wheel_r
                vels = np.array([v_left, v_right, v_left, v_right])
        elif abs(cmd.linear.x) > 0.01 or abs(cmd.angular.z) > 0.01:
            v_left = (cmd.linear.x - cmd.angular.z * track / 2) / wheel_r
            v_right = (cmd.linear.x + cmd.angular.z * track / 2) / wheel_r
            vels = np.array([v_left, v_right, v_left, v_right])
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
        # sync camera with robot
        cam_x = rx + CAM_FWD * math.cos(current_yaw)
        cam_y = ry + CAM_FWD * math.sin(current_yaw)
        cam_z = rz + CAM_UP
        fwd_d = 0.5
        z_front = _terrain_height(rx + fwd_d * math.cos(current_yaw), ry + fwd_d * math.sin(current_yaw))
        z_back = _terrain_height(rx - fwd_d * math.cos(current_yaw), ry - fwd_d * math.sin(current_yaw))
        pitch = math.atan2(z_front - z_back, 2 * fwd_d)
        _cam_transform_op.Set(_make_cam_matrix(cam_x, cam_y, cam_z, current_yaw, pitch))

        # save camera at ~10Hz
        frame_n = int(sim_time * 60)
        if frame_n % 6 == 0:
            d_fwd = ann_fwd.get_data() if 'ann_fwd' in dir() else None
            if d_fwd is not None and np.mean(d_fwd[:, :, :3]) > 1:
                from PIL import Image as PILImg
                img = PILImg.fromarray(d_fwd[:, :, :3]).resize((640, 480))
                img.save("/tmp/isaac_cam_fwd.tmp.jpg", quality=75)
                os.replace("/tmp/isaac_cam_fwd.tmp.jpg", "/tmp/isaac_cam_fwd.jpg")
                # save to recording dir
                if _recording:
                    img.save(f"{_rec_dir}/camera_rgb/{sim_time:.4f}.jpg", quality=90)
                    # save depth (16-bit PNG, millimeters)
                    d_depth = ann_depth.get_data() if 'ann_depth' in dir() else None
                    if d_depth is not None:
                        # distance_to_image_plane returns float32 z-depth in meters
                        depth_mm = (d_depth * 1000).astype(np.uint16)
                        PILImg.fromarray(depth_mm).save(f"{_rec_dir}/camera_depth/{sim_time:.4f}.png")
                    _rec_img_count += 1
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
            # TUM format GT (camera position)
            cam_gt_x = rx + CAM_FWD * math.cos(cur_yaw)
            cam_gt_y = ry + CAM_FWD * math.sin(cur_yaw)
            cam_gt_z = rz + CAM_UP
            _gt_tum_file.write(f"{ts:.4f} {cam_gt_x:.6f} {cam_gt_y:.6f} {cam_gt_z:.6f} 0.000000 0.000000 {math.sin(cur_yaw/2):.6f} {math.cos(cur_yaw/2):.6f}\n")

            # wheel odometry
            dt = 1.0/60.0
            if hasattr(_get_husky_pose, '_prev'):
                px, py, pyaw = _get_husky_pose._prev
                vx = (rx - px) / dt
                vy = (ry - py) / dt
                vyaw = cur_yaw - pyaw
                if vyaw > math.pi: vyaw -= 2 * math.pi
                if vyaw < -math.pi: vyaw += 2 * math.pi
                vyaw /= dt
                # encoder noise
                vx += rng.uniform(-0.01, 0.01)
                vy += rng.uniform(-0.01, 0.01)
                vyaw += rng.uniform(-0.002, 0.002)
                lin_v = math.hypot(vx, vy)
                _odom_file.write(f"{ts:.4f},{rx:.6f},{ry:.6f},{rz:.6f},{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f},{lin_v:.6f},{vyaw:.6f}\n")

                # IMU from PhysX sensor - convert URF->FLU, write ONE unique reading
                # (NOT 3 duplicates - that broke ORB-SLAM3 preintegration)
                _imu_reading = _imu_interface.get_sensor_reading(IMU_SENSOR_PATH, read_gravity=True)
                if _imu_reading.is_valid:
                    ax, ay, az = _imu_urf_to_flu(
                        _imu_reading.lin_acc_x,
                        _imu_reading.lin_acc_y,
                        _imu_reading.lin_acc_z)
                    gx, gy, gz = _imu_urf_to_flu(
                        _imu_reading.ang_vel_x,
                        _imu_reading.ang_vel_y,
                        _imu_reading.ang_vel_z)
                    _imu_file.write(f"{ts:.4f},"
                        f"{ax:.6f},{ay:.6f},{az:.6f},"
                        f"{gx:.6f},{gy:.6f},{gz:.6f},"
                        f"{qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}\n")
            _get_husky_pose._prev = (rx, ry, cur_yaw)

        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < 0.02:
            pp = _get_husky_pose()[0]
            print(f"  t={sim_time:.0f}s robot=({pp[0]:.1f}, {pp[1]:.1f}, {pp[2]:.2f})")

except KeyboardInterrupt:
    print("\nstopped")

# close recording files
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
