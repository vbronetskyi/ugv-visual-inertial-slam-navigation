#!/usr/bin/env python3
"""
Isaac Sim bridge for Nav2 navigation.
Publishes GT TF + depth pointcloud, reads cmd_vel from Nav2.

Usage:
  export ROS_DOMAIN_ID=42
  /opt/isaac-sim-6.0.0/python.sh scripts/run_nav2_bridge.py --route road --obstacles --duration 900
"""
import os, sys, argparse, time, math, json
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--route", default="road")
parser.add_argument("--obstacles", action="store_true")
parser.add_argument("--duration", type=float, default=900.0)
args, _ = parser.parse_known_args()

os.environ.setdefault("ROS_DOMAIN_ID", "42")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "max_bounces": 4,
                     "samples_per_pixel_per_frame": 8,
                     "renderer": "PathTracing"})
import omni
from pxr import UsdGeom, Gf, Sdf

HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
SCENE_USD = "/opt/husky_forest_scene.usd"

manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

# Load scene
print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()
print("scene loaded")

# Spawn obstacles
if args.obstacles:
    sys.path.insert(0, os.path.dirname(__file__))
    from spawn_obstacles import spawn_obstacles
    n = spawn_obstacles(stage, args.route)
    print(f"spawned {n} obstacles")
    for _ in range(30):
        app.update()

# Add Husky
print("adding Husky...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(300):
    app.update()

BASE_LINK = "/World/Husky/Geometry/base_link"
CAM_FWD = 0.5
CAM_UP = 0.48

# Load route start position
ROUTE_MEMORY = f"/workspace/simulation/isaac/route_memory/{args.route}"
with open(f"{ROUTE_MEMORY}/anchors.json") as f:
    all_anchors = json.load(f)
max_x_idx = max(range(len(all_anchors)), key=lambda i: all_anchors[i]["x"])
anchors = all_anchors[:max_x_idx + 1]
start = anchors[0]

# Terrain height function
def _terrain_height(x, y):
    RWPS = [(-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),
            (-70,-0.5),(-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),
            (-40,-5.2),(-35,-4),(-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),
            (-10,1.8),(-5,2),(0,1.5),(5,0.5),(10,-0.8),(15,-2.2),(20,-3.5),
            (25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),(50,-0.5),(55,-1),
            (60,-2),(65,-3.2),(70,-4.5),(75,-5)]
    def road_y(x):
        if x <= RWPS[0][0]: return RWPS[0][1]
        if x >= RWPS[-1][0]: return RWPS[-1][1]
        for i in range(len(RWPS)-1):
            if RWPS[i][0] <= x <= RWPS[i+1][0]:
                t = (x - RWPS[i][0]) / (RWPS[i+1][0] - RWPS[i][0])
                return RWPS[i][1] + t * (RWPS[i+1][1] - RWPS[i][1])
        return 0
    h = 0.0
    h += 0.5*math.sin(x*0.018+0.5)*math.cos(y*0.022+1.2)
    h += 0.35*math.sin(x*0.035+2.1)*math.sin(y*0.03+0.7)
    h += 0.18*math.sin(x*0.07+3.3)*math.cos(y*0.065+2.5)
    h += 0.12*math.cos(x*0.11+1.0)*math.sin(y*0.09+4.0)
    h += 0.06*math.sin(x*0.5+0.7)*math.cos(y*0.43+2.1)
    h += 0.04*math.cos(x*0.7+3.5)*math.sin(y*0.6+0.4)
    h += 0.03*math.sin(x*1.0+1.2)*math.cos(y*0.83+3.8)
    rd = abs(y - road_y(x))
    if rd < 4.0: h *= (rd / 4.0) ** 2
    if rd < 2.0: h -= 0.06 * (1.0 - rd / 2.0)
    return max(h, -0.5)

# Teleport Husky to start
sx, sy, syaw = start["x"], start["y"], start["yaw"]
sz = _terrain_height(sx, sy) + 0.15
xf = UsdGeom.Xformable(stage.GetPrimAtPath(BASE_LINK))
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(sx, sy, sz))
xf.AddRotateZOp().Set(math.degrees(syaw))
print(f"spawn: ({sx:.1f}, {sy:.1f}) yaw={math.degrees(syaw):.0f}")

# Husky drive API (UsdPhysics.DriveAPI, same as run_husky_teach_then_repeat.py)
from pxr import UsdPhysics
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
    else:
        print(f"WARNING: drive not found for {wname}")
print(f"  {len(_wheel_vel_attrs)} wheel drives initialized")

WHEEL_SEP = 0.555
WHEEL_RAD = 0.165

def send_wheels(lin, ang):
    vl = (lin - ang * WHEEL_SEP / 2) / WHEEL_RAD
    vr = (lin + ang * WHEEL_SEP / 2) / WHEEL_RAD
    # Left wheels: indices 0, 2. Right wheels: indices 1, 3.
    for i, vel in enumerate([vl, vr, vl, vr]):
        if i < len(_wheel_vel_attrs):
            _wheel_vel_attrs[i].Set(math.degrees(vel))

def _get_husky_pose():
    prim = stage.GetPrimAtPath(BASE_LINK)
    m = omni.usd.get_world_transform_matrix(prim)
    t = m.ExtractTranslation()
    fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
    yaw = math.atan2(fwd[1], fwd[0])
    return float(t[0]), float(t[1]), float(t[2]), float(yaw)

# Setup camera
from isaacsim.sensors.camera import Camera
cam = Camera(prim_path="/World/HuskyCamera",
             resolution=(640, 480), frequency=10)
cam.initialize()
cam.set_focal_length(1.88)
cam.set_clipping_range(0.1, 100.0)

# Start simulation
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(120):
    app.update()

# Setup ROS2 bridge
import sys as _sys
_ros2_path = "/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/rclpy"
if _ros2_path not in _sys.path:
    _sys.path.insert(0, _ros2_path)
import os as _os
_os.environ["LD_LIBRARY_PATH"] = _os.environ.get("LD_LIBRARY_PATH", "") + \
    ":/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib"
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import tf2_ros
import struct

rclpy.init()
node = rclpy.create_node("nav2_bridge")

# cmd_vel subscriber
latest_cmd = [0.0, 0.0]  # [linear, angular]
def cmd_cb(msg):
    latest_cmd[0] = msg.linear.x
    latest_cmd[1] = msg.angular.z
node.create_subscription(Twist, "/cmd_vel", cmd_cb, 10)

# TF broadcaster (GT as map->base_link)
tf_broadcaster = tf2_ros.TransformBroadcaster(node)

# Depth pointcloud publisher
pc_pub = node.create_publisher(PointCloud2, "/depth_points", 10)

# Camera intrinsics for depth -> pointcloud
FX = FY = 320.0
CX, CY = 320.0, 240.0
DEPTH_W, DEPTH_H = 640, 480
POINT_STEP = 4  # every 4th pixel

print(f"\nNav2 bridge ready. Publishing GT TF + depth pointcloud.")
print(f"Subscribing to /cmd_vel. Duration: {args.duration}s")
print(f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', 'default')}")

sim_time = 0.0
dt = 1.0 / 60.0
frame_count = 0
log_path = f"/workspace/simulation/isaac/experiments/40_tr_nav2_hybrid/logs/bridge_{int(time.time())}.log"
log_f = open(log_path, "w")

try:
    while sim_time < args.duration:
        app.update()
        sim_time += dt
        frame_count += 1

        rx, ry, rz, ryaw = _get_husky_pose()

        # Update camera position
        cam_x = rx + CAM_FWD * math.cos(ryaw)
        cam_y = ry + CAM_FWD * math.sin(ryaw)
        cam_z = rz + CAM_UP
        cam_prim = stage.GetPrimAtPath("/World/HuskyCamera")
        xf_cam = UsdGeom.Xformable(cam_prim)
        ops = xf_cam.GetOrderedXformOps()
        if ops:
            ops[0].Set(Gf.Vec3d(cam_x, cam_y, cam_z))
            if len(ops) > 1:
                # Rotation as quaternion (yaw only)
                half = ryaw / 2.0
                ops[1].Set(Gf.Quatd(math.cos(half), 0, 0, math.sin(half)))

        # Apply Nav2 cmd_vel
        send_wheels(latest_cmd[0], latest_cmd[1])

        # Publish TF every 3 frames (~20Hz)
        if frame_count % 3 == 0:
            rclpy.spin_once(node, timeout_sec=0)

            now_stamp = node.get_clock().now().to_msg()

            # map -> odom (identity - GT localization, no drift)
            tf_map_odom = TransformStamped()
            tf_map_odom.header.stamp = now_stamp
            tf_map_odom.header.frame_id = "map"
            tf_map_odom.child_frame_id = "odom"
            tf_map_odom.transform.rotation.w = 1.0
            tf_broadcaster.sendTransform(tf_map_odom)

            # odom -> base_link (GT pose)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now_stamp
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = rx
            tf_msg.transform.translation.y = ry
            tf_msg.transform.translation.z = rz
            tf_msg.transform.rotation.z = math.sin(ryaw / 2)
            tf_msg.transform.rotation.w = math.cos(ryaw / 2)
            tf_broadcaster.sendTransform(tf_msg)

            # base_link -> camera_link (static offset)
            tf_cam = TransformStamped()
            tf_cam.header.stamp = tf_msg.header.stamp
            tf_cam.header.frame_id = "base_link"
            tf_cam.child_frame_id = "camera_link"
            tf_cam.transform.translation.x = CAM_FWD
            tf_cam.transform.translation.z = CAM_UP
            tf_cam.transform.rotation.w = 1.0
            tf_broadcaster.sendTransform(tf_cam)

        # Publish depth pointcloud every 6 frames (~10Hz)
        if frame_count % 6 == 0:
            depth = cam.get_depth()
            if depth is not None and depth.size > 0:
                points = []
                for v in range(0, DEPTH_H, POINT_STEP):
                    for u in range(0, DEPTH_W, POINT_STEP):
                        d = float(depth[v, u])
                        if not (0.3 < d < 10.0 and np.isfinite(d)):
                            continue
                        # Camera frame: Z forward, X right, Y down
                        # -> base_link frame: X forward, Y left, Z up
                        cx = (u - CX) / FX * d
                        cy = (v - CY) / FY * d
                        # Transform camera -> base_link
                        px = d          # forward
                        py = -cx        # left
                        pz = -cy + CAM_UP  # up
                        points.append(struct.pack("fff", px, py, pz))

                if points:
                    pc = PointCloud2()
                    pc.header.stamp = node.get_clock().now().to_msg()
                    pc.header.frame_id = "base_link"
                    pc.height = 1
                    pc.width = len(points)
                    pc.fields = [
                        PointField(name="x", offset=0, datatype=7, count=1),
                        PointField(name="y", offset=4, datatype=7, count=1),
                        PointField(name="z", offset=8, datatype=7, count=1),
                    ]
                    pc.is_bigendian = False
                    pc.point_step = 12
                    pc.row_step = 12 * len(points)
                    pc.data = b"".join(points)
                    pc.is_dense = True
                    pc_pub.publish(pc)

        # Log every 10s
        if frame_count % 600 == 0:
            cmd_lin, cmd_ang = latest_cmd
            print(f"  t={sim_time:.0f}s | GT=({rx:.1f},{ry:.1f}) "
                  f"cmd=({cmd_lin:.2f},{cmd_ang:.2f})")
            log_f.write(f"{sim_time:.1f},{rx:.4f},{ry:.4f},{ryaw:.4f},"
                        f"{cmd_lin:.3f},{cmd_ang:.3f}\n")
            log_f.flush()

except KeyboardInterrupt:
    print("\nstopped")

log_f.close()
rx, ry, rz, ryaw = _get_husky_pose()
print(f"\nFinal GT: ({rx:.1f}, {ry:.1f})")
print(f"Log: {log_path}")

timeline.stop()
node.destroy_node()
rclpy.shutdown()
app.close()
