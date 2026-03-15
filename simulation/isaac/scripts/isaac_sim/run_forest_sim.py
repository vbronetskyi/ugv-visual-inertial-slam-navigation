#!/usr/bin/env python3
"""
run the converted gazebo forest scene with husky a200 + ros2 sensor publishing

publishes:
  /camera/color/image_raw       sensor_msgs/Image (rgb8, 640x480)
  /camera/depth/image_rect_raw  sensor_msgs/Image (32FC1, 640x480)
  /imu/data                     sensor_msgs/Imu
  /odom                         nav_msgs/Odometry
  /tf                           tf2_msgs/TFMessage

usage:
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  /opt/isaac-sim-6.0.0/python.sh run_forest_sim.py [--duration 60]
"""
import os
import sys
import argparse
import time
import math
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=float, default=60.0)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import omni.replicator.core as rep
import usdrt.Sdf
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf, PhysicsSchemaTools

SCENE_USD = "/workspace/simulation/isaac/assets/husky_forest_scene.usd"
HUSKY_USD = "/workspace/simulation/isaac/assets/husky_d435i/husky_d435i.usda"
RENDERS = '/workspace/simulation/isaac/assets/renders/maps'

# -- enable ros2 --
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

eid = manager.get_enabled_extension_id("isaacsim.ros2.bridge")
if not eid:
    print("ERROR: ros2 bridge failed to load")
    app.close()
    sys.exit(1)
print(f"ros2 bridge: {eid}")

# -- load scene --
print(f"loading scene: {SCENE_USD}")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()
stage = omni.usd.get_context().get_stage()

# -- add husky --
print("adding husky...")
robot_prim = stage.DefinePrim("/World/Husky", "Xform")
robot_prim.GetReferences().AddReference(HUSKY_USD)
for _ in range(20):
    app.update()

# remove root_joint (it locks robot to world origin)
BASE_LINK = "/World/Husky/Geometry/base_link"
rj = stage.GetPrimAtPath("/World/Husky/Physics/root_joint")
if rj.IsValid():
    stage.RemovePrim("/World/Husky/Physics/root_joint")
    print("  removed root_joint")

base = stage.GetPrimAtPath(BASE_LINK)

# fix mass - URDF import left it at 0, PhysX can't move massless objects
mass_api = UsdPhysics.MassAPI.Apply(base)
mass_api.CreateMassAttr().Set(46.0)  # husky is ~46kg
print("  mass: 46 kg")

# add collision box so robot sits on ground plane
col_path = f"{BASE_LINK}/collision_box"
if not stage.GetPrimAtPath(col_path).IsValid():
    col = UsdGeom.Cube.Define(stage, col_path)
    col.CreateSizeAttr(1.0)
    col_xf = UsdGeom.Xformable(col)
    col_xf.AddScaleOp().Set(Gf.Vec3f(0.98, 0.57, 0.24))
    col_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.12))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(col_path))
    col.CreatePurposeAttr("guide")
    print("  added collision box")

# fix wheel drives - need damping for velocity control
# without damping, targetVelocity has no effect
for wname in ["front_left_wheel", "front_right_wheel",
              "rear_left_wheel", "rear_right_wheel"]:
    joint_path = f"/World/Husky/Physics/{wname}"
    joint_prim = stage.GetPrimAtPath(joint_path)
    if joint_prim.IsValid():
        drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
        if drive:
            drive.GetDampingAttr().Set(1000.0)    # velocity drive damping
            drive.GetStiffnessAttr().Set(0.0)     # 0 stiffness = pure velocity mode
            drive.GetMaxForceAttr().Set(100.0)     # reasonable force limit
print("  wheel drives: damping=1000, velocity mode")

# set spawn position via Geometry scope transform
geom = stage.GetPrimAtPath("/World/Husky/Geometry")
if geom.IsValid():
    xf = UsdGeom.Xformable(geom)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-105, 0, 0.3))
# print(f"DEBUG state={state} pose={pose}")
print("  spawn: (-105, 0, 0.3)")

# -- forward camera (d435i simulation) --
# static world camera that follows robot via script each frame
CAM_PATH = "/World/ForwardCamera"
def make_camera(path, focal, aperture, clip_near, clip_far, mat):
    """create or reset a camera prim with transform matrix"""
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateClippingRangeAttr(Gf.Vec2f(clip_near, clip_far))
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(mat)

# cameras at (-105, -8) - only position that renders reliably
# Replicator render product only works near the initial camera position
# moving cameras after creation causes black frames (Isaac Sim 6.0 bug)
tilt = math.radians(10)
cos_t, sin_t = math.cos(tilt), math.sin(tilt)
make_camera(CAM_PATH, 18.0, 20.955, 0.105, 100.0, Gf.Matrix4d(
     0,      -1,  0,     0,
     sin_t,   0,  cos_t, 0,
    -cos_t,   0,  sin_t, 0,
    -105,     0,  1.5,   1
))

CHASE_PATH = "/World/ChaseCamera"
chase_tilt = math.radians(30)
ct2, st2 = math.cos(chase_tilt), math.sin(chase_tilt)
make_camera(CHASE_PATH, 24.0, 20.955, 0.5, 200.0, Gf.Matrix4d(
     0,    -1,  0,    0,
     st2,   0,  ct2,  0,
    -ct2,   0,  st2,  0,
    -110,   0,  4,    1
))
print(f"  camera: {CAM_PATH}")

# -- create IMU sensor --
IMU_PARENT = "/World/Husky/Geometry/base_link/imu_link"
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor",
    parent=IMU_PARENT,
    sensor_period=1.0 / 250.0,
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
IMU_PATH = f"{IMU_PARENT}/imu_sensor"
# print(f"DEBUG: ran {len(ran)} waypoints")
print(f"  imu: {IMU_PATH}")

for _ in range(20):
    app.update()

# -- ros2 omnigraph --
print("\ncreating ros2 graph...")
keys = og.Controller.Keys

og.Controller.edit(
    {"graph_path": "/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            # chase camera
            ("CreateChaseRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("ChasePub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PubIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ],
        keys.SET_VALUES: [
            ("CreateRP.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PATH)]),
            ("CreateRP.inputs:width", 640),
            ("CreateRP.inputs:height", 480),
            ("RGBPub.inputs:topicName", "camera/color/image_raw"),
            ("RGBPub.inputs:type", "rgb"),
            ("RGBPub.inputs:frameId", "camera_link"),
            ("DepthPub.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPub.inputs:type", "depth"),
            ("DepthPub.inputs:frameId", "camera_link"),
            # chase camera
            ("CreateChaseRP.inputs:cameraPrim", [usdrt.Sdf.Path(CHASE_PATH)]),
            ("CreateChaseRP.inputs:width", 640),
            ("CreateChaseRP.inputs:height", 480),
            ("ChasePub.inputs:topicName", "chase_camera/image"),
            ("ChasePub.inputs:type", "rgb"),
            ("ChasePub.inputs:frameId", "base_link"),
            ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(IMU_PATH)]),
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
            ("OnPlaybackTick.outputs:tick", "CreateRP.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "CreateChaseRP.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PubTF.inputs:execIn"),
            ("CreateRP.outputs:execOut", "RGBPub.inputs:execIn"),
            ("CreateRP.outputs:execOut", "DepthPub.inputs:execIn"),
            ("CreateRP.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            # chase camera connections
            ("CreateChaseRP.outputs:execOut", "ChasePub.inputs:execIn"),
            ("CreateChaseRP.outputs:renderProductPath", "ChasePub.inputs:renderProductPath"),
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

# -- start simulation --
print(f"\nstarting simulation ({args.duration}s)...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# teleport robot to spawn point after physics starts
# PhysX ignores USD xform set before play - must use teleport API
SPAWN_X, SPAWN_Y, SPAWN_Z = -105.0, 0.0, 0.3

print("warming up renderer...")
for _ in range(300):
    app.update()

# teleport AFTER warmup - set xform every frame to fight PhysX
def get_xform_op(path):
    p = stage.GetPrimAtPath(path)
    if p.IsValid():
        ops = UsdGeom.Xformable(p).GetOrderedXformOps()
        return ops[0] if ops else None
    return None

# cameras stay at initial positions from make_camera()
# forward: (-105, -8, 1.5), chase: (-110, -8, 4)
for _ in range(100):
    app.update()

# -- replicator render products for manual RGB publishing --
# ROS2 CameraHelper rgb type produces black frames in PathTracing mode
# workaround: use Replicator annotator + publish via rclpy manually
rp_fwd = rep.create.render_product(CAM_PATH, (640, 480))
rp_chase = rep.create.render_product(CHASE_PATH, (640, 480))
ann_fwd = rep.AnnotatorRegistry.get_annotator("rgb")
ann_fwd.attach([rp_fwd])
ann_chase = rep.AnnotatorRegistry.get_annotator("rgb")
ann_chase.attach([rp_chase])

for _ in range(200):
    app.update()

for ann_obj, fpath, label in [
    (ann_fwd, "/tmp/isaac_cam_fwd.jpg", "forward"),
    (ann_chase, "/tmp/isaac_cam_chase.jpg", "chase"),
]:
    d = ann_obj.get_data()
    if d is not None and np.mean(d[:, :, :3]) > 3:
        from PIL import Image as PILImg
        PILImg.fromarray(d[:, :, :3]).resize((320, 240)).save(fpath, quality=60)
        PILImg.fromarray(d[:, :, :3]).save(f"{RENDERS}/forest_{label}.png")
        print(f"  {label} camera: mean={np.mean(d[:,:,:3]):.0f}, saved to {fpath}")
    else:
        print(f"  {label} camera: dark")

# -- verify ros2 topics --
import rclpy
from sensor_msgs.msg import Image as RosImage, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

rclpy.init()
node = rclpy.create_node("verifier")

# manual RGB publishers (workaround for CameraHelper black frame bug)
from sensor_msgs.msg import Image as RosImageMsg
from builtin_interfaces.msg import Time as RosTime
cam_pub = node.create_publisher(RosImageMsg, "/camera/color/image_raw", 10)
chase_pub = node.create_publisher(RosImageMsg, "/chase_camera/image", 10)
frame_count = [0]

def publish_rgb(annotator, publisher, frame_id="camera_link"):
    """grab frame from replicator annotator, publish as ROS2 Image"""
    d = annotator.get_data()
    if d is None:
        return
    rgb = d[:, :, :3].copy()  # drop alpha
    if np.mean(rgb) < 1:
        return
    msg = RosImageMsg()
    msg.header.frame_id = frame_id
    now = node.get_clock().now().to_msg()
    msg.header.stamp = now
    msg.height, msg.width = rgb.shape[0], rgb.shape[1]
    msg.encoding = "rgb8"
    msg.step = rgb.shape[1] * 3
    msg.data = rgb.tobytes()
    publisher.publish(msg)

topic_counts = {}

def make_cb(topic):
    def cb(msg):
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
    return cb

node.create_subscription(RosImage, "/camera/color/image_raw", make_cb("/camera/color/image_raw"), 10)
node.create_subscription(RosImage, "/camera/depth/image_rect_raw", make_cb("/camera/depth/image_rect_raw"), 10)
node.create_subscription(Imu, "/imu/data", make_cb("/imu/data"), 10)
node.create_subscription(Odometry, "/odom", make_cb("/odom"), 10)
node.create_subscription(TFMessage, "/tf", make_cb("/tf"), 10)

print("\nmeasuring topic rates (5s)...")
last_cam_msg = [None]
orig_cam_cb = make_cb("/camera/color/image_raw")
def cam_save_cb(msg):
    # TODO: tune per route
    orig_cam_cb(msg)
    if len(msg.data) > 0:
        last_cam_msg[0] = msg

# override camera sub to also save last frame
node.create_subscription(RosImage, "/camera/color/image_raw", cam_save_cb, 1)

t0 = time.time()
while time.time() - t0 < 5:
    app.update()
    rclpy.spin_once(node, timeout_sec=0.001)

elapsed = time.time() - t0
print("ros2 topics:")
for topic in sorted(topic_counts):
    hz = topic_counts[topic] / elapsed
    print(f"  {topic}: {topic_counts[topic]} msgs, {hz:.1f} Hz")

if not topic_counts:
    print("  no messages received")

# save ROS2 camera frame
if last_cam_msg[0] is not None:
    msg = last_cam_msg[0]
    img = np.frombuffer(msg.data, dtype=np.uint8)
    if len(img) > 0:
        try:
            ch = len(msg.data) // (msg.height * msg.width)
            img = img.reshape(msg.height, msg.width, ch)
            from PIL import Image as PILImg
            PILImg.fromarray(img[:, :, :3]).save(f"{RENDERS}/forest_ros2_camera.png")
            print(f"  ros2 camera frame: {msg.width}x{msg.height}, mean={np.mean(img):.0f}")
        except Exception as e:
            print(f"  ros2 camera frame error: {e}")
else:
    print("  no camera frame received via ros2")

# robot position
xfc = UsdGeom.XformCache()
mat = xfc.GetLocalToWorldTransform(base)
pos = mat.ExtractTranslation()
print(f"\nrobot position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.2f})")

# -- main loop with follow-cam --
# update static camera position each frame to track the robot
print(f"\nrunning for {args.duration}s... (ctrl+c to stop)")
sim_time = 5.0

fwd_op = get_xform_op(CAM_PATH)
chase_op = get_xform_op(CHASE_PATH)

# pre-compute tilt matrices
fwd_tilt = math.radians(10)
chase_tilt = math.radians(30)

# subscribe to /cmd_vel for click-to-drive from web UI
from geometry_msgs.msg import Twist
latest_cmd = [Twist()]
def cmd_vel_cb(msg):
    latest_cmd[0] = msg
node.create_subscription(Twist, "/cmd_vel", cmd_vel_cb, 10)

# cameras are STATIC - no follow-cam
# moving cameras after Replicator render_product creation causes black frames
# cameras show the initial view from spawn point

try:
    while sim_time < args.duration:
        app.update()
        rclpy.spin_once(node, timeout_sec=0.001)
        sim_time += 1.0 / 60.0

        # apply cmd_vel to robot wheels
        # husky skid-steer: left/right wheels get different velocities
        cmd = latest_cmd[0]
        wheel_r = 0.1651   # wheel radius
        track = 0.555      # track width
        v_left = (cmd.linear.x - cmd.angular.z * track / 2) / wheel_r
        v_right = (cmd.linear.x + cmd.angular.z * track / 2) / wheel_r
        # convert to degrees/sec - PhysX angular drive uses deg/s
        for wname, vel in [("front_left_wheel", v_left), ("front_right_wheel", v_right),
                           ("rear_left_wheel", v_left), ("rear_right_wheel", v_right)]:
            joint_path = f"/World/Husky/Physics/{wname}"
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
                if drive:
                    drive.GetTargetVelocityAttr().Set(float(math.degrees(vel)))

        if int(sim_time) % 10 == 0 and abs(sim_time - int(sim_time)) < 0.02:
            xfc = UsdGeom.XformCache()
            rp = xfc.GetLocalToWorldTransform(base).ExtractTranslation()
            print(f"  t={sim_time:.0f}s robot=({rp[0]:.1f}, {rp[1]:.1f}, {rp[2]:.2f})")
except KeyboardInterrupt:
    print("\nstopped by user")

node.destroy_node()
rclpy.shutdown()
timeline.stop()
app.close()
print("done")
