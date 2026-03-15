#!/usr/bin/env python3
import os
import sys
import time
import numpy as np

ISAAC_SIM_PATH = os.environ.get("ISAAC_SIM_PATH", "/opt/isaac-sim-6.0.0")
ROS2_LIB = os.path.join(ISAAC_SIM_PATH, "exts/isaacsim.ros2.core/jazzy/lib")
os.environ.setdefault("RMW_IMPLEMENTATION", 'rmw_fastrtps_cpp')
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = ROS2_LIB + ":" + os.environ.get("LD_LIBRARY_PATH", "")

from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import omni
import omni.kit.app
import omni.kit.commands
import omni.graph.core as og
import usdrt.Sdf
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf

SCENE_USD = "/workspace/simulation/isaac/assets/husky_outdoor_scene.usd"
ROBOT_PATH = "/husky"
BASE_LINK = f"{ROBOT_PATH}/Geometry/base_link"
CAM_LINK = f"{BASE_LINK}/top_plate_link/camera_realsense_bottom_screw_frame/camera_realsense_link"
CAM_PRIM = f"{CAM_LINK}/d435i_camera"
IMU_LINK = f"{BASE_LINK}/imu_link"

# enable extensions
manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["isaacsim.ros2.core", "isaacsim.ros2.nodes",
            "isaacsim.sensors.physics.nodes", "isaacsim.ros2.bridge"]:
    manager.set_extension_enabled_immediate(ext, True)
for _ in range(50):
    app.update()

# load scene
print("loading scene...")
omni.usd.get_context().open_stage(SCENE_USD)
for _ in range(30):
    app.update()

stage = omni.usd.get_context().get_stage()


# FIX 1: remove root_joint that locks robot to world
print("\nfix 1: removing root_joint FixedJoint...")
root_joint = stage.GetPrimAtPath(f"{ROBOT_PATH}/Physics/root_joint")
if root_joint.IsValid():
    stage.RemovePrim(f"{ROBOT_PATH}/Physics/root_joint")
    print("  removed /husky/Physics/root_joint")

    # the robot needs a rigid body on base_link for physics to work
    # check if base_link has RigidBodyAPI
    base_prim = stage.GetPrimAtPath(BASE_LINK)
    if not base_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(base_prim)
        print("  applied RigidBodyAPI to base_link")

    # set mass
    if not base_prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI.Apply(base_prim)
        mass_api.CreateMassAttr(46.0)  # husky is ~46kg
        print("  set base_link mass to 46 kg")
else:
    print("  root_joint not found, skipping")


# FIX 2: raise robot spawn height above ground
print("\nfix 2: setting robot spawn height...")
# husky wheel radius = 0.1651m, so base_link should be ~0.13m above ground
# (wheel center is 0.03282m above base_link origin)
# set the robot root xform to spawn slightly above ground
# set initial position on base_link (the physics rigid body root)
# PhysX uses this as the starting pose
base_prim = stage.GetPrimAtPath(BASE_LINK)
if base_prim.IsValid():
    xf = UsdGeom.Xformable(base_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(-20, 0, 0.3))
    print("  spawn position: (-20, 0, 0.3) on base_link")
# also set on Geometry scope for visual alignment before physics starts
robot_root = stage.GetPrimAtPath(f"{ROBOT_PATH}/Geometry")
if robot_root.IsValid():
    xf = UsdGeom.Xformable(robot_root)
    xf.ClearXformOpOrder()


# FIX 3: fix camera orientation
# print(f"DEBUG len(traj)={len(traj)}")
print("\nfix 3: fixing camera orientation...")
cam_prim = stage.GetPrimAtPath(CAM_PRIM)
if cam_prim.IsValid():
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    # USD camera default: looks -Z, up +Y
    # target: look +X (robot forward), up +Z (world up)
    # verified: rotateXYZ(90, 0, -90) maps:
    #   Rx(90): y->z, z->-y -> cam(-Z)->+Y, cam(+Y)->+Z
    #   Rz(-90): x->y, y->-x -> cam(+Y from prev)->-X... wrong?
    # actually for rotateXYZ, the matrix is Rz * Ry * Rx (applied right to left)
    # so the full transform on view dir (0,0,-1):
    #   Rx(90)(0,0,-1) = (0,1,0)
    #   Ry(0)(0,1,0) = (0,1,0)
    #   Rz(-90)(0,1,0) = (1,0,0) <- +X forward 
    # and on up dir (0,1,0):
    #   Rx(90)(0,1,0) = (0,0,1)
    #   Ry(0)(0,0,1) = (0,0,1)
    #   Rz(-90)(0,0,1) = (0,0,1) <- +Z up 
    xf.AddRotateXYZOp().Set(Gf.Vec3f(90, 0, -90))
    print("  camera rotated: look along +X (forward)")
    print("  view direction: +X (robot forward)")
else:
    print("  camera prim not found!")


# FIX 4: add dome light for sky
print("\nfix 4: adding dome light (sky)...")
dome_path = "/World/Environment/DomeLight"
dome = UsdLux.DomeLight.Define(stage, dome_path)
dome.CreateIntensityAttr(1000)
dome.CreateColorAttr(Gf.Vec3f(0.53, 0.70, 0.92))  # clear sky blue
dome.CreateTextureFormatAttr("latlong")
# no HDR texture available, but the color tint gives a blue sky
print("  dome light: sky blue, intensity 1000")

# reduce sun intensity now that we have ambient sky light
sun = stage.GetPrimAtPath("/World/Environment/SunLight")
if sun.IsValid():
    UsdLux.DistantLight(sun).GetIntensityAttr().Set(3000)
    print("  sun intensity reduced to 3000")


# SAVE fixed scene
print("\nsaving fixed scene...")
stage.Export(SCENE_USD)
print(f"  saved: {SCENE_USD}")


# CREATE IMU SENSOR (recreate with proper schema)
imu_sensor_path = f"{IMU_LINK}/imu_sensor"
imu_prim = stage.GetPrimAtPath(imu_sensor_path)
if imu_prim.IsValid():
    stage.RemovePrim(imu_sensor_path)
omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor", parent=IMU_LINK,
    sensor_period=1.0 / 250.0,
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
for _ in range(10):
    app.update()


# SET UP RENDER + ROS2 GRAPH
print("\nsetting up omnigraph for camera capture...")

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/CaptureGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("CreateRP.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PRIM)]),
            ("CreateRP.inputs:width", 640),
            ("CreateRP.inputs:height", 480),
            ("RGBPub.inputs:topicName", "camera/color/image_raw"),
            ("RGBPub.inputs:type", "rgb"),
            ("RGBPub.inputs:frameId", "camera_realsense_link"),
            ("DepthPub.inputs:topicName", "camera/depth/image_rect_raw"),
            ("DepthPub.inputs:type", "depth"),
            ("DepthPub.inputs:frameId", "camera_realsense_link"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateRP.inputs:execIn"),
            ("CreateRP.outputs:execOut", "RGBPub.inputs:execIn"),
            ("CreateRP.outputs:execOut", "DepthPub.inputs:execIn"),
            ("CreateRP.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
            ("CreateRP.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
        ],
    },
)

# also create a topdown camera for the overview shot
topdown_cam_path = "/World/TopDownCamera"
topdown_cam = UsdGeom.Camera.Define(stage, topdown_cam_path)
topdown_cam.CreateFocalLengthAttr(15.0)
topdown_cam.CreateHorizontalApertureAttr(20.0)
topdown_cam.CreateClippingRangeAttr(Gf.Vec2f(1.0, 200.0))
txf = UsdGeom.Xformable(topdown_cam)
txf.AddTranslateOp().Set(Gf.Vec3d(0, 0, 50))  # 50m above origin
txf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))  # looking down -Z = straight down

og.Controller.edit(
    {"graph_path": "/TopDownGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CreateRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ],
        keys.SET_VALUES: [
            ("CreateRP.inputs:cameraPrim", [usdrt.Sdf.Path(topdown_cam_path)]),
            ("CreateRP.inputs:width", 800),
            ("CreateRP.inputs:height", 600),
            ("RGBPub.inputs:topicName", "topdown/image_raw"),
            ("RGBPub.inputs:type", "rgb"),
            ("RGBPub.inputs:frameId", "world"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CreateRP.inputs:execIn"),
            ("CreateRP.outputs:execOut", "RGBPub.inputs:execIn"),
            ("CreateRP.outputs:renderProductPath", "RGBPub.inputs:renderProductPath"),
        ],
    },
)


# RUN SIMULATION AND CAPTURE FRAMES
print("\nstarting simulation...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# long warmup for RTX to initialize
print("warming up renderer (300 frames)...")
for _ in range(300):
    app.update()

# capture frames via rclpy
import rclpy
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node("frame_capture")

frames = {}

def make_cb(topic, key):
    def cb(msg):
        if key not in frames and len(msg.data) > 0:
            frames[key] = msg
    return cb

node.create_subscription(Image, "/camera/color/image_raw", make_cb("/camera/color/image_raw", "cam"), 10)
node.create_subscription(Image, "/topdown/image_raw", make_cb("/topdown/image_raw", "topdown"), 10)

# run for 5 seconds, capturing frames
# print("DEBUG: isaac sim step")
print("capturing frames (5s)...")
cam_count = 0
start = time.time()

def count_cb(msg):
    global cam_count
    cam_count += 1

node.create_subscription(Image, "/camera/color/image_raw", count_cb, 10)

while time.time() - start < 5.0:
    app.update()
    rclpy.spin_once(node, timeout_sec=0.001)

elapsed = time.time() - start
hz = cam_count / elapsed if elapsed > 0 else 0
print(f"\ncamera rate: {cam_count} frames in {elapsed:.1f}s = {hz:.1f} Hz")

# check robot position
xfc = UsdGeom.XformCache()
base = stage.GetPrimAtPath(BASE_LINK)
mat = xfc.GetLocalToWorldTransform(base)
pos = mat.ExtractTranslation()
print(f"robot position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

# save frames
for key, path in [("cam", "/tmp/isaac_fixed.ppm"), ("topdown", "/tmp/isaac_topdown.ppm")]:
    if key in frames:
        msg = frames[key]
        img = np.frombuffer(msg.data, dtype=np.uint8)
        channels = len(msg.data) // (msg.height * msg.width)
        img = img.reshape(msg.height, msg.width, channels)
        with open(path, "wb") as f:
            f.write(f"P6\n{msg.width} {msg.height}\n255\n".encode())
            f.write(img[:, :, :3].tobytes())
        print(f"saved {key}: {path} ({msg.width}x{msg.height})")
    else:
        print(f"no frame captured for {key}")

node.destroy_node()
rclpy.shutdown()
timeline.stop()
app.close()
print("\ndone")
