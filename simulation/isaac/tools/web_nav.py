#!/usr/bin/env python3
"""web UI for isaac sim husky navigation
camera feed + 2D map + click-to-drive + stop button

subscribes to isaac sim ros2 topics:
  /camera/color/image_raw, /odom, /tf

publishes:
  /cmd_vel (Twist)

usage:
  # start isaac sim first:
  /opt/isaac-sim-6.0.0/python.sh simulation/isaac/scripts/run_forest_sim.py &
  # then start web UI:
  python3 simulation/isaac/tools/web_nav.py
  # open http://localhost:8765
"""
import sys, os
# add isaac sim's bundled rclpy
ROS2_LIB = "/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib"
RCLPY = "/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/rclpy"
sys.path.insert(0, RCLPY)
sys.path.insert(0, f"{ROS2_LIB}/python3.12/site-packages")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
if ROS2_LIB not in os.environ.get("LD_LIBRARY_PATH", ""):
    os.environ["LD_LIBRARY_PATH"] = f"{ROS2_LIB}:{os.environ.get('LD_LIBRARY_PATH', '')}"

import threading, time, io, math, json
import numpy as np
from PIL import Image as PILImage, ImageDraw

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from flask import Flask, send_file, request, Response

app = Flask(__name__)

# -- global state --
camera_jpeg = None
robot_world = (0.0, 0.0, 0.0)  # (x, y, yaw)
nav_status = "idle"
goal_pos = None
robot_trail = []
MAX_TRAIL = 2000
lock = threading.Lock()

# world extents from gazebo SDF
WORLD_SIZE_X = 240.0
WORLD_SIZE_Y = 160.0
MAP_PX = 800
MAP_PX_Y = int(MAP_PX * WORLD_SIZE_Y / WORLD_SIZE_X)

# load model positions for map overlay
MODEL_POSITIONS = []
MODEL_COLORS = {
    "pine": (0, 100, 0), "oak": (34, 139, 34), "rock": (139, 119, 101),
    "house": (180, 160, 130), "barrel": (255, 165, 0),
    "fallen_oak": (90, 55, 25), "fallen_pine": (70, 45, 25),
    "shrub": (60, 130, 50),
}

_fallen_scales = {}

def load_models():
    try:
        with open("/tmp/gazebo_models.json") as f:
            models = json.load(f)
        # match thinning from convert_gazebo_to_isaac.py
        tree_skip = 0
        fallen_idx = 0
        west_sw = sorted([m for m in models if m["type"] in ("pine","oak")],
                         key=lambda m: math.hypot(m["x"]+120, m["y"]+80))
        west_nw = sorted([m for m in models if m["type"] in ("pine","oak")],
                         key=lambda m: math.hypot(m["x"]+120, m["y"]-80))
        remove_names = set(t["name"] for t in west_sw[:5]) | set(t["name"] for t in west_nw[:5])
        for m in models:
            color = MODEL_COLORS.get(m["type"], (128, 128, 128))
            if m["type"] in ("pine", "oak"):
                if m["name"] in remove_names:
                    continue
                tree_skip += 1
                if tree_skip % 3 == 0:
                    continue  # thinned
            if "fallen" in m["type"]:
                continue  # fallen trees deactivated in scene (exp 32)
                # scale = 1.2 if fallen_idx % 3 != 2 else 0.9
                # _fallen_scales[m["name"]] = scale
                # fallen_idx += 1
            MODEL_POSITIONS.append((m["name"], m["x"], m["y"], color, m["type"], m.get("yaw", 0)))
    except:
        pass

load_models()


# -- ros2 node --
class IsaacNavNode(Node):
    def __init__(self):
        super().__init__("isaac_web_nav")
        self.cam_sub = self.create_subscription(Image, "/camera/color/image_raw", self.cam_cb, 1)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._drive_cancel = False
        # timer to read position file even without odom topic
        self.create_timer(0.2, self._update_pos_from_file)

    def _img_to_jpeg(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        ch = len(msg.data) // (msg.height * msg.width)
        arr = arr.reshape(msg.height, msg.width, ch)
        img = PILImage.fromarray(arr[:, :, :3])
        img = img.resize((320, 240), PILImage.BILINEAR)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=50)
        return buf.getvalue()

    def cam_cb(self, msg):
        global camera_jpeg
        try:
            with lock:
                camera_jpeg = self._img_to_jpeg(msg)
        except:
            pass


    def odom_cb(self, msg):
        global robot_world, robot_trail
        self._update_pos_from_file()

    def _update_pos_from_file(self):
        global robot_world, robot_trail
        try:
            with open("/tmp/isaac_robot_pos.txt", "r") as f:
                parts = f.read().strip().split()
                x, y = float(parts[0]), float(parts[1])
        except:
            return  # file not ready yet
        # compute yaw from trail (no odom msg available)
        yaw = 0.0
        with lock:
            if robot_trail and len(robot_trail) >= 2:
                dx = x - robot_trail[-1][0]
                dy = y - robot_trail[-1][1]
                if math.hypot(dx, dy) > 0.1:
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = robot_world[2]  # keep previous yaw
            robot_world = (x, y, yaw)
            if not robot_trail or math.hypot(x - robot_trail[-1][0], y - robot_trail[-1][1]) > 0.5:
                robot_trail.append((x, y))
                if len(robot_trail) > MAX_TRAIL:
                    robot_trail = robot_trail[-MAX_TRAIL:]

    def send_goal(self, world_x, world_y):
        global nav_status, goal_pos
        goal_pos = (world_x, world_y)
        nav_status = f"driving to ({world_x:.1f}, {world_y:.1f})"
        # write goal to file - sim reads it and does the driving
        try:
            with open("/tmp/isaac_goal.txt", "w") as f:
                f.write(f"{world_x} {world_y}")
        except:
            pass

    def stop(self):
        global nav_status, goal_pos
        nav_status = "stopped"
        goal_pos = None
        try:
            with open("/tmp/isaac_goal.txt", "w") as f:
                f.write("stop")
        except:
            pass

    def reverse(self):
        global nav_status, goal_pos
        nav_status = "reversing..."
        goal_pos = None
        try:
            with open("/tmp/isaac_goal.txt", "w") as f:
                f.write("reverse")
        except:
            pass

    def reset(self):
        global nav_status, goal_pos, robot_trail
        nav_status = "resetting..."
        goal_pos = None
        with lock:
            robot_trail = []
        try:
            with open("/tmp/isaac_goal.txt", "w") as f:
                f.write("reset")
        except:
            pass


node = None


def render_map():
    with lock:
        rx, ry, ryaw = robot_world
        gp = goal_pos

    sx, sy = MAP_PX, MAP_PX_Y
    half_x = WORLD_SIZE_X / 2.0
    half_y = WORLD_SIZE_Y / 2.0

    def w2px(wx, wy):
        px = int((wx + half_x) / WORLD_SIZE_X * sx)
        py = int((half_y - wy) / WORLD_SIZE_Y * sy)
        return px, py

    img = PILImage.new("RGB", (sx, sy), (30, 40, 30))
    draw = ImageDraw.Draw(img)

    # road following original Gazebo route_1 trajectory
    route_pts = [
        (-100, -7.0), (-95, -6.0), (-90, -4.5), (-85, -2.8), (-80, -1.5),
        (-75, -0.8), (-70, -0.5), (-65, -1.0), (-60, -2.2), (-55, -3.8),
        (-50, -5.0), (-45, -5.5), (-40, -5.2), (-35, -4.0), (-30, -2.5),
        (-25, -1.0), (-20, 0.2), (-15, 1.2), (-10, 1.8), (-5, 2.0),
        (0, 1.5), (5, 0.5), (10, -0.8), (15, -2.2), (20, -3.5),
        (25, -4.2), (30, -4.0), (35, -3.0), (40, -1.8), (45, -0.8),
        (50, -0.5), (55, -1.0), (60, -2.0), (65, -3.2), (70, -4.5), (75, -5.0),
    ]
    for i in range(len(route_pts) - 1):
        p1 = w2px(route_pts[i][0], route_pts[i][1] + 1.5)
        p2 = w2px(route_pts[i+1][0], route_pts[i+1][1] - 1.5)
        lx, ly = min(p1[0], p2[0]), min(p1[1], p2[1])
        rx2, ry2 = max(p1[0], p2[0]), max(p1[1], p2[1])
        draw.rectangle([lx, ly, rx2, ry2], fill=(80, 65, 45))

    # scale: pixels per meter
    ppm = sx / WORLD_SIZE_X

    # models (sizes proportional to real world)
    for name, mx, my, color, mtype, yaw in MODEL_POSITIONS:
        px, py = w2px(mx, my)
        if not (0 <= px < sx and 0 <= py < sy):
            continue
        if mtype in ("pine", "oak"):
            # tree trunk ~0.5m radius (what robot drives between)
            r = max(2, int(0.5 * ppm))
            draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(200, 200, 200))
        elif mtype == "shrub":
            # shrub ~1m radius
            r = max(2, int(1.0 * ppm))
            draw.ellipse([px-r, py-r, px+r, py+r], fill=color, outline=(40, 90, 30))
        elif mtype == "rock":
            # rock ~0.7m
            r = max(2, int(0.7 * ppm))
            draw.polygon([(px, py-r), (px+r, py), (px, py+r), (px-r, py)], fill=color)
        elif mtype == "house":
            # house ~5x5m
            r = max(5, int(5.0 * ppm))
            draw.rectangle([px-r, py-r, px+r, py+r], fill=color, outline=(60, 40, 30), width=2)
        elif "fallen" in mtype:
            # fallen trees: actual size in sim is smaller than tree height
            # trunk visible part ~5-8m depending on scale
            scale = _fallen_scales.get(name, 1.0)
            trunk_len = 8.0 * scale
            half = max(3, int(trunk_len / 2 * ppm))
            w = max(2, int(0.4 * ppm))
            dx = int(half * math.cos(yaw))
            dy = int(-half * math.sin(yaw))
            draw.line([px-dx, py-dy, px+dx, py+dy], fill=color, width=w)
        elif mtype == "barrel":
            # barrel ~0.4m
            r = max(2, int(0.4 * ppm))
            draw.ellipse([px-r, py-r, px+r, py+r], fill=color)

    # trail
    with lock:
        trail = list(robot_trail)
    if len(trail) > 1:
        pts = [w2px(tx, ty) for tx, ty in trail]
        for i in range(1, len(pts)):
            draw.line([pts[i - 1], pts[i]], fill=(255, 200, 0), width=2)

    # goal
    if gp:
        gx, gy = w2px(gp[0], gp[1])
        draw.line([gx - 8, gy - 8, gx + 8, gy + 8], fill=(255, 0, 0), width=3)
        draw.line([gx - 8, gy + 8, gx + 8, gy - 8], fill=(255, 0, 0), width=3)

    # robot
    rpx, rpy = w2px(rx, ry)
    r = 10  # bigger robot dot
    draw.ellipse([rpx - r, rpy - r, rpx + r, rpy + r], fill=(0, 150, 255), outline=(255, 255, 255), width=3)
    dx = int(r * 2 * math.cos(ryaw))
    dy = int(-r * 2 * math.sin(ryaw))
    draw.line([rpx, rpy, rpx + dx, rpy + dy], fill=(255, 80, 80), width=2)

    # legend with proper icons
    y0 = 8
    legend_items = [
        ("Robot", (0, 150, 255), "circle"),
        ("Trail", (255, 200, 0), "line"),
        ("Goal", (255, 0, 0), "cross"),
        ("Pine tree", (0, 100, 0), "circle"),
        ("Oak tree", (34, 139, 34), "circle"),
        ("Shrub", (60, 130, 50), "circle_sm"),
        ("Fallen tree", (90, 55, 25), "line"),
        ("Rock", (139, 119, 101), "diamond"),
        ("Building", (180, 160, 130), "rect"),
        ("Barrel", (255, 165, 0), "circle_sm"),
        ("Road", (80, 65, 45), "rect"),
    ]
    # background
    draw.rectangle([4, 4, 110, 8 + len(legend_items) * 14 + 4], fill=(20, 30, 20, 200), outline=(60, 70, 60))
    for label, c, icon in legend_items:
        cx, cy = 13, y0 + 5
        if icon == "circle":
            draw.ellipse([cx-4, cy-4, cx+4, cy+4], fill=c)
        elif icon == "circle_sm":
            draw.ellipse([cx-3, cy-3, cx+3, cy+3], fill=c)
        elif icon == "line":
            draw.line([cx-5, cy, cx+5, cy], fill=c, width=2)
        elif icon == "cross":
            draw.line([cx-4, cy-4, cx+4, cy+4], fill=c, width=2)
            draw.line([cx-4, cy+4, cx+4, cy-4], fill=c, width=2)
        elif icon == "diamond":
            draw.polygon([(cx, cy-4), (cx+4, cy), (cx, cy+4), (cx-4, cy)], fill=c)
        elif icon == "rect":
            draw.rectangle([cx-4, cy-3, cx+4, cy+3], fill=c)
        draw.text((24, y0 - 1), label, fill=(220, 220, 220))
        y0 += 14

    # scale bar
    bar_m = 20
    bar_px = int(bar_m / WORLD_SIZE_X * sx)
    draw.rectangle([8, sy - 20, 8 + bar_px, sy - 16], fill=(255, 255, 255))
    draw.text((8, sy - 14), f"{bar_m}m", fill=(200, 200, 200))

    return img


# -- flask routes --
@app.route("/")
def index():
    return """<!DOCTYPE html>
<html>
<head>
<title>Isaac Sim - Husky Navigation</title>
<style>
  * { box-sizing: border-box; }
  body { background: #0d1117; color: #eee; font-family: 'Segoe UI', sans-serif; margin: 0; padding: 10px; }
  h2 { margin: 5px 0; color: #58a6ff; }
  .top { display: flex; align-items: center; gap: 12px; margin-bottom: 8px; }
  #status { font-size: 16px; color: #3fb950; font-weight: bold; }
  .btn { color: white; border: none; padding: 8px 18px; border-radius: 6px; cursor: pointer; font-weight: bold; }
  .btn-stop { background: #da3633; font-size: 16px; }
  .main { display: flex; gap: 10px; }
  .panel { background: #161b22; border: 1px solid #30363d; border-radius: 8px; padding: 8px; }
  .panel b { color: #8b949e; font-size: 13px; }
  .map-container { cursor: crosshair; }
  #cam-img { display: block; width: 100%; border: 1px solid #30363d; }
  .cam-panel { width: 360px; flex-shrink: 0; }
  .info { color: #8b949e; font-size: 12px; margin-top: 4px; }
</style>
</head>
<body>
<h2>Isaac Sim &mdash; Husky A200 Navigation</h2>
<div class="top">
  <span id="status">idle</span>
  <button class="btn btn-stop" onclick="stopRobot()">STOP</button>
  <button class="btn" onclick="reverseRobot()" style="background:#e6a700">REVERSE</button>
  <button class="btn" onclick="resetRobot()" style="background:#1f6feb">RESET</button>
</div>
<div class="main">
  <div class="panel" style="flex:1">
    <b>MAP</b> &mdash; click to send robot
    <div class="map-container">
      <img id="map-img" src="/map.png" onclick="clickMap(event)" style="width:100%;cursor:crosshair">
    </div>
    <div class="info" id="map-info">Click on map to navigate</div>
  </div>
  <div class="panel cam-panel">
    <b>FRONT CAMERA (D435i)</b>
    <img id="cam-img" src="/camera.jpg">
    <div class="info" id="cam-info">waiting for camera...</div>
  </div>
</div>
<script>
function refreshMap() {
  document.getElementById('map-img').src = '/map.png?' + Date.now();
}
function refreshCam() {
  document.getElementById('cam-img').src = '/camera.jpg?' + Date.now();
}
function refreshStatus() {
  fetch('/status').then(r => r.json()).then(d => {
    document.getElementById('status').textContent = d.status;
    document.getElementById('cam-info').textContent =
      'robot: (' + d.x.toFixed(1) + ', ' + d.y.toFixed(1) + ')';
  });
}
setInterval(refreshMap, 1000);
setInterval(refreshCam, 150);
setInterval(refreshStatus, 500);

function clickMap(e) {
  var img = e.target;
  var rect = img.getBoundingClientRect();
  var px = (e.clientX - rect.left) / rect.width;
  var py = (e.clientY - rect.top) / rect.height;
  var wx = px * 240 - 120;
  var wy = 80 - py * 160;
  fetch('/goal?x=' + wx.toFixed(1) + '&y=' + wy.toFixed(1));
  document.getElementById('map-info').textContent =
    'Goal: (' + wx.toFixed(1) + ', ' + wy.toFixed(1) + ')';
}
function stopRobot() { fetch('/stop'); }
function reverseRobot() { fetch('/reverse'); document.getElementById('status').textContent = 'reversing...'; }
function resetRobot() { fetch('/reset'); document.getElementById('status').textContent = 'resetting...'; }
</script>
</body>
</html>"""


@app.route("/map.png")
def map_png():
    img = render_map()
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return send_file(buf, mimetype="image/png")


@app.route("/camera.jpg")
def camera_jpg():
    # NOTE: relies on scripts/common/ being on path
    # file first (atomic write from sim), then ROS2 fallback
    try:
        with open("/tmp/isaac_cam_fwd.jpg", "rb") as f:
            data = f.read()
        if len(data) > 100:
            return Response(data, mimetype="image/jpeg")
    except:
        pass
    with lock:
        data = camera_jpeg
    if data:
        return Response(data, mimetype="image/jpeg")
    return Response(b"", mimetype="image/jpeg", status=204)




@app.route("/status")
def status():
    with lock:
        rx, ry, ryaw = robot_world
    return {"status": nav_status, "x": rx, "y": ry, "yaw": ryaw}


@app.route("/goal")
def goal():
    x = float(request.args.get("x", 0))
    y = float(request.args.get("y", 0))
    if node:
        node.send_goal(x, y)
    return {"ok": True, "x": x, "y": y}


@app.route("/stop")
def stop():
    if node:
        node.stop()
    return {"ok": True}


@app.route("/reverse")
def reverse():
    if node:
        node.reverse()
    return {"ok": True}


@app.route("/reset")
def reset():
    if node:
        node.reset()
    return {"ok": True}


def ros_spin():
    global node
    rclpy.init()
    node = IsaacNavNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    print("starting ros2 subscriber thread...")
    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()
    time.sleep(1)
    print("starting web server on http://localhost:8765")
    app.run(host="0.0.0.0", port=8765, threaded=True)
