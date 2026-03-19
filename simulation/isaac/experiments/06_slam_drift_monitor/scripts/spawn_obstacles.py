"""
spawn/remove dynamic obstacles for navigation testing.
obstacles placed ON route trajectory with space for bypass.

usage:
  from spawn_obstacles import spawn_obstacles, remove_obstacles, get_obstacle_positions
  spawn_obstacles(stage, "road")
"""
import math
from pxr import UsdGeom, UsdPhysics, Gf, Sdf

# per-route obstacle definitions
# each group: cones on trajectory, one side open for bypass

OBSTACLES = {
    "road": {
        "cones": [
            # x=-50: 3 cones block center/north, wide gap south (y < -6)
            [(-50, -5.0), (-50, -4.0), (-50, -3.0)],
            # x=15: 3 cones block center/south, wide gap north (y > -1)
            [(15, -4.0), (15, -3.0), (15, -2.0)],
            # x=45: 3 cones block center, gap south (y < -2)
            [(45, -1.0), (45, 0.0), (45, 1.0)],
        ],
        "tent": (-20, 0.2),
    },
    "north": {
        "cones": [
            [(-70, 8.0), (-69, 9.0), (-68, 10.0), (-67, 11.0)],     # early forest, east open
            [(-20, 25.0), (-19, 26.0), (-18, 27.0)],                  # deep north, west open
            [(30, 15.0), (31, 14.0), (32, 13.0), (33, 12.0)],        # return path, south open
        ],
        "tent": (-45, 18.0),
    },
    "south": {
        "cones": [
            [(-65, -15.0), (-64, -16.0), (-63, -17.0), (-62, -18.0)],  # early south, west open
            [(-10, -30.0), (-9, -31.0), (-8, -30.0)],                   # deep south, east open
            [(25, -15.0), (26, -14.0), (27, -13.0)],                    # return, north open
        ],
        "tent": (-35, -22.0),
    },
}

_active_route = None


def _terrain_height(x, y):
    """same terrain function as run_husky_forest.py"""
    RWPS = [
        (-100,-7),(-95,-6),(-90,-4.5),(-85,-2.8),(-80,-1.5),(-75,-0.8),(-70,-0.5),
        (-65,-1),(-60,-2.2),(-55,-3.8),(-50,-5),(-45,-5.5),(-40,-5.2),(-35,-4),
        (-30,-2.5),(-25,-1),(-20,0.2),(-15,1.2),(-10,1.8),(-5,2),(0,1.5),(5,0.5),
        (10,-0.8),(15,-2.2),(20,-3.5),(25,-4.2),(30,-4),(35,-3),(40,-1.8),(45,-0.8),
        (50,-0.5),(55,-1),(60,-2),(65,-3.2),(70,-4.5),(75,-5),
    ]
    def road_y(x):
        if x <= RWPS[0][0]: return RWPS[0][1]
        if x >= RWPS[-1][0]: return RWPS[-1][1]
        for i in range(len(RWPS)-1):
            if RWPS[i][0] <= x <= RWPS[i+1][0]:
                t = (x - RWPS[i][0]) / (RWPS[i+1][0] - RWPS[i][0])
                return RWPS[i][1] + t * (RWPS[i+1][1] - RWPS[i][1])
        return 0

    h = 0.0
    h += 0.5 * math.sin(x * 0.018 + 0.5) * math.cos(y * 0.022 + 1.2)
    h += 0.35 * math.sin(x * 0.035 + 2.1) * math.sin(y * 0.03 + 0.7)
    h += 0.18 * math.sin(x * 0.07 + 3.3) * math.cos(y * 0.065 + 2.5)
    h += 0.12 * math.cos(x * 0.11 + 1.0) * math.sin(y * 0.09 + 4.0)
    h += 0.06 * math.sin(x * 0.5 + 0.7) * math.cos(y * 0.43 + 2.1)
    h += 0.04 * math.cos(x * 0.7 + 3.5) * math.sin(y * 0.6 + 0.4)
    h += 0.03 * math.sin(x * 1.0 + 1.2) * math.cos(y * 0.83 + 3.8)
    rd = abs(y - road_y(x))
    if rd < 4.0:
        h *= (rd / 4.0) ** 2
    if rd < 2.0:
        h -= 0.06 * (1.0 - rd / 2.0)
    return max(h, -0.5)


def _make_cone(stage, path, x, y):
    """traffic cone: orange cone with collision"""
    gz = _terrain_height(x, y)
    cone = UsdGeom.Cone.Define(stage, path)
    cone.CreateRadiusAttr(0.3)
    cone.CreateHeightAttr(1.0)
    cone.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(cone)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, gz + 0.5))
    UsdGeom.Gprim(cone).CreateDisplayColorAttr([(1.0, 0.4, 0.0)])  # orange
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(path))


def _make_tent(stage, path, x, y):
    """camping tent: triangular prism approximated by a scaled cube"""
    gz = _terrain_height(x, y)
    # tent body (cube scaled to tent shape)
    body_path = f"{path}/body"
    body = UsdGeom.Cube.Define(stage, body_path)
    body.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(body)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, gz + 0.7))
    xf.AddScaleOp().Set(Gf.Vec3f(2.0, 1.8, 1.4))
    UsdGeom.Gprim(body).CreateDisplayColorAttr([(0.2, 0.5, 0.2)])  # dark green
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body_path))

    # tent roof (flattened cube on top)
    roof_path = f"{path}/roof"
    roof = UsdGeom.Cube.Define(stage, roof_path)
    roof.CreateSizeAttr(1.0)
    xf2 = UsdGeom.Xformable(roof)
    xf2.AddTranslateOp().Set(Gf.Vec3d(x, y, gz + 1.5))
    xf2.AddScaleOp().Set(Gf.Vec3f(2.2, 2.0, 0.3))
    xf2.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 0))
    UsdGeom.Gprim(roof).CreateDisplayColorAttr([(0.15, 0.4, 0.15)])


def spawn_obstacles(stage, route="road"):
    """add obstacles for given route"""
    global _active_route
    _active_route = route
    obs = OBSTACLES[route]
    root = "/World/NavObstacles"
    stage.DefinePrim(root, "Xform")

    cone_idx = 0
    for group in obs["cones"]:
        for cx, cy in group:
            _make_cone(stage, f"{root}/cone_{cone_idx:02d}", cx, cy)
            cone_idx += 1
    print(f"  spawned {cone_idx} cones in {len(obs['cones'])} groups")

    tx, ty = obs["tent"]
    stage.DefinePrim(f"{root}/tent", "Xform")
    _make_tent(stage, f"{root}/tent", tx, ty)
    print(f"  spawned tent at ({tx}, {ty})")

    return cone_idx + 1


def remove_obstacles(stage):
    """remove all obstacles from scene"""
    root = "/World/NavObstacles"
    if stage.GetPrimAtPath(root).IsValid():
        stage.RemovePrim(root)
        print("  removed all navigation obstacles")


def get_obstacle_positions(route=None):
    """return list of (x, y, radius) for collision checking"""
    r = route or _active_route or "road"
    obs = OBSTACLES[r]
    positions = []
    for group in obs["cones"]:
        for cx, cy in group:
            positions.append((cx, cy, 0.5))
    tx, ty = obs["tent"]
    positions.append((tx, ty, 1.5))
    return positions
