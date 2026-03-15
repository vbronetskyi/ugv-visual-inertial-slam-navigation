"""
spawn/remove dynamic obstacles for navigation testing.
obstacles placed ON route trajectory with space for bypass.

usage:
  from spawn_obstacles import spawn_obstacles, remove_obstacles, get_obstacle_positions
  spawn_obstacles(stage, "road")
"""
import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

def _frange(start, stop, step):
    """Float range for dense cone walls."""
    vals = []
    v = start
    while v <= stop + 0.01:
        vals.append(round(v, 1))
        v += step
    return vals

# per-route obstacle definitions
# Barriers: dense cone walls (0.5m spacing) blocking part of road
# Robot MUST go around - cannot pass through

OBSTACLES = {
    "road": {
        "cones": [
            # BARRIER 1 (x=-50): cone wall blocks south half of road
            # Route y≈-4.8. Wall from y=-8 to y=-2.5, 1.0m spacing (halved density)
            # Bypass: north side y>-2 (3m free)
            [(-50, y) for y in _frange(-8.0, -2.5, 1.0)],
            # BARRIER 2 (x=15): cone wall blocks north half
            # Route y≈-2.0. Wall from y=-1 to y=+4, 1.0m spacing
            # Bypass: south side y<-1.5 (4m free)
            [(15, y) for y in _frange(-1.0, 4.0, 1.0)],
            # BARRIER 3 (x=45): cone wall blocks center
            # Route y≈-1.2. Wall from y=-3 to y=+1, 1.0m spacing
            # Bypass: south y<-3.5 (2m) or north y>+1.5 (3m)
            [(45, y) for y in _frange(-3.0, 1.0, 1.0)],
        ],
        "tent": (-20, 0.0),
    },
    "north": {
        "cones": [
            [(-70, 8.0), (-69, 9.0), (-68, 10.0), (-67, 11.0)],     # early forest, east open
            [(-20, 25.0), (-19, 26.0), (-18, 27.0)],                  # deep north, west open
            [(30, 15.0), (31, 14.0), (32, 13.0), (33, 12.0)],        # return path, south open
        ],
        "tent": (-45, 18.0),
    },
    "north_forest": {
        # exp 72 north-forest repeat obstacles (on outbound path):
        # - 2 cones in deep forest @ x=-45, route y≈+23
        # - 2 cones further east @ x=-10, route y≈+28
        # - tent at route exit (x≈0, y≈+24)
        # - 3 cones after tent @ x=+21, route y≈+3
        'cones': [
            [(-45, 22.5), (-45, 23.5)],
            [(-10, 27.5), (-10, 28.5)],
            [(21, 2.0), (21, 3.0), (21, 4.0)],
        ],
        "tent": (0.0, 24.0),
    },
    "south": {
        "cones": [
            # Obstacle A (x=-75, on route @ y~-24.6): 3-cone wall y=-24..-26
            [(-75, -24.0), (-75, -25.0), (-75, -26.0)],
            # Obstacle B (x=-18, on route @ y~-24.0): 2-cone wall y=-24..-25
            [(-18, -24.0), (-18, -25.0)],
            # Obstacle C (x=+5, on route @ y~-17.8): 4-cone wall y=-17..-20
            [(5, -17.0), (5, -18.0), (5, -19.0), (5, -20.0)],
        ],
        # Tent placed ON route between group 0 (x=-75) and group 1 (x=-18).
        # Route passes through (-45,-38) outbound - tent sits across the path.
        "tent": (-45.0, -38.0),
    },
    "04_nw_se": {
        # Repeat obstacles along 04 outbound (LT -> RB diagonal).
        # Placed on the teach path at 15/40/65/85% through outbound.
        "cones": [
            [(-65.0, 28.0), (-65.0, 29.5)],                  # cones group 1 - early forest
            [(4.0, -19.0), (4.0, -18.0)],                    # cones group 2 - mid outbound
            [(40.0, -27.0), (40.0, -28.0), (40.0, -29.0)],   # cones group 3 - approach to RB
        ],
        "tent": (-39.4, -4.5),
    },
    "05_ne_sw": {
        # Auto-generated obstacle config - 6 props along outbound
        "props": [
            {"kind": "bench", "x": +32.89, "y": +11.39, "yaw": -0.7854},
            {"kind": "barrel_medium", "x": -4.75, "y": +2.14},
            {"kind": "barrel_medium", "x": -4.75, "y": +3.34},
            {"kind": "barrel_medium", "x": -4.75, "y": +4.54},
            {"kind": "concrete_block_a", "x": -44.20, "y": -4.00, "yaw": +4.7124},
            {"kind": "dumpster_small", "x": -82.76, "y": -7.31},
        ],
    },
    "06_nw_ne": {
        # Auto-generated obstacle config - 6 props along outbound
        "props": [
            {"kind": "firehydrant", "x": -62.74, "y": +18.77},
            {"kind": "cardbox_large", "x": -38.07, "y": -4.89},
            {"kind": "cardbox_large", "x": -38.07, "y": -3.89},
            {"kind": "cardbox_large", "x": -38.07, "y": -2.89},
            {"kind": "railing", "x": -0.11, "y": +3.50, "yaw": +1.5833},
            {"kind": "dumpster_large", "x": +34.98, "y": +13.48},
        ],
    },
    "07_se_sw": {
        # Auto-generated obstacle config - 7 props along outbound
        "props": [
            {"kind": "trashcan", "x": +25.78, "y": -31.00},
            {"kind": "trashcan", "x": +25.78, "y": -30.00},
            {"kind": "trashcan", "x": +25.78, "y": -29.00},
            {"kind": "concrete_block_b", "x": -9.62, "y": -18.38, "yaw": +3.9270},
            {"kind": "barrel_large", "x": -46.37, "y": -8.60},
            {"kind": "barrel_large", "x": -46.37, "y": -7.40},
            {"kind": "bench", "x": -82.96, "y": -7.70, "yaw": -0.1709},
        ],
    },
    "08_nw_sw": {
        # Auto-generated obstacle config - 5 props along outbound
        # Color-diverse: grey trashcan / grey concrete / blue dumpster / wood bench / brown cardboxes
        "props": [
            {"kind": "trashcan",         "x": -100.80, "y": +13.31},
            {"kind": "trashcan",         "x": -100.80, "y": +14.41},
            {"kind": "concrete_block_a", "x": -101.55, "y": -10.95, "yaw": -0.1330},
            {"kind": "dumpster_small",   "x":  -99.03, "y": -25.54},
            {"kind": "bench",            "x":  -95.00, "y":  -0.50, "yaw": -0.5236},
        ],
    },
    "09_se_ne": {
        # Auto-generated obstacle config - 5 props along outbound
        "props": [
            {"kind": "cardbox_large", "x": +76.69, "y": -15.04},
            {"kind": "cardbox_large", "x": +76.69, "y": -13.94},
            {"kind": "dumpster_small", "x": +76.42, "y": +9.52},
            {"kind": "barrel_large", "x": +73.70, "y": +24.53},
            {"kind": "barrel_large", "x": +73.70, "y": +25.73},
        ],
    },
}

_active_route = None


def _terrain_height(x, y):
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
    # FIXME: hardcoded spawn, read from routes.json
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


# ─── Quality prop assets (downloaded from Isaac content) ──────────────────────
_PROPS_DIR = "/workspace/simulation/isaac/assets/props"

# key -> (usd_path, collision_radius_for_reporting_m, variant_type)
# Rivermark assets use the "_inst.usd" variant which has the actual geometry.
PROP_ASSETS = {
    # warehouse (single-file usd)
    "barrel_large":       (f"{_PROPS_DIR}/SM_BarelPlastic_A_01.usd",        0.4),
    "barrel_medium":      (f"{_PROPS_DIR}/SM_BarelPlastic_B_01.usd",        0.4),
    "barrel_small":       (f"{_PROPS_DIR}/SM_BarelPlastic_C_01.usd",        0.25),
    "crate_plastic":      (f"{_PROPS_DIR}/SM_BarelPlastic_D_01.usd",        0.2),
    "cardbox_large":      (f"{_PROPS_DIR}/SM_CardBoxA_01.usd",              0.35),
    "cardbox_cube":       (f"{_PROPS_DIR}/SM_CardBoxB_01.usd",              0.3),
    "cardbox_flat":       (f"{_PROPS_DIR}/SM_CardBoxC_01.usd",              0.3),
    "cardbox_small":      (f"{_PROPS_DIR}/SM_CardBoxD_01.usd",              0.2),
    # Rivermark (use *_inst.usd - geometry resides there)
    "concrete_block_a":   (f"{_PROPS_DIR}/concrete_block_01_inst.usd",      0.6),
    "concrete_block_b":   (f"{_PROPS_DIR}/concrete_block_02_inst.usd",      0.6),
    "dumpster_large":     (f"{_PROPS_DIR}/dumpster_lrg_01_inst.usd",        1.3),
    "dumpster_small":     (f"{_PROPS_DIR}/dumpster_blue_sm_01_inst.usd",    1.2),
    "trashcan":           (f"{_PROPS_DIR}/trashcan_cylinder_01_inst.usd",   0.4),
    "firehydrant":        (f"{_PROPS_DIR}/firehydrant04_inst.usd",          0.3),
    "railing":            (f"{_PROPS_DIR}/safety_railing_01_inst.usd",      1.2),  # x-length ~2.2 m
    "bench":              (f"{_PROPS_DIR}/bench_curved_01_inst.usd",        1.5),
}


def _spawn_prop(stage, prim_path, asset_key, x, y, yaw_rad=0.0, scale=1.0):
    """Reference an external USD asset + attach solid collision.

    - Wraps the reference in an Xform prim at (x, y, terrain_height(x,y))
    - Applies UsdPhysics.CollisionAPI + MeshCollisionAPI with approximation
      "convexHull" on every descendant mesh -> robot cannot pass through.
    """
    if asset_key not in PROP_ASSETS:
        print(f"    [warn] unknown prop '{asset_key}' - skipping")
        return None
    asset_path, r = PROP_ASSETS[asset_key]
    gz = _terrain_height(x, y)
    # OUTER Xform - our own transform ops (position / yaw / scale)
    outer = stage.DefinePrim(prim_path, "Xform")
    xf = UsdGeom.Xformable(outer)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, gz))
    xf.AddRotateZOp().Set(math.degrees(yaw_rad))
    if scale != 1.0:
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    # INNER prim references the asset - its own xformOps live here and
    # don't collide with ours on the outer.
    inner_path = f"{prim_path}/ref"
    inner = stage.DefinePrim(inner_path, "Xform")
    inner.GetReferences().AddReference(asset_path)
    # Apply solid collision to every mesh descendant of the referenced asset.
    _apply_collision_recursive(stage, inner_path)
    return outer


def _apply_collision_recursive(stage, root_path):
    """For every Mesh below root_path that doesn't already have physics,
    apply UsdPhysics.CollisionAPI + MeshCollisionAPI(convexHull).

    Warehouse props (SM_BarelPlastic_*, SM_CardBox*) ship with collision
    already baked in; Rivermark assets (concrete_block, dumpster, bench,
    trashcan, firehydrant, railing) do not. We only add where missing -
    never override existing collision to avoid PhysX cooking failures."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    added = 0
    try:
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                continue   # asset already has collision - don't touch
            UsdPhysics.CollisionAPI.Apply(prim)
            mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
            attr = mca.GetApproximationAttr()
            if not attr.IsAuthored():
                attr.Set("convexHull")
            added += 1
    except Exception as e:
        print(f"    [warn] collision apply partial failure under {root_path}: {e}")
    return added


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
    """add obstacles for given route.

    Route config may contain:
      - "cones":  list[list[(x,y)]]       legacy traffic cones
      - "tent":   (x,y) | None            legacy camping tent
      - "props":  list of prop dicts      new asset-backed obstacles:
                    { "kind": <key from PROP_ASSETS>, "x": float, "y": float,
                      "yaw": float (rad, default 0), "scale": float (default 1) }
    """
    global _active_route
    _active_route = route
    obs = OBSTACLES[route]
    root = "/World/NavObstacles"
    stage.DefinePrim(root, "Xform")

    cone_idx = 0
    for group in obs.get("cones", []):
        for cx, cy in group:
            _make_cone(stage, f"{root}/cone_{cone_idx:02d}", cx, cy)
            cone_idx += 1
    if cone_idx:
        # print(f">>> frame {i}/{n_frames}")
        print(f"  spawned {cone_idx} cones in {len(obs['cones'])} groups")

    if obs.get("tent"):
        tx, ty = obs["tent"]
        stage.DefinePrim(f"{root}/tent", "Xform")
        _make_tent(stage, f"{root}/tent", tx, ty)
        print(f"  spawned tent at ({tx}, {ty})")

    prop_count = 0
    for p in obs.get("props", []):
        path = f"{root}/prop_{prop_count:02d}_{p['kind']}"
        _spawn_prop(stage, path, p["kind"], p["x"], p["y"],
                    yaw_rad=p.get("yaw", 0.0), scale=p.get("scale", 1.0))
        prop_count += 1
        # print(f"DEBUG state={state} pose={pose}")
        print(f"  spawned prop '{p['kind']}' at ({p['x']}, {p['y']})")

    total = cone_idx + (1 if obs.get("tent") else 0) + prop_count
    return total


def remove_obstacles(stage):
    root = "/World/NavObstacles"
    if stage.GetPrimAtPath(root).IsValid():
        stage.RemovePrim(root)
        print("  removed all navigation obstacles")


def get_obstacle_positions(route=None):
    """return list of (x, y, radius) for collision checking"""
    r = route or _active_route or "road"
    obs = OBSTACLES[r]
    positions = []
    for group in obs.get("cones", []):
        for cx, cy in group:
            positions.append((cx, cy, 0.5))
    if obs.get("tent"):
        tx, ty = obs["tent"]
        positions.append((tx, ty, 1.5))
    for p in obs.get("props", []):
        _, r_m = PROP_ASSETS.get(p["kind"], (None, 0.5))
        positions.append((p["x"], p["y"], r_m))
    return positions
