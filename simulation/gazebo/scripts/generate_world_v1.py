#!/usr/bin/env python3
# v1 world generator - 100x100m, replaced by v2 (220x150m)
"""
Procedural outdoor world generator V1 - small scenario map

Layout (100x100m):
  FOREST (x<-10)  ->  OPEN FIELD (-10..15)  ->  VILLAGE (x>15)
  Robot spawns at (-40, 0) in the forest
  Destination: village at (~35, 0)
  Winding dirt trail connects the two

Features:
  - FBM noise heightmap (513x513) with zone-based amplitude
  - Trail carved into terrain (depression + flattened)
  - Dense forest (40 trees), sparse transition, village with buildings
  - 6 inline buildings (2 ruined) with box geometry
  - 3-layer textures: grass, dirt, rock (512x512 with normals)
  - Heightmap collision (robot drives ON terrain)

Usage:
    python3 generate_world.py [--seed 42] [--size 100] [--max-height 5]
"""

import argparse
import math
import struct
import zlib
from pathlib import Path

import numpy as np

# Paths
SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent / "src" / "ugv_gazebo"
WORLDS_DIR = PKG_DIR / "worlds"
MEDIA_DIR = WORLDS_DIR / "media" / "materials" / "textures"
HEIGHTMAP_PATH = WORLDS_DIR / "heightmap.png"
SDF_PATH = WORLDS_DIR / "outdoor_terrain.sdf"

# Scenario
SPAWN_POS = (-40.0, 0.0)

TRAIL_WAYPOINTS = [
    (-40, 0), (-33, 6), (-26, 3), (-18, -5), (-10, -2),
    (-2, 4), (6, 1), (14, -4), (22, 1), (30, 3), (35, 0),
]
TRAIL_WIDTH = 3.5       # metres
TRAIL_BLEND = 2.5       # blend edge
TRAIL_DEPRESSION = 0.25 # metres lower than surroundings

# village buildings - Fuel models with realistic meshes
BUILDINGS = [
    {"name": "house_1",     "x": 24, "y": -10, "yaw": 0.2,  "footprint": 12,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 1"},
    {"name": "collapsed_house_1", "x": 40, "y": 8, "yaw": -0.3, "footprint": 12,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Collapsed House"},
    {"name": "house_2",     "x": 42, "y": -8,  "yaw": 1.0,  "footprint": 12,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 3"},
    {"name": "collapsed_house_2", "x": 28, "y": 14, "yaw": 0.8, "footprint": 12,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Collapsed House"},
]

# village accessories - Fuel includes
VILLAGE_EXTRAS = [
    # lamp posts - some still working, village had power once
    {"name": "lamp_post_1",  "x": 23, "y": -2,  "yaw": 0.0,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"},
    {"name": "lamp_post_2",  "x": 36, "y": 0,   "yaw": 0.0,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"},
    # dumpsters - abandoned, nobody's collecting
    {"name": "dumpster_1",   "x": 26, "y": -14, "yaw": 0.8,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster"},
    {"name": "dumpster_2",   "x": 44, "y": 5,   "yaw": 2.1,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster"},
    # debris near collapsed buildings
    {"name": "wall_debris_1", "x": 38, "y": 11, "yaw": 0.5,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    {"name": "wall_debris_2", "x": 42, "y": 6,  "yaw": 2.3,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    {"name": "wall_debris_3", "x": 30, "y": 17, "yaw": 1.1,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    # fire hydrants
    {"name": "fire_hydrant_1","x": 30, "y": 2,  "yaw": 1.5,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire hydrant"},
]

# object placement by zone
ZONE_MODELS = [
    # dense forest (left side)
    {"name": "Oak tree",  "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 22, "radius": 2.0, "x_range": (-55, -8), "y_range": (-50, 50)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 18, "radius": 1.8, "x_range": (-55, -8), "y_range": (-50, 50)},
    # sparse transition
    {"name": "Oak tree",  "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 5, "radius": 2.0, "x_range": (-8, 18), "y_range": (-50, 50)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 3, "radius": 1.8, "x_range": (-8, 18), "y_range": (-50, 50)},
    # rocks everywhere
    {"name": "Falling Rock 1", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling Rock 1",
     "count": 15, "radius": 1.0, "x_range": (-55, 55), "y_range": (-50, 50)},
    # village area objects
    {"name": "Construction Barrel", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Barrel",
     "count": 6, "radius": 0.5, "x_range": (18, 48), "y_range": (-25, 25)},
    {"name": "Construction Cone", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone",
     "count": 5, "radius": 0.3, "x_range": (18, 48), "y_range": (-25, 25)},
    # trees around/behind village
    {"name": "Oak tree",  "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 8, "radius": 2.0, "x_range": (20, 55), "y_range": (-50, -20)},
    {"name": "Oak tree",  "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 8, "radius": 2.0, "x_range": (20, 55), "y_range": (20, 50)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 6, "radius": 1.8, "x_range": (46, 55), "y_range": (-30, 30)},
    # far end trees (beyond village)
    {"name": "Oak tree",  "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 5, "radius": 2.0, "x_range": (48, 55), "y_range": (-40, 40)},
    {"name": "Falling Rock 1", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling Rock 1",
     "count": 4, "radius": 1.0, "x_range": (45, 55), "y_range": (-40, 40)},
]

MIN_DISTANCE = 2.0
MAX_SLOPE_DEG = 25.0


# Noise

def _smooth_noise(size, scale, rng):
    grid_n = max(2, int(size / scale) + 2)
    grid = rng.random((grid_n, grid_n)).astype(np.float64)
    coords = np.linspace(0, grid_n - 1.001, size)
    xi = np.floor(coords).astype(int)
    xf = coords - xi
    xi = np.clip(xi, 0, grid_n - 2)
    xf_s = xf * xf * (3.0 - 2.0 * xf)
    yi_2d, xi_2d = np.meshgrid(xi, xi, indexing="ij")
    yf_2d, xf_2d = np.meshgrid(xf_s, xf_s, indexing="ij")
    c00 = grid[yi_2d, xi_2d]
    c10 = grid[yi_2d, xi_2d + 1]
    c01 = grid[yi_2d + 1, xi_2d]
    c11 = grid[yi_2d + 1, xi_2d + 1]
    top = c00 * (1 - xf_2d) + c10 * xf_2d
    bot = c01 * (1 - xf_2d) + c11 * xf_2d
    return top * (1 - yf_2d) + bot * yf_2d


def fbm(size, octaves, persistence, lacunarity, base_scale, rng):
    result = np.zeros((size, size), dtype=np.float64)
    amp, scale = 1.0, base_scale
    for _ in range(octaves):
        result += amp * _smooth_noise(size, scale, rng)
        amp *= persistence
        scale /= lacunarity
    result -= result.min()
    mx = result.max()
    if mx > 0:
        result /= mx
    return result


# Heightmap

def _dist_to_polyline(xx, yy, pts):
    """Min distance from each pixel to nearest trail segment"""
    min_d = np.full_like(xx, 1e10)
    interp_t = np.zeros_like(xx)  # parametric position along trail
    best_seg = np.zeros_like(xx, dtype=int)
    total_len = 0.0
    seg_starts = [0.0]
    for i in range(len(pts) - 1):
        total_len += math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
        seg_starts.append(total_len)

    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-6:
            continue
        t = np.clip(((xx - ax) * dx + (yy - ay) * dy) / seg_len_sq, 0, 1)
        px, py = ax + t * dx, ay + t * dy
        d = np.sqrt((xx - px) ** 2 + (yy - py) ** 2)
        closer = d < min_d
        min_d = np.where(closer, d, min_d)
        # global parametric position
        global_t = (seg_starts[i] + t * math.sqrt(seg_len_sq)) / max(total_len, 1)
        interp_t = np.where(closer, global_t, interp_t)
    return min_d, interp_t


def generate_heightmap(size_px, terrain_size, max_height, rng):
    half = terrain_size / 2
    x = np.linspace(-half, half, size_px)
    xx, yy = np.meshgrid(x, x)

    # base FBM terrain
    base = fbm(size_px, 6, 0.50, 2.0, size_px / 3, rng)

    # domain warping for natural look
    wx = fbm(size_px, 4, 0.45, 2.0, size_px / 4, rng) - 0.5
    wy = fbm(size_px, 4, 0.45, 2.0, size_px / 4, rng) - 0.5
    iy, ix = np.mgrid[0:size_px, 0:size_px]
    wxi = np.clip((ix + wx * 15).astype(int), 0, size_px - 1)
    wyi = np.clip((iy + wy * 15).astype(int), 0, size_px - 1)
    hmap = base[wyi, wxi]

    # zone-based amplitude: forest=full, open=medium, village=low
    zone_amp = np.clip(0.3 + 0.7 * (1.0 - (xx + 10) / 40), 0.25, 1.0)
    hmap *= zone_amp

    # normalize to [0, max_height]
    hmap -= hmap.min()
    mx = hmap.max()
    if mx > 0:
        hmap = hmap / mx * max_height

    # flatten spawn area (-40, 0)
    dist_spawn = np.sqrt((xx - SPAWN_POS[0]) ** 2 + (yy - SPAWN_POS[1]) ** 2)
    blend_spawn = np.clip((dist_spawn - 5) / 4, 0, 1)
    sp_r = int((SPAWN_POS[1] + half) / (terrain_size / (size_px - 1)))
    sp_c = int((SPAWN_POS[0] + half) / (terrain_size / (size_px - 1)))
    sp_r = max(0, min(sp_r, size_px - 1))
    sp_c = max(0, min(sp_c, size_px - 1))
    spawn_h = hmap[sp_r, sp_c]
    hmap = spawn_h + (hmap - spawn_h) * blend_spawn

    # flatten village center (35, 0) - larger area
    vc = (35.0, 0.0)
    dist_village = np.sqrt((xx - vc[0]) ** 2 + (yy - vc[1]) ** 2)
    blend_village = np.clip((dist_village - 12) / 6, 0, 1)
    vr = int((vc[1] + half) / (terrain_size / (size_px - 1)))
    vc_col = int((vc[0] + half) / (terrain_size / (size_px - 1)))
    vr = max(0, min(vr, size_px - 1))
    vc_col = max(0, min(vc_col, size_px - 1))
    village_h = hmap[vr, vc_col]
    hmap = village_h + (hmap - village_h) * blend_village

    # flatten and depress trail
    trail_dist, trail_t = _dist_to_polyline(xx, yy, TRAIL_WAYPOINTS)
    # trail height: interpolated between spawn and village
    trail_h = spawn_h + trail_t * (village_h - spawn_h)
    # blend: within trail_width -> flatten to trail_h, depress a bit
    blend_trail = np.clip((trail_dist - TRAIL_WIDTH / 2) / TRAIL_BLEND, 0, 1)
    target_h = trail_h - TRAIL_DEPRESSION
    hmap = target_h + (hmap - target_h) * blend_trail

    # flatten building footprints
    res = terrain_size / (size_px - 1)
    for bld in BUILDINGS:
        bx, by = bld["x"], bld["y"]
        dist_b = np.sqrt((xx - bx) ** 2 + (yy - by) ** 2)
        footprint_r = bld["footprint"] / 2 + 1.0
        blend_b = np.clip((dist_b - footprint_r) / 2.0, 0, 1)
        br = int((by + half) / res)
        bc = int((bx + half) / res)
        br = max(0, min(br, size_px - 1))
        bc = max(0, min(bc, size_px - 1))
        bh = hmap[br, bc]
        hmap = bh + (hmap - bh) * blend_b

    return hmap


def slope_map(hmap, terrain_size):
    res = terrain_size / (hmap.shape[0] - 1)
    gy, gx = np.gradient(hmap, res)
    return np.degrees(np.arctan(np.sqrt(gx ** 2 + gy ** 2)))


# PNG writers

def _png_chunk(ctype, data):
    c = ctype + data
    return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)


def write_png_gray16(path, data):
    h, w = data.shape
    d = data.astype(np.float64)
    d -= d.min()
    mx = d.max()
    if mx > 0:
        d = d / mx * 65535
    px = d.astype(np.uint16)
    raw = b""
    for r in range(h):
        raw += b"\x00"
        for c in range(w):
            raw += struct.pack(">H", int(px[r, c]))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(_png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0)))
        f.write(_png_chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(_png_chunk(b"IEND", b""))
    print(f"  {path.name}: {w}x{h}, {path.stat().st_size // 1024} KB")


def write_png_rgb(path, img):
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w, _ = img.shape
    raw = b""
    for r in range(h):
        raw += b"\x00" + img[r].tobytes()
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(_png_chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)))
        f.write(_png_chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(_png_chunk(b"IEND", b""))
    print(f"  {path.name}: {w}x{h}, {path.stat().st_size // 1024} KB")


# Textures

def _clamp_u8(a):
    return np.clip(a, 0, 255).astype(np.uint8)


def _normal_map(height, strength=1.0):
    gy, gx = np.gradient(height)
    gx *= strength; gy *= strength
    nz = np.ones_like(gx)
    ln = np.sqrt(gx**2 + gy**2 + nz**2)
    return np.stack([
        _clamp_u8((-gx/ln*0.5+0.5)*255),
        _clamp_u8((-gy/ln*0.5+0.5)*255),
        _clamp_u8((nz/ln*0.5+0.5)*255),
    ], axis=-1)


def generate_textures(rng, sz=512):
    MEDIA_DIR.mkdir(parents=True, exist_ok=True)

    # grass
    n1 = fbm(sz, 5, 0.55, 2.0, sz/4, rng)
    n2 = fbm(sz, 4, 0.50, 2.0, sz/8, rng)
    n3 = fbm(sz, 3, 0.40, 2.0, sz/16, rng)
    r = _clamp_u8(55 + n1*50 + n2*25 - n3*15)
    g = _clamp_u8(110 + n1*60 + n2*30 + n3*15)
    b = _clamp_u8(35 + n1*25 + n2*10)
    write_png_rgb(MEDIA_DIR / "grass_diffusespecular.png", np.stack([r,g,b], -1))
    write_png_rgb(MEDIA_DIR / "grass_normal.png", _normal_map(n1*0.6+n2*0.3+n3*0.1, 2.0))

    # dirt
    n1 = fbm(sz, 5, 0.50, 2.0, sz/4, rng)
    n2 = fbm(sz, 4, 0.45, 2.0, sz/8, rng)
    peb = (rng.random((sz,sz)) > 0.985).astype(np.float64)
    peb = np.clip(peb + np.roll(peb,1,0)*0.5, 0, 1)
    r = _clamp_u8(125 + n1*40 + n2*20 + peb*40)
    g = _clamp_u8(100 + n1*30 + n2*15 + peb*30)
    b = _clamp_u8(70 + n1*20 + n2*10 + peb*20)
    write_png_rgb(MEDIA_DIR / "dirt_diffusespecular.png", np.stack([r,g,b], -1))
    write_png_rgb(MEDIA_DIR / "dirt_normal.png", _normal_map(n1*0.5+n2*0.3+peb*0.05, 2.5))

    # rock
    n1 = fbm(sz, 5, 0.55, 2.0, sz/4, rng)
    n2 = fbm(sz, 4, 0.50, 2.0, sz/6, rng)
    crack = fbm(sz, 4, 0.60, 2.5, sz/8, rng)
    cracks = np.clip(1.0 - np.abs(crack-0.5)*6, 0, 1) * 0.4
    bg = 115 + n1*40
    r = _clamp_u8(bg + n2*15 - cracks*60)
    g = _clamp_u8(bg + n2*12 - cracks*55)
    b = _clamp_u8(bg + n2*10 - cracks*50)
    write_png_rgb(MEDIA_DIR / "rock_diffusespecular.png", np.stack([r,g,b], -1))
    write_png_rgb(MEDIA_DIR / "rock_normal.png", _normal_map(n1*0.4+n2*0.3-cracks*0.3, 3.0))

    print(f"  3 texture sets at {sz}x{sz}")


# Object placement

def _on_trail(x, y):
    for i in range(len(TRAIL_WAYPOINTS) - 1):
        ax, ay = TRAIL_WAYPOINTS[i]
        bx, by = TRAIL_WAYPOINTS[i + 1]
        dx, dy = bx - ax, by - ay
        sl2 = dx*dx + dy*dy
        if sl2 < 1e-6: continue
        t = max(0, min(1, ((x-ax)*dx + (y-ay)*dy) / sl2))
        if math.hypot(x - (ax+t*dx), y - (ay+t*dy)) < (TRAIL_WIDTH/2 + 1.5):
            return True
    return False


def _near_building(x, y):
    for b in BUILDINGS:
        if math.hypot(x - b["x"], y - b["y"]) < b["footprint"]/2 + 3.0:
            return True
    for e in VILLAGE_EXTRAS:
        if math.hypot(x - e["x"], y - e["y"]) < 2.0:
            return True
    return False


def _near_spawn(x, y):
    return math.hypot(x - SPAWN_POS[0], y - SPAWN_POS[1]) < 6.0


def place_objects(hmap, slopes, terrain_size, rng):
    placed = []
    half = terrain_size / 2
    res = terrain_size / (hmap.shape[0] - 1)
    counters = {}  # global counter per model name

    def get_h(x, y):
        c = int((x + half) / res); r = int((y + half) / res)
        c = max(0, min(c, hmap.shape[1]-1)); r = max(0, min(r, hmap.shape[0]-1))
        return float(hmap[r, c])

    def get_s(x, y):
        c = int((x + half) / res); r = int((y + half) / res)
        c = max(0, min(c, slopes.shape[1]-1)); r = max(0, min(r, slopes.shape[0]-1))
        return float(slopes[r, c])

    def too_close(x, y):
        for p in placed:
            if math.hypot(x - p["x"], y - p["y"]) < MIN_DISTANCE:
                return True
        return False

    for mdef in ZONE_MODELS:
        name = mdef["name"]
        counters.setdefault(name, 0)
        x_lo, x_hi = mdef["x_range"]
        y_lo, y_hi = mdef["y_range"]
        count, attempts = 0, 0

        while count < mdef["count"] and attempts < mdef["count"] * 300:
            attempts += 1
            x = rng.uniform(x_lo, x_hi)
            y = rng.uniform(y_lo, y_hi)
            if _near_spawn(x, y): continue
            if _on_trail(x, y): continue
            if _near_building(x, y): continue
            if get_s(x, y) > MAX_SLOPE_DEG: continue
            if too_close(x, y): continue

            counters[name] += 1
            placed.append({
                "x": x, "y": y, "z": get_h(x, y),
                "yaw": rng.uniform(0, 2*math.pi),
                "name": f"{name}_{counters[name]}",
                "uri": mdef["uri"], "type": name,
            })
            count += 1

        print(f"  {name} [{x_lo:.0f}..{x_hi:.0f}]: {count}/{mdef['count']}")

    return placed


# Terrain collision mesh

def generate_collision_stl(hmap, terrain_size, z_off, step=4):
    """Downsampled STL mesh from heightmap for ODE collision"""
    hmap_ds = hmap[::step, ::step]
    rows, cols = hmap_ds.shape
    half = terrain_size / 2
    xs = np.linspace(-half, half, cols)
    ys = np.linspace(-half, half, rows)
    xx, yy = np.meshgrid(xs, ys)
    zz = hmap_ds + z_off

    stl_path = WORLDS_DIR / "terrain_collision.stl"
    n_tri = (rows - 1) * (cols - 1) * 2
    with open(stl_path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', n_tri))
        for r in range(rows - 1):
            for c in range(cols - 1):
                verts = [
                    (xx[r,c], yy[r,c], zz[r,c]),
                    (xx[r,c+1], yy[r,c+1], zz[r,c+1]),
                    (xx[r+1,c], yy[r+1,c], zz[r+1,c]),
                    (xx[r+1,c+1], yy[r+1,c+1], zz[r+1,c+1]),
                ]
                for tri in [(0,1,2), (1,3,2)]:
                    f.write(struct.pack('<fff', 0, 0, 1))
                    for i in tri:
                        f.write(struct.pack('<fff', *[float(v) for v in verts[i]]))
                    f.write(struct.pack('<H', 0))

    print(f"  {stl_path.name}: {rows}x{cols} grid, {n_tri} triangles, "
          f"{stl_path.stat().st_size/1024/1024:.1f} MB, "
          f"res {terrain_size/(cols-1):.1f}m/cell")


# SDF generation

def _get_terrain_z(x, y, hmap, terrain_size, z_off):
    """Get terrain z at world position"""
    half = terrain_size / 2
    res = terrain_size / (hmap.shape[0] - 1)
    c = int((x + half) / res)
    r = int((y + half) / res)
    c = max(0, min(c, hmap.shape[1] - 1))
    r = max(0, min(r, hmap.shape[0] - 1))
    return float(hmap[r, c]) + z_off


def generate_sdf(placed, terrain_size, max_height, z_off, hmap):
    fuel_includes = ""
    for obj in placed:
        fuel_includes += f"""
    <include>
      <name>{obj['name']}</name>
      <uri>{obj['uri']}</uri>
      <static>true</static>
      <pose>{obj['x']:.2f} {obj['y']:.2f} {obj['z']+z_off:.2f} 0 0 {obj['yaw']:.3f}</pose>
    </include>
"""

    # buildings (Fuel models)
    building_models = ""
    for bld in BUILDINGS:
        gz = _get_terrain_z(bld["x"], bld["y"], hmap, terrain_size, z_off)
        building_models += f"""
    <include>
      <name>{bld['name']}</name>
      <uri>{bld['uri']}</uri>
      <static>true</static>
      <pose>{bld['x']:.2f} {bld['y']:.2f} {gz:.2f} 0 0 {bld['yaw']:.3f}</pose>
    </include>
"""

    # village extras (lamp posts, dumpsters, debris, etc.)
    for extra in VILLAGE_EXTRAS:
        gz = _get_terrain_z(extra["x"], extra["y"], hmap, terrain_size, z_off)
        building_models += f"""
    <include>
      <name>{extra['name']}</name>
      <uri>{extra['uri']}</uri>
      <static>true</static>
      <pose>{extra['x']:.2f} {extra['y']:.2f} {gz:.2f} 0 0 {extra['yaw']:.3f}</pose>
    </include>
"""

    sdf = f"""<?xml version="1.0" ?>
<!--
  Scenario world: Forest -> Open field -> Village
  Generated by generate_world.py V3
  Terrain: {terrain_size}x{terrain_size}m, max height {max_height}m
  Robot spawn: ({SPAWN_POS[0]}, {SPAWN_POS[1]}) in forest
  Destination: village at (~35, 0)
  Objects: {len(placed)} Fuel + {len(BUILDINGS)} buildings
-->
<sdf version="1.9">
  <world name="outdoor_terrain">

    <physics name="4ms" type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.65 1.0</ambient>
      <background>0.55 0.70 0.95 1.0</background>
      <shadows>true</shadows>
    </scene>

    <plugin filename="gz-sim-physics-system"       name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"  name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"        name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system"        name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"            name="gz::sim::systems::Imu"/>

    <!-- Sunlight (warm) -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 0.97 0.92 1</diffuse>
      <specular>0.3 0.28 0.25 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant>
        <linear>0.001</linear><quadratic>0.0001</quadratic></attenuation>
      <direction>-0.5 0.2 -0.9</direction>
    </light>
    <light name="fill" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.25 0.28 0.35 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.3 -0.5</direction>
    </light>
    <light name="back" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.15 0.13 0.10 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.3 0.5 -0.4</direction>
    </light>

    <!-- Ground plane (flat collision at z=0) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>

    <!-- Heightmap terrain (visual only) -->
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/grass_normal.png</normal>
                <size>8</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/dirt_normal.png</normal>
                <size>6</size>
              </texture>
              <texture>
                <diffuse>file://media/materials/textures/rock_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/rock_normal.png</normal>
                <size>5</size>
              </texture>
              <blend><min_height>1.5</min_height><fade_dist>2.5</fade_dist></blend>
              <blend><min_height>3.0</min_height><fade_dist>1.5</fade_dist></blend>
              <uri>file://heightmap.png</uri>
              <size>{terrain_size} {terrain_size} {max_height}</size>
              <pos>0 0 {z_off:.2f}</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- ===== FUEL MODELS ===== -->
{fuel_includes}
    <!-- ===== BUILDINGS ===== -->
{building_models}
  </world>
</sdf>
"""
    return sdf



def main():
    parser = argparse.ArgumentParser(description="Generate scenario world V3")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--size", type=float, default=130.0)
    parser.add_argument("--max-height", type=float, default=2.0)
    parser.add_argument("--heightmap-px", type=int, default=513)
    parser.add_argument("--texture-px", type=int, default=512)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    print("=== Generating scenario world V3 ===")
    print(f"  Seed: {args.seed}, Terrain: {args.size}m, Height: {args.max_height}m")
    print(f"  Layout: Forest({SPAWN_POS}) -> Open -> Village(35,0)")

    # 1. Heightmap
    print("\n[1/5] Heightmap...")
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    hmap = generate_heightmap(args.heightmap_px, args.size, args.max_height, rng)
    write_png_gray16(HEIGHTMAP_PATH, hmap)

    # 2. Slopes
    print("\n[2/5] Slopes...")
    slopes = slope_map(hmap, args.size)
    print(f"  Max: {slopes.max():.1f}°, mean: {slopes.mean():.1f}°")

    # 3. Textures
    print("\n[3/5] Textures...")
    generate_textures(rng, args.texture_px)

    # 4. Objects
    print("\n[4/5] Objects...")
    placed = place_objects(hmap, slopes, args.size, rng)

    # 5. SDF
    print("\n[5/5] SDF...")
    half = args.size / 2
    res = args.size / (args.heightmap_px - 1)
    sp_r = int((SPAWN_POS[1] + half) / res)
    sp_c = int((SPAWN_POS[0] + half) / res)
    sp_r = max(0, min(sp_r, args.heightmap_px - 1))
    sp_c = max(0, min(sp_c, args.heightmap_px - 1))
    spawn_z = float(hmap[sp_r, sp_c])
    z_off = -spawn_z  # spawn area at z≈0

    # generate collision mesh STL
    print("  Generating collision mesh...")
    generate_collision_stl(hmap, args.size, z_off, step=2)

    sdf = generate_sdf(placed, args.size, args.max_height, z_off, hmap)
    SDF_PATH.write_text(sdf)

    # summary
    total = len(placed)
    print(f"\n=== Summary ===")
    print(f"  Heightmap: {args.heightmap_px}x{args.heightmap_px} ({args.size}x{args.size}m)")
    print(f"  Objects: {total} Fuel + {len(BUILDINGS)} buildings")
    for name in dict.fromkeys(m["name"] for m in ZONE_MODELS):
        n = sum(1 for p in placed if p["type"] == name)
        if n > 0:
            print(f"    {name}: {n}")
    print(f"  Buildings: {len(BUILDINGS)} (Fuel models)")
    print(f"  Village extras: {len(VILLAGE_EXTRAS)} (lamp posts, debris, etc.)")
    print(f"  Spawn: ({SPAWN_POS[0]}, {SPAWN_POS[1]}), terrain z={spawn_z:.2f}, z_offset={z_off:.2f}")
    print(f"  Robot spawn z (launch file): 0.3")
    print(f"  Trail: {len(TRAIL_WAYPOINTS)} waypoints, {TRAIL_WIDTH}m wide")
    print("  Done!")


if __name__ == "__main__":
    main()
