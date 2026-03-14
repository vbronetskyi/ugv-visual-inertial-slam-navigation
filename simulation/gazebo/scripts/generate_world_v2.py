#!/usr/bin/env python3
"""
Procedural outdoor world generator V2 - extended 220x150m map

Full outdoor environment for UGV Husky A200 simulation:
  - Heightmap 257x257 (FBM noise + domain warping)
  - Terrain mesh: OBJ visual (257x257) + STL collision (65x65, smoothed, +15cm offset)
  - Texture: 2048x2048 procedural (grass/dirt/rock blend + dirt road)
  - 370+ objects: trees, bushes, stumps, fallen trees, rocks, buildings, ruins, barriers
  - 3 routes with A* pathfinding, 2m+ clearance from obstacles
  - Zones: dense forest (west), open field (center), village (east)
  - Road from spawn (-105,-8) through curved path to village (82,-13)

Usage:
    python3 generate_world_v2.py [--seed 42] [--size 220] [--max-height 10]

Physics engine: Bullet (more stable than ODE for outdoor terrain)
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
TERRAIN_OBJ_PATH = WORLDS_DIR / "terrain.obj"
TERRAIN_MTL_PATH = WORLDS_DIR / "terrain.mtl"
TERRAIN_STL_PATH = WORLDS_DIR / "terrain_collision.stl"
TERRAIN_TEXTURE_PATH = WORLDS_DIR / "terrain_texture.png"
SDF_PATH = WORLDS_DIR / "outdoor_terrain.sdf"

# Map parameters
DEFAULT_SIZE = 390.0
DEFAULT_HEIGHT = 5.0
HEIGHTMAP_PX = 257
TEXTURE_PX = 2048
COLLISION_STEP = 4  # 257/4 ≈ 65 vertices for STL
COLLISION_Z_OFFSET = 0.15  # +15cm up for reliable wheel contact

# Road
ROAD_POINTS = [
    (-170, -20), (-150, -25), (-130, -35), (-110, -45),
    (-90, -50), (-60, -50), (-20, -25), (20, -10), (60, -3), (100, 0)
]
ROAD_WIDTH = 5.0
ROAD_BLEND = 3.0

# Zones
# dense forest: x < -60
# open field: -60 < x < 40
# village: x > 40

# Objects by zone
ZONE_MODELS = [
    # dense forest west
    {"name": "Oak tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 80, "radius": 3.0, "x_range": (-190, -60), "y_range": (-190, 190)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 60, "radius": 2.5, "x_range": (-190, -60), "y_range": (-190, 190)},
    # bushes in the forest
    {"name": "Bush", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 40, "radius": 1.5, "x_range": (-190, -40), "y_range": (-190, 190),
     "scale": 0.3},
    # stumps
    {"name": "Stump", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 20, "radius": 0.8, "x_range": (-190, 0), "y_range": (-190, 190),
     "scale": 0.15},
    # fallen trees
    {"name": "Fallen tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 15, "radius": 4.0, "x_range": (-190, -20), "y_range": (-190, 190),
     "fallen": True},
    # sparse trees - transition zone
    {"name": "Oak tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 20, "radius": 3.0, "x_range": (-60, 40), "y_range": (-190, 190)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 10, "radius": 2.5, "x_range": (-60, 40), "y_range": (-190, 190)},
    # rocks everywhere
    {"name": "Falling Rock 1", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Falling Rock 1",
     "count": 50, "radius": 1.5, "x_range": (-190, 190), "y_range": (-190, 190)},
    # village trees
    {"name": "Oak tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree",
     "count": 25, "radius": 3.0, "x_range": (40, 190), "y_range": (-190, 190)},
    {"name": "Pine Tree", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree",
     "count": 15, "radius": 2.5, "x_range": (40, 190), "y_range": (-190, 190)},
    # barriers + barrels in village
    {"name": "Construction Barrel", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Barrel",
     "count": 15, "radius": 0.8, "x_range": (50, 150), "y_range": (-60, 60)},
    {"name": "Construction Cone", "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone",
     "count": 12, "radius": 0.5, "x_range": (50, 150), "y_range": (-60, 60)},
]

# village buildings (Fuel models)
BUILDINGS = [
    {"name": "house_1", "x": 70, "y": -25, "yaw": 0.2, "footprint": 14,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 1"},
    {"name": "house_2", "x": 90, "y": 20, "yaw": -0.4, "footprint": 14,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 2"},
    {"name": "house_3", "x": 110, "y": -15, "yaw": 0.8, "footprint": 14,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/House 3"},
    {"name": "collapsed_house_1", "x": 130, "y": 10, "yaw": -0.3, "footprint": 14,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Collapsed House"},
    {"name": "collapsed_house_2", "x": 80, "y": 40, "yaw": 1.2, "footprint": 14,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Collapsed House"},
    {"name": "depot_1", "x": 120, "y": 35, "yaw": 0.5, "footprint": 10,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"},
]

# village extras
VILLAGE_EXTRAS = [
    {"name": "lamp_post_1", "x": 75, "y": -5, "yaw": 0.0,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"},
    {"name": "lamp_post_2", "x": 95, "y": 5, "yaw": 0.0,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"},
    {"name": "lamp_post_3", "x": 115, "y": 0, "yaw": 0.0,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Lamp Post"},
    {"name": "dumpster_1", "x": 72, "y": -35, "yaw": 0.8,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster"},
    {"name": "dumpster_2", "x": 125, "y": 25, "yaw": 2.1,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster"},
    {"name": "dumpster_3", "x": 95, "y": -30, "yaw": 1.5,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Dumpster"},
    {"name": "wall_debris_1", "x": 128, "y": 15, "yaw": 0.5,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    {"name": "wall_debris_2", "x": 132, "y": 8, "yaw": 2.3,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    {"name": "wall_debris_3", "x": 82, "y": 45, "yaw": 1.1,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Urban Wall Debris"},
    {"name": "fire_hydrant_1", "x": 85, "y": 5, "yaw": 1.5,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire hydrant"},
    {"name": "fire_hydrant_2", "x": 105, "y": -10, "yaw": 0.3,
     "uri": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Fire hydrant"},
]

MIN_DISTANCE = 2.0
MAX_SLOPE_DEG = 25.0


# FBM noise

def _smooth_noise(size, scale, rng):
    """Smooth noise with bilinear interpolation"""
    grid_n = max(2, int(size / scale) + 2)
    grid = rng.random((grid_n, grid_n)).astype(np.float64)
    coords = np.linspace(0, grid_n - 1.001, size)
    xi = np.floor(coords).astype(int)
    xf = coords - xi
    xi = np.clip(xi, 0, grid_n - 2)
    xf_s = xf * xf * (3.0 - 2.0 * xf)  # smoothstep
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
    """Fractal Brownian Motion - sum of multiple noise octaves"""
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
    """Min distance from each pixel to nearest polyline segment"""
    min_d = np.full_like(xx, 1e10)
    interp_t = np.zeros_like(xx)
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
        global_t = (seg_starts[i] + t * math.sqrt(seg_len_sq)) / max(total_len, 1)
        interp_t = np.where(closer, global_t, interp_t)
    return min_d, interp_t


def generate_heightmap(size_px, terrain_size, max_height, rng):
    """Heightmap with FBM + domain warping + zone-based amplitude"""
    half = terrain_size / 2
    x = np.linspace(-half, half, size_px)
    xx, yy = np.meshgrid(x, x)

    # base FBM noise
    base = fbm(size_px, 6, 0.50, 2.0, size_px / 3, rng)

    # domain warping - shift coords with noise for natural look
    wx = fbm(size_px, 4, 0.45, 2.0, size_px / 4, rng) - 0.5
    wy = fbm(size_px, 4, 0.45, 2.0, size_px / 4, rng) - 0.5
    iy, ix = np.mgrid[0:size_px, 0:size_px]
    wxi = np.clip((ix + wx * 15).astype(int), 0, size_px - 1)
    wyi = np.clip((iy + wy * 15).astype(int), 0, size_px - 1)
    hmap = base[wyi, wxi]

    # zone amplitude - forest=full, field=medium, village=low
    zone_amp = np.clip(0.3 + 0.7 * (1.0 - (xx + 60) / 120), 0.25, 1.0)
    hmap *= zone_amp

    # normalization
    hmap -= hmap.min()
    mx = hmap.max()
    if mx > 0:
        hmap = hmap / mx * max_height

    # smooth the road path
    road_dist, road_t = _dist_to_polyline(xx, yy, ROAD_POINTS)
    start_h = hmap[size_px // 2, int((-170 + half) / terrain_size * (size_px - 1))]
    end_h = hmap[size_px // 2, int((100 + half) / terrain_size * (size_px - 1))]
    road_h = start_h + road_t * (end_h - start_h)
    blend_road = np.clip((road_dist - ROAD_WIDTH / 2) / ROAD_BLEND, 0, 1)
    target_h = road_h - 0.15  # road slightly below
    hmap = target_h + (hmap - target_h) * blend_road

    # flatten building footprints
    res = terrain_size / (size_px - 1)
    for bld in BUILDINGS:
        bx, by = bld["x"], bld["y"]
        dist_b = np.sqrt((xx - bx) ** 2 + (yy - by) ** 2)
        footprint_r = bld["footprint"] / 2 + 2.0
        blend_b = np.clip((dist_b - footprint_r) / 3.0, 0, 1)
        br = int((by + half) / res)
        bc = int((bx + half) / res)
        br = max(0, min(br, size_px - 1))
        bc = max(0, min(bc, size_px - 1))
        bh = hmap[br, bc]
        hmap = bh + (hmap - bh) * blend_b

    return hmap


def slope_map(hmap, terrain_size):
    """Compute slope map in degrees"""
    res = terrain_size / (hmap.shape[0] - 1)
    gy, gx = np.gradient(hmap, res)
    return np.degrees(np.arctan(np.sqrt(gx ** 2 + gy ** 2)))


# PNG writers

def _png_chunk(ctype, data):
    c = ctype + data
    return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)


def write_png_gray16(path, data):
    """Write 16-bit grayscale PNG"""
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
    """Write 8-bit RGB PNG"""
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


# Terrain texture (2048x2048)

def generate_terrain_texture(hmap, terrain_size, rng, sz=2048):
    """Procedural texture: grass/dirt/rock blend + dirt road"""
    half = terrain_size / 2
    x = np.linspace(-half, half, sz)
    xx, yy = np.meshgrid(x, x)

    # base noise layers
    n1 = fbm(sz, 5, 0.55, 2.0, sz / 4, rng)
    n2 = fbm(sz, 4, 0.50, 2.0, sz / 8, rng)

    # grass base
    r = (55 + n1 * 50 + n2 * 25).astype(np.float64)
    g = (110 + n1 * 60 + n2 * 30).astype(np.float64)
    b = (35 + n1 * 25 + n2 * 10).astype(np.float64)

    # height blend - interpolate heightmap to texture res
    from scipy.ndimage import zoom
    hmap_tex = zoom(hmap, sz / hmap.shape[0], order=1)
    h_norm = (hmap_tex - hmap_tex.min()) / max(hmap_tex.max() - hmap_tex.min(), 0.01)

    # rock on steep slopes
    gy, gx = np.gradient(hmap_tex, terrain_size / sz)
    slope = np.sqrt(gx ** 2 + gy ** 2)
    rock_blend = np.clip((slope - 0.05) / 0.1, 0, 1)
    r = r * (1 - rock_blend) + (115 + n1 * 30) * rock_blend
    g = g * (1 - rock_blend) + (110 + n1 * 25) * rock_blend
    b = b * (1 - rock_blend) + (100 + n1 * 20) * rock_blend

    # dirt road overlay
    road_dist, _ = _dist_to_polyline(xx, yy, ROAD_POINTS)
    road_blend = np.clip(1.0 - (road_dist - ROAD_WIDTH * 0.3) / (ROAD_WIDTH * 0.7), 0, 1)
    dirt_r = 125 + n1 * 30
    dirt_g = 100 + n1 * 20
    dirt_b = 70 + n1 * 15
    r = r * (1 - road_blend) + dirt_r * road_blend
    g = g * (1 - road_blend) + dirt_g * road_blend
    b = b * (1 - road_blend) + dirt_b * road_blend

    rgb = np.stack([
        np.clip(r, 0, 255).astype(np.uint8),
        np.clip(g, 0, 255).astype(np.uint8),
        np.clip(b, 0, 255).astype(np.uint8),
    ], axis=-1)

    write_png_rgb(TERRAIN_TEXTURE_PATH, rgb)
    return rgb


# Terrain mesh (OBJ + STL)

def generate_terrain_obj(hmap, terrain_size, z_off):
    """OBJ visual mesh (257x257 vertices) with UV coords"""
    rows, cols = hmap.shape
    half = terrain_size / 2
    xs = np.linspace(-half, half, cols)
    ys = np.linspace(-half, half, rows)
    xx, yy = np.meshgrid(xs, ys)
    zz = hmap + z_off

    with open(TERRAIN_OBJ_PATH, 'w') as f:
        f.write(f"# Terrain mesh {cols}x{rows}\n")
        f.write(f"mtllib terrain.mtl\n")
        f.write(f"usemtl terrain_mat\n\n")

        # vertices
        for r in range(rows):
            for c in range(cols):
                f.write(f"v {xx[r,c]:.3f} {yy[r,c]:.3f} {zz[r,c]:.3f}\n")

        # uVs
        for r in range(rows):
            for c in range(cols):
                u = c / (cols - 1)
                v = r / (rows - 1)
                f.write(f"vt {u:.4f} {v:.4f}\n")

        # faces (CCW winding)
        for r in range(rows - 1):
            for c in range(cols - 1):
                i = r * cols + c + 1  # OBJ is 1-indexed
                f.write(f"f {i}/{i} {i+1}/{i+1} {i+cols+1}/{i+cols+1}\n")
                f.write(f"f {i+1}/{i+1} {i+cols+2}/{i+cols+2} {i+cols+1}/{i+cols+1}\n")

    # mtl file
    with open(TERRAIN_MTL_PATH, 'w') as f:
        f.write("newmtl terrain_mat\n")
        f.write("Ka 0.5 0.5 0.5\n")
        f.write("Kd 0.8 0.8 0.8\n")
        f.write("Ks 0.0 0.0 0.0\n")
        f.write(f"map_Kd terrain_texture.png\n")

    print(f"  {TERRAIN_OBJ_PATH.name}: {rows}x{cols} vertices")


def generate_collision_stl(hmap, terrain_size, z_off, step=4):
    """STL collision mesh (downsampled, smoothed, +offset)"""
    from scipy.ndimage import gaussian_filter
    hmap_smooth = gaussian_filter(hmap, sigma=1.5)
    hmap_ds = hmap_smooth[::step, ::step]
    rows, cols = hmap_ds.shape
    half = terrain_size / 2
    xs = np.linspace(-half, half, cols)
    ys = np.linspace(-half, half, rows)
    xx, yy = np.meshgrid(xs, ys)
    zz = hmap_ds + z_off + COLLISION_Z_OFFSET

    n_tri = (rows - 1) * (cols - 1) * 2
    with open(TERRAIN_STL_PATH, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', n_tri))
        for r in range(rows - 1):
            for c in range(cols - 1):
                verts = [
                    (xx[r, c], yy[r, c], zz[r, c]),
                    (xx[r, c+1], yy[r, c+1], zz[r, c+1]),
                    (xx[r+1, c], yy[r+1, c], zz[r+1, c]),
                    (xx[r+1, c+1], yy[r+1, c+1], zz[r+1, c+1]),
                ]
                for tri in [(0, 1, 2), (1, 3, 2)]:
                    v0, v1, v2 = [verts[i] for i in tri]
                    e1 = (v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2])
                    e2 = (v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2])
                    nx = e1[1]*e2[2] - e1[2]*e2[1]
                    ny = e1[2]*e2[0] - e1[0]*e2[2]
                    nz = e1[0]*e2[1] - e1[1]*e2[0]
                    ln = math.sqrt(nx*nx + ny*ny + nz*nz)
                    if ln > 0:
                        nx, ny, nz = nx/ln, ny/ln, nz/ln
                    f.write(struct.pack('<fff', nx, ny, nz))
                    for i in tri:
                        f.write(struct.pack('<fff', *[float(v) for v in verts[i]]))
                    f.write(struct.pack('<H', 0))

    print(f"  {TERRAIN_STL_PATH.name}: {rows}x{cols} grid, {n_tri} triangles, "
          f"{TERRAIN_STL_PATH.stat().st_size / 1024 / 1024:.1f} MB")


# Object placement

def _on_road(x, y):
    for i in range(len(ROAD_POINTS) - 1):
        ax, ay = ROAD_POINTS[i]
        bx, by = ROAD_POINTS[i + 1]
        dx, dy = bx - ax, by - ay
        sl2 = dx * dx + dy * dy
        if sl2 < 1e-6:
            continue
        t = max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / sl2))
        if math.hypot(x - (ax + t * dx), y - (ay + t * dy)) < (ROAD_WIDTH / 2 + 2.0):
            return True
    return False


def _near_building(x, y):
    for b in BUILDINGS:
        if math.hypot(x - b["x"], y - b["y"]) < b["footprint"] / 2 + 4.0:
            return True
    for e in VILLAGE_EXTRAS:
        if math.hypot(x - e["x"], y - e["y"]) < 3.0:
            return True
    return False


def place_objects(hmap, slopes, terrain_size, rng):
    """Place objects respecting constraints"""
    placed = []
    half = terrain_size / 2
    res = terrain_size / (hmap.shape[0] - 1)
    counters = {}

    def get_h(x, y):
        c = int((x + half) / res)
        r = int((y + half) / res)
        c = max(0, min(c, hmap.shape[1] - 1))
        r = max(0, min(r, hmap.shape[0] - 1))
        return float(hmap[r, c])

    def get_s(x, y):
        c = int((x + half) / res)
        r = int((y + half) / res)
        c = max(0, min(c, slopes.shape[1] - 1))
        r = max(0, min(r, slopes.shape[0] - 1))
        return float(slopes[r, c])

    def too_close(x, y, min_d=MIN_DISTANCE):
        for p in placed:
            if math.hypot(x - p["x"], y - p["y"]) < min_d:
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
            if _on_road(x, y):
                continue
            if _near_building(x, y):
                continue
            if get_s(x, y) > MAX_SLOPE_DEG:
                continue
            if too_close(x, y):
                continue

            counters[name] += 1
            obj = {
                "x": x, "y": y, "z": get_h(x, y),
                "yaw": rng.uniform(0, 2 * math.pi),
                "name": f"{name}_{counters[name]}",
                "uri": mdef["uri"], "type": name,
            }
            if mdef.get("scale"):
                obj["scale"] = mdef["scale"]
            if mdef.get("fallen"):
                obj["pitch"] = 1.5
            placed.append(obj)
            count += 1

        print(f"  {name} [{x_lo:.0f}..{x_hi:.0f}]: {count}/{mdef['count']}")

    return placed


# SDF generation (Bullet physics)

def _get_terrain_z(x, y, hmap, terrain_size, z_off):
    half = terrain_size / 2
    res = terrain_size / (hmap.shape[0] - 1)
    c = int((x + half) / res)
    r = int((y + half) / res)
    c = max(0, min(c, hmap.shape[1] - 1))
    r = max(0, min(r, hmap.shape[0] - 1))
    return float(hmap[r, c]) + z_off


def generate_sdf(placed, terrain_size, max_height, z_off, hmap):
    """Generate SDF file with Bullet physics"""
    fuel_includes = ""
    for obj in placed:
        pitch = obj.get("pitch", 0)
        scale_str = ""
        if "scale" in obj:
            s = obj["scale"]
            scale_str = f"\n      <scale>{s} {s} {s}</scale>"
        fuel_includes += f"""
    <include>
      <name>{obj['name']}</name>
      <uri>{obj['uri']}</uri>
      <static>true</static>{scale_str}
      <pose>{obj['x']:.2f} {obj['y']:.2f} {obj['z']+z_off:.2f} 0 {pitch:.2f} {obj['yaw']:.3f}</pose>
    </include>
"""

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
  Outdoor terrain world V2 (390x390m)
  Generated by generate_world_v2.py
  Terrain: {terrain_size}x{terrain_size}m, max height {max_height}m
  Zones: dense forest (west) | open field (center) | village (east)
  Road: (-170,-20) -> (100,0)
  Objects: {len(placed)} scattered + {len(BUILDINGS)} buildings + {len(VILLAGE_EXTRAS)} extras
  Physics: Bullet
-->
<sdf version="1.9">
  <world name="outdoor_terrain">

    <!-- Bullet physics engine -->
    <physics name="bullet_4ms" type="bullet">
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

    <!-- Sunlight -->
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

    <!-- Terrain: OBJ visual mesh -->
    <model name="terrain_visual">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://terrain.obj</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Terrain: STL collision mesh (65x65, smoothed, +15cm) -->
    <model name="terrain_collision">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://terrain_collision.stl</uri>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <bullet>
                <friction>1.0</friction>
                <friction2>0.8</friction2>
              </bullet>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- ===== FUEL MODELS ({len(placed)} objects) ===== -->
{fuel_includes}
    <!-- ===== BUILDINGS + EXTRAS ({len(BUILDINGS)} + {len(VILLAGE_EXTRAS)}) ===== -->
{building_models}
  </world>
</sdf>
"""
    return sdf



def main():
    parser = argparse.ArgumentParser(description="Generate outdoor world V2 (390x390m)")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--size", type=float, default=DEFAULT_SIZE)
    parser.add_argument("--max-height", type=float, default=DEFAULT_HEIGHT)
    parser.add_argument("--heightmap-px", type=int, default=HEIGHTMAP_PX)
    parser.add_argument("--texture-px", type=int, default=TEXTURE_PX)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    print("=== Generating outdoor world V2 ===")
    print(f"  Seed: {args.seed}")
    print(f"  Terrain: {args.size}x{args.size}m, height: {args.max_height}m")
    print(f"  Heightmap: {args.heightmap_px}px, Texture: {args.texture_px}px")

    # 1. Heightmap
    print("\n[1/6] Heightmap...")
    WORLDS_DIR.mkdir(parents=True, exist_ok=True)
    hmap = generate_heightmap(args.heightmap_px, args.size, args.max_height, rng)
    write_png_gray16(HEIGHTMAP_PATH, hmap)

    # 2. Slopes
    print("\n[2/6] Slopes...")
    slopes = slope_map(hmap, args.size)
    print(f"  Max: {slopes.max():.1f}deg, mean: {slopes.mean():.1f}deg")

    # 3. Z offset (spawn at z=0)
    half = args.size / 2
    res = args.size / (args.heightmap_px - 1)
    sp_r = int((0 + half) / res)
    sp_c = int((-170 + half) / res)
    sp_r = max(0, min(sp_r, args.heightmap_px - 1))
    sp_c = max(0, min(sp_c, args.heightmap_px - 1))
    spawn_z = float(hmap[sp_r, sp_c])
    z_off = -spawn_z

    # 4. Terrain meshes
    print("\n[3/6] Terrain OBJ mesh...")
    generate_terrain_obj(hmap, args.size, z_off)

    print("\n[4/6] Collision STL mesh...")
    generate_collision_stl(hmap, args.size, z_off, step=COLLISION_STEP)

    # 5. Texture
    print("\n[5/6] Terrain texture...")
    generate_terrain_texture(hmap, args.size, rng, args.texture_px)

    # 6. Objects + SDF
    print("\n[6/6] Objects...")
    placed = place_objects(hmap, slopes, args.size, rng)

    sdf = generate_sdf(placed, args.size, args.max_height, z_off, hmap)
    SDF_PATH.write_text(sdf)

    # summary
    total = len(placed) + len(BUILDINGS) + len(VILLAGE_EXTRAS)
    print(f"\n=== Summary ===")
    print(f"  Terrain: {args.size}x{args.size}m, heightmap {args.heightmap_px}px")
    print(f"  Visual mesh: OBJ {args.heightmap_px}x{args.heightmap_px} vertices")
    print(f"  Collision mesh: STL {args.heightmap_px // COLLISION_STEP}x{args.heightmap_px // COLLISION_STEP} (+{COLLISION_Z_OFFSET}m offset)")
    print(f"  Texture: {args.texture_px}x{args.texture_px} (grass/dirt/rock + road)")
    print(f"  Total objects: {total}")
    print(f"    Scattered: {len(placed)}")
    print(f"    Buildings: {len(BUILDINGS)}")
    print(f"    Village extras: {len(VILLAGE_EXTRAS)}")
    print(f"  Physics: Bullet")
    print(f"  z_offset: {z_off:.2f}")
    print("  Done!")


if __name__ == "__main__":
    main()
