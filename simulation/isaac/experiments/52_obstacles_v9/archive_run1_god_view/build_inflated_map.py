#!/usr/bin/env python3
"""Bake exp 52 obstacles into the static Nav2 occupancy map.

Reads the v9 blank_south_map.pgm + obstacle_positions.yaml, stamps each
cone cluster as occupied with an inflation disk, writes
map_with_obstacles.pgm + .yaml.

Map convention (from Nav2 map_server):
  pgm is greyscale, values are interpreted as free/occupied via yaml
  occupied_thresh / free_thresh. pgm coord (col, row) is top-left origin,
  row grows downward. World (x, y) aligns with map_yaml origin + resolution:
    col = (x - origin_x) / res
    row = H - 1 - (y - origin_y) / res
"""
import math
import os
import sys
import yaml
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "..", "config")

with open(os.path.join(CFG, "blank_south_map.yaml")) as f:
    map_yaml = yaml.safe_load(f)

res = float(map_yaml["resolution"])
origin = map_yaml["origin"]  # [ox, oy, oyaw]
ox, oy = float(origin[0]), float(origin[1])
pgm_src = map_yaml["image"]
# Map yaml points at a relative or abs path; resolve both
if not os.path.isabs(pgm_src):
    pgm_src = os.path.join(CFG, pgm_src)
# But our map was already copied from v9's blank_south_map - use local copy
pgm_local = os.path.join(CFG, "blank_south_map.pgm")
if os.path.exists(pgm_local):
    pgm_src = pgm_local

print(f"map yaml res={res} origin=({ox},{oy}) image={pgm_src}")

# Load PGM (P5 binary or P2 ASCII, grayscale)
def load_pgm(path):
    with open(path, "rb") as f:
        head = f.readline().strip()
        if head not in (b"P5", b"P2"):
            raise RuntimeError(f"unsupported PGM magic {head}")
        # Skip comments
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = [int(v) for v in line.split()]
        maxv = int(f.readline().strip())
        if head == b"P5":
            data = np.frombuffer(f.read(), dtype=np.uint8 if maxv < 256 else np.uint16).reshape(h, w)
        else:
            data = np.array([int(v) for v in f.read().split()], dtype=np.uint8).reshape(h, w)
    return data, maxv

img, maxv = load_pgm(pgm_src)
img = np.array(img, dtype=np.uint8, copy=True)  # ensure writable
H, W = img.shape
print(f"pgm {W}×{H} maxv={maxv}")

# Load obstacle config
with open(os.path.join(CFG, "obstacle_positions.yaml")) as f:
    obs = yaml.safe_load(f)

inflation_r_m = obs["inflation"]["total_radius_m"]
inflation_r_px = int(math.ceil(inflation_r_m / res))
print(f"inflation radius {inflation_r_m}m = {inflation_r_px}px")

def world_to_pix(x, y):
    col = int(round((x - ox) / res))
    row = int(round(H - 1 - (y - oy) / res))
    return col, row

OCC = 0  # pgm: 0 = black = occupied

def stamp_disk(wx, wy, radius_m):
    """Stamp an occupied disk of given radius at world (wx, wy)."""
    r_px = int(math.ceil(radius_m / res))
    c, r = world_to_pix(wx, wy)
    count = 0
    for dr in range(-r_px, r_px + 1):
        for dc in range(-r_px, r_px + 1):
            if dr*dr + dc*dc <= r_px * r_px:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W:
                    img[rr, cc] = OCC
                    count += 1
    return count

stamped = 0
# Cone walls
for ob in obs["obstacles"]:
    print(f"  stamping obstacle {ob['id']} kind={ob['kind']}")
    for (wx, wy) in ob["cone_cluster"]:
        stamped += stamp_disk(wx, wy, inflation_r_m)

# Stamp scene objects so Nav2 knows where REAL forest is (trees, houses, shrubs).
# Without this, drift -> robot off route -> Nav2 plans direct line through forest
# -> robot hits unmapped tree -> endless spin loop (exp 52 run 1 failure mode).
import json, os
scene_json = "/tmp/gazebo_models.json"
if os.path.exists(scene_json):
    with open(scene_json) as f:
        scene = json.load(f)
    # Trees: physical trunk r=0.7, canopy visual 1.3, use 1.0 m + robot margin
    TREE_R = 1.0 + inflation_r_m       # effective 1.8 m
    SHRUB_R = 0.4 + inflation_r_m      # 1.2 m
    ROCK_R = 0.8 + inflation_r_m       # 1.6 m
    HOUSE_R = 3.0 + inflation_r_m      # 3.8 m (house box 6m)
    tree_n = shrub_n = rock_n = house_n = 0
    for m in scene:
        t = m["type"]
        if t in ("pine", "oak"):
            stamped += stamp_disk(m["x"], m["y"], TREE_R)
            tree_n += 1
        elif t == "shrub":
            stamped += stamp_disk(m["x"], m["y"], SHRUB_R)
            shrub_n += 1
        elif t == "rock":
            stamped += stamp_disk(m["x"], m["y"], ROCK_R)
            rock_n += 1
        elif t == "house":
            stamped += stamp_disk(m["x"], m["y"], HOUSE_R)
            house_n += 1
    print(f"  stamped {tree_n} trees, {shrub_n} shrubs, {rock_n} rocks, {house_n} houses")
else:
    print(f"  [warn] {scene_json} not found - skipping scene objects")

print(f"stamped {stamped} cells total (cones + forest)")

# ALSO build a return-leg map: forest without the cone walls (supervisor
# swaps to this on turnaround so planner isn't blind to forest either).
import copy
img_return = np.array(img, copy=True)
# Clear cone cells by restamping them as free
for ob in obs["obstacles"]:
    for (wx, wy) in ob["cone_cluster"]:
        c, r = world_to_pix(wx, wy)
        r_px = int(math.ceil(inflation_r_m / res))
        for dr in range(-r_px, r_px + 1):
            for dc in range(-r_px, r_px + 1):
                if dr*dr + dc*dc <= r_px * r_px:
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W:
                        img_return[rr, cc] = 254  # free
# Re-stamp nearby scene objects so we don't accidentally carve holes in a tree
# that happens to be inside the cleared cone inflation disk.
# Simplest: re-run scene stamping on the return image as well.
def stamp_disk_on(dst, wx, wy, radius_m):
    r_px = int(math.ceil(radius_m / res))
    c, r = world_to_pix(wx, wy)
    for dr in range(-r_px, r_px + 1):
        for dc in range(-r_px, r_px + 1):
            if dr*dr + dc*dc <= r_px * r_px:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W:
                    dst[rr, cc] = OCC
if os.path.exists(scene_json):
    for m in scene:
        t = m["type"]
        if t in ("pine", "oak"):
            stamp_disk_on(img_return, m["x"], m["y"], TREE_R)
        elif t == "shrub":
            stamp_disk_on(img_return, m["x"], m["y"], SHRUB_R)
        elif t == "rock":
            stamp_disk_on(img_return, m["x"], m["y"], ROCK_R)
        elif t == "house":
            stamp_disk_on(img_return, m["x"], m["y"], HOUSE_R)

# Save return-leg map
ret_pgm = os.path.join(CFG, "map_return_leg.pgm")
ret_yaml = os.path.join(CFG, "map_return_leg.yaml")
with open(ret_pgm, "wb") as f:
    f.write(b"P5\n")
    f.write(b"# exp 52 return leg - forest only, no cone walls\n")
    f.write(f"{W} {H}\n".encode())
    f.write(f"{maxv}\n".encode())
    f.write(img_return.tobytes())
with open(ret_yaml, "w") as f:
    yaml.safe_dump({
        "image": ret_pgm,
        "resolution": res,
        "origin": origin,
        "occupied_thresh": map_yaml.get("occupied_thresh", 0.65),
        "free_thresh": map_yaml.get("free_thresh", 0.25),
        "negate": map_yaml.get("negate", 0),
    }, f, default_flow_style=False)
print(f"wrote {ret_pgm} (return leg map)")
print(f"wrote {ret_yaml}")

# Save new pgm
out_pgm = os.path.join(CFG, "map_with_obstacles.pgm")
out_yaml = os.path.join(CFG, "map_with_obstacles.yaml")
with open(out_pgm, "wb") as f:
    f.write(b"P5\n")
    f.write(f"# exp 52 blank + obstacles\n".encode())
    f.write(f"{W} {H}\n".encode())
    f.write(f"{maxv}\n".encode())
    f.write(img.tobytes())

# Write yaml pointing at new pgm
out_yaml_content = {
    "image": out_pgm,
    "resolution": res,
    "origin": origin,
    "occupied_thresh": map_yaml.get("occupied_thresh", 0.65),
    "free_thresh": map_yaml.get("free_thresh", 0.25),
    "negate": map_yaml.get("negate", 0),
}
with open(out_yaml, "w") as f:
    yaml.safe_dump(out_yaml_content, f, default_flow_style=False)

print(f"wrote {out_pgm}")
print(f"wrote {out_yaml}")
