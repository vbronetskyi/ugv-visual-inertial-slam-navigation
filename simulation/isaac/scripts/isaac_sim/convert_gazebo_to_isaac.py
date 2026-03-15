#!/usr/bin/env python3
import json
import numpy as np
import math
import os

from isaacsim import SimulationApp
app = SimulationApp({'headless': True})

import omni
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdPhysics, PhysxSchema, Gf, Sdf, PhysicsSchemaTools
from PIL import Image

ASSETS = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
DSREADY = f"{ASSETS}/NVIDIA/dsready_content/nv_content/common_assets/props_vegetation"
OUT_DIR = "/workspace/simulation/isaac/assets/renders/maps"
SCENE_OUT = "/opt/husky_forest_scene.usd"

# dsready photorealistic assets - multiple variants per type
PINE_ASSETS = [
    f"{DSREADY}/veg_tree_fir_douglas_01/veg_tree_fir_douglas_01.usd",
    f"{DSREADY}/veg_tree_fir_douglas_02/veg_tree_fir_douglas_02.usd",
    f"{DSREADY}/veg_tree_pine_yellow_01/veg_tree_pine_yellow_01.usd",
    f"{DSREADY}/veg_tree_pine_yellow_02/veg_tree_pine_yellow_02.usd",
    f"{DSREADY}/veg_tree_pine_yellow_03/veg_tree_pine_yellow_03.usd",
    f"{DSREADY}/veg_tree_hemlock_01/veg_tree_hemlock_01.usd",
    f"{DSREADY}/veg_tree_spruce_01/veg_tree_spruce_01.usd",
]

OAK_ASSETS = [
    f"{DSREADY}/veg_tree_oak_shumard_01/veg_tree_oak_shumard_01.usd",
    f"{DSREADY}/veg_tree_oak_shumard_02/veg_tree_oak_shumard_02.usd",
    f"{DSREADY}/veg_tree_beech_american_01/veg_tree_beech_american_01.usd",
    f"{DSREADY}/veg_tree_beech_american_02/veg_tree_beech_american_02.usd",
    f"{DSREADY}/veg_tree_beech_american_03/veg_tree_beech_american_03.usd",
    f"{DSREADY}/veg_tree_maple_red_01/veg_tree_maple_red_01.usd",
    f"{DSREADY}/veg_tree_birch_gray_01/veg_tree_birch_gray_01.usd",
    f"{DSREADY}/veg_tree_ash_red_01/veg_tree_ash_red_01.usd",
]

FALLEN_ASSETS = [
    f"{DSREADY}/sct_debris_branch_fallen_01/sct_debris_branch_fallen_01.usd",
    f"{DSREADY}/sct_debris_branch_fallen_02/sct_debris_branch_fallen_02.usd",
    f"{DSREADY}/sct_debris_branch_fallen_03/sct_debris_branch_fallen_03.usd",
    f"{DSREADY}/sct_debris_branch_fallen_04/sct_debris_branch_fallen_04.usd",
]

ROCK_ASSETS = [
    f"{ASSETS}/NVIDIA/Assets/Vegetation/Rocks/rock_small_{i:02d}.usda"
    for i in range(1, 16)
]

SHRUB_ASSETS = [
    f"{DSREADY}/veg_shrub_gen_05/veg_shrub_gen_05.usd",
    f"{DSREADY}/veg_shrub_gen_06/veg_shrub_gen_06.usd",
    f"{DSREADY}/veg_shrub_gen_07/veg_shrub_gen_07.usd",
    f"{DSREADY}/veg_shrub_gen_08/veg_shrub_gen_08.usd",
    f"{DSREADY}/veg_shrub_sm_01/veg_shrub_sm_01.usd",
    f"{DSREADY}/veg_shrub_sm_02/veg_shrub_sm_02.usd",
]

FERN_ASSETS = [
    f"{DSREADY}/veg_plant_fern_01/veg_plant_fern_01.usd",
    f"{DSREADY}/veg_plant_fern_02/veg_plant_fern_02.usd",
    f"{DSREADY}/veg_plant_fern_03/veg_plant_fern_03.usd",
]

GRASS_ASSETS = [
    f"{DSREADY}/veg_grass_clump_01/veg_grass_clump_01.usd",
    f"{DSREADY}/veg_grass_clump_02/veg_grass_clump_02.usd",
    f"{DSREADY}/veg_grass_clump_03/veg_grass_clump_03.usd",
]

LEAF_DEBRIS = [
    f"{DSREADY}/sct_debris_leaves_dry_01/sct_debris_leaves_dry_01.usd",
    f"{DSREADY}/sct_debris_leaves_dry_02/sct_debris_leaves_dry_02.usd",
]


def add_dsready(stage, path, asset_url, x, y, z, yaw=0, scale=1.0, purpose=None):
    stage.DefinePrim(path, "Xform")
    prim = stage.GetPrimAtPath(path)
    wxf = UsdGeom.Xformable(prim)
    gz = terrain_height(x, y)
    wxf.AddTranslateOp().Set(Gf.Vec3d(x, y, gz))
    if yaw != 0:
        wxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, math.degrees(yaw)))
    if scale != 1.0:
        wxf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    if purpose:
        UsdGeom.Imageable(prim).CreatePurposeAttr(purpose)
    child = stage.DefinePrim(f"{path}/mesh", "Xform")
    child.GetReferences().AddReference(asset_url)


# LOAD DATA FIRST - models and rng before any generation code
with open("/tmp/gazebo_models.json") as f:
    models = json.load(f)

rng = np.random.RandomState(42)

# S-curve route - more natural winding forest road (not just a gentle arc)
ROUTE_WPS = [
    (-100, -7.0), (-95, -6.0), (-90, -4.5), (-85, -2.8), (-80, -1.5),
    (-75, -0.8), (-70, -0.5), (-65, -1.0), (-60, -2.2), (-55, -3.8),
    (-50, -5.0), (-45, -5.5), (-40, -5.2), (-35, -4.0), (-30, -2.5),
    (-25, -1.0), (-20, 0.2), (-15, 1.2), (-10, 1.8), (-5, 2.0),
    (0, 1.5), (5, 0.5), (10, -0.8), (15, -2.2), (20, -3.5),
    (25, -4.2), (30, -4.0), (35, -3.0), (40, -1.8), (45, -0.8),
    (50, -0.5), (55, -1.0), (60, -2.0), (65, -3.2), (70, -4.5), (75, -5.0),
]


def road_y_at(x):
    """interpolate road Y position at given X"""
    if x <= ROUTE_WPS[0][0]:
        return ROUTE_WPS[0][1]
    if x >= ROUTE_WPS[-1][0]:
        return ROUTE_WPS[-1][1]
    for i in range(len(ROUTE_WPS) - 1):
        x1, y1 = ROUTE_WPS[i]
        x2, y2 = ROUTE_WPS[i + 1]
        if x1 <= x <= x2:
            t = (x - x1) / (x2 - x1)
            return y1 + t * (y2 - y1)
    return 0.0


def is_near_road(x, y, margin=4.0):
    return abs(y - road_y_at(x)) < margin


# house positions for overlap checking
_house_positions = [(m["x"], m["y"]) for m in models if m["type"] == "house"]


def is_near_house(x, y, margin=10.0):
    return any(math.hypot(hx - x, hy - y) < margin for hx, hy in _house_positions)


print("creating scene...")
omni.usd.get_context().new_stage()
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# physics
phys = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
phys.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
phys.CreateGravityMagnitudeAttr(9.81)
PhysxSchema.PhysxSceneAPI.Apply(
    stage.GetPrimAtPath("/World/PhysicsScene")
).CreateTimeStepsPerSecondAttr(60)

# TERRAIN - heightfield mesh with real undulation + collision
print("  building heightfield terrain mesh...")

# grid covers -120..120 x -80..80, 2m resolution
GRID_X0, GRID_X1 = -120, 120
GRID_Y0, GRID_Y1 = -80, 80
# PHYSICS_DT = 1/240  # tried, too slow for real-time sim
GRID_RES = 2.0  # coarse mesh - smooth surface, texture handles detail
nx = int((GRID_X1 - GRID_X0) / GRID_RES) + 1
ny = int((GRID_Y1 - GRID_Y0) / GRID_RES) + 1

# pre-generate micro noise grid for cheap per-vertex roughness
_micro_noise = rng.uniform(-0.03, 0.03, size=(ny, nx))  # less micro noise - smoother surface


def terrain_height(x, y):
    h = 0.0
    # large rolling hills - halved amplitude
    h += 0.5 * math.sin(x * 0.018 + 0.5) * math.cos(y * 0.022 + 1.2)
    h += 0.35 * math.sin(x * 0.035 + 2.1) * math.sin(y * 0.03 + 0.7)
    # medium undulation
    h += 0.18 * math.sin(x * 0.07 + 3.3) * math.cos(y * 0.065 + 2.5)
    h += 0.12 * math.cos(x * 0.11 + 1.0) * math.sin(y * 0.09 + 4.0)
    # small bumps - forest floor roots, stones (~3-6m wavelength, gentle)
    h += 0.06 * math.sin(x * 0.5 + 0.7) * math.cos(y * 0.43 + 2.1)
    h += 0.04 * math.cos(x * 0.7 + 3.5) * math.sin(y * 0.6 + 0.4)
    h += 0.03 * math.sin(x * 1.0 + 1.2) * math.cos(y * 0.83 + 3.8)
    # flatten near road
    ry = road_y_at(x)
    road_dist = abs(y - ry)
    if road_dist < 4.0:
        t = road_dist / 4.0
        h *= t * t
    if road_dist < 2.0:
        h -= 0.06 * (1.0 - road_dist / 2.0)
    return max(h, -0.5)


# build vertex grid - add per-vertex micro noise for rough surface
points = []
for iy in range(ny):
    for ix in range(nx):
        x = GRID_X0 + ix * GRID_RES
        y = GRID_Y0 + iy * GRID_RES
        z = terrain_height(x, y)  # no micro noise - texture handles detail
        points.append(Gf.Vec3f(x, y, z))

# build triangle faces
face_vertex_counts = []
face_vertex_indices = []
for iy in range(ny - 1):
    for ix in range(nx - 1):
        i00 = iy * nx + ix
        i10 = iy * nx + ix + 1
        i01 = (iy + 1) * nx + ix
        i11 = (iy + 1) * nx + ix + 1
        # two triangles per cell
        face_vertex_counts.extend([3, 3])
        face_vertex_indices.extend([i00, i10, i11, i00, i11, i01])

# per-vertex color - green/brown mix based on height and position
colors = []
tree_positions = [(m["x"], m["y"]) for m in models if m["type"] in ("pine", "oak")]
for iy in range(ny):
    for ix in range(nx):
        x = GRID_X0 + ix * GRID_RES
        y = GRID_Y0 + iy * GRID_RES
        z = points[iy * nx + ix][2]
        # forest floor - patchy, high contrast, never uniform
        # use spatial noise for clustered patches (not pure random per vertex)
        # low-freq noise decides "zone": grass, dirt, leaves, moss
        zone_noise = (math.sin(x * 0.5 + 1.7) * math.cos(y * 0.45 + 0.9)
                     + 0.5 * math.sin(x * 1.2 + 3.1) * math.sin(y * 1.0 + 2.4)
                     + 0.3 * math.cos(x * 2.5 + 0.3) * math.sin(y * 2.2 + 1.8))
        # per-vertex jitter on top
        jitter = rng.uniform(-0.04, 0.04)
        zone = zone_noise + jitter

        if zone < -0.6:
            # dark soil - exposed earth under trees
            r, g, b = 0.10, 0.07, 0.03
        elif zone < -0.3:
            # wet dark earth
            r, g, b = 0.14, 0.10, 0.05
        elif zone < -0.05:
            # dead leaf litter - brown/orange
            r, g, b = 0.24, 0.16, 0.07
        elif zone < 0.2:
            # dark green grass
            r, g, b = 0.10, 0.17, 0.05
        elif zone < 0.5:
            # medium grass
            r, g, b = 0.14, 0.22, 0.07
        elif zone < 0.8:
            # dry grass / straw
            r, g, b = 0.20, 0.18, 0.08
        else:
            # moss - bright green
            r, g, b = 0.08, 0.16, 0.04

        # per-vertex noise for texture
        r += rng.uniform(-0.03, 0.03)
        g += rng.uniform(-0.03, 0.03)
        b += rng.uniform(-0.015, 0.015)
        # height tint
        r += z * 0.015
        g += z * 0.02
        # dirt road - compacted earth with tire ruts and grass center
        road_dist = abs(y - road_y_at(x))
        if road_dist < 3.5:
            t = max(0, 1.0 - road_dist / 3.5)
            t2 = t * t  # sharper falloff
            rut_dist = abs(road_dist - 0.65)
            if rut_dist < 0.25:
                # tire ruts - dark packed mud
                r, g, b = 0.18 + rng.uniform(-0.02, 0.02), 0.13 + rng.uniform(-0.02, 0.02), 0.07
            elif road_dist < 0.35:
                # center grass strip between ruts
                r = r * 0.5 + 0.11 * 0.5
                g = g * 0.5 + 0.18 * 0.5
                b = b * 0.5 + 0.05 * 0.5
            else:
                # road surface - brown compacted dirt
                dr = 0.26 + rng.uniform(-0.04, 0.04)
                dg = 0.20 + rng.uniform(-0.03, 0.03)
                db = 0.11 + rng.uniform(-0.02, 0.02)
                r = r * (1 - t2) + dr * t2
                g = g * (1 - t2) + dg * t2
                b = b * (1 - t2) + db * t2
        # near trees = leaf litter / bare soil baked into terrain
        min_tree_dist = min((math.hypot(tx - x, ty - y) for tx, ty in tree_positions), default=999)
        if min_tree_dist < 5:
            t = 1.0 - min_tree_dist / 5.0  # 1 at trunk, 0 at 5m
            # blend toward leaf litter
            leaf_roll = rng.random()
            if leaf_roll < 0.4:
                lr, lg, lb = 0.22, 0.16, 0.08  # brown leaf litter
            elif leaf_roll < 0.7:
                lr, lg, lb = 0.16, 0.11, 0.05  # dark soil
            else:
                lr, lg, lb = 0.26, 0.20, 0.10  # dry leaves
            lr += rng.uniform(-0.03, 0.03)
            lg += rng.uniform(-0.02, 0.02)
            lb += rng.uniform(-0.01, 0.01)
            blend = t * t * 0.8  # strong near trunk
            r = r * (1 - blend) + lr * blend
            g = g * (1 - blend) + lg * blend
            b = b * (1 - blend) + lb * blend
        colors.append(Gf.Vec3f(max(0, r), max(0, g), max(0, b)))

# base terrain: coast_sand_rocks - grass with rocky patches
terrain_mesh = UsdGeom.Mesh.Define(stage, "/World/Terrain/Heightfield")
terrain_mesh.CreatePointsAttr(points)
terrain_mesh.CreateFaceVertexCountsAttr(face_vertex_counts)
terrain_mesh.CreateFaceVertexIndicesAttr(face_vertex_indices)
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/Terrain/Heightfield"))
PhysxSchema.PhysxCollisionAPI.Apply(stage.GetPrimAtPath("/World/Terrain/Heightfield"))

from pxr import UsdShade

def make_omnipbr(stage, mat_path, diff, norm, rough, scale, rotate=0):
    # FIXME breaks if isaac restarts mid-run, works if clean start
    mat = UsdShade.Material.Define(stage, mat_path)
    sh = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    sh.CreateIdAttr("OmniPBR")
    sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    out = sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(out)
    mat.CreateVolumeOutput("mdl").ConnectToSource(out)
    mat.CreateDisplacementOutput("mdl").ConnectToSource(out)
    sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(diff)
    sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(norm)
    if rough:
        sh.CreateInput("reflectionroughness_texture", Sdf.ValueTypeNames.Asset).Set(rough)
        sh.CreateInput("reflection_roughness_texture_influence", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("bump_factor", Sdf.ValueTypeNames.Float).Set(2.0)
    sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
    sh.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(scale, scale))
    if rotate:
        sh.CreateInput("texture_rotate", Sdf.ValueTypeNames.Float).Set(float(rotate))
    return mat

# material 1: base grass (coast_sand_rocks + aerial blend done in texture gen)
# single baked texture with proper uv mapping
# generate UV coordinates for terrain mesh - maps texture 1:1 to world
uvs = []
for iy in range(ny):
    for ix in range(nx):
        u = ix / (nx - 1)  # 0..1 across terrain width
        v = iy / (ny - 1)  # 0..1 across terrain height
        uvs.append(Gf.Vec2f(u, v))
uv_primvar = UsdGeom.PrimvarsAPI(terrain_mesh).CreatePrimvar(
    "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
uv_primvar.Set(uvs)

from pxr import UsdShade
baked_mat = UsdShade.Material.Define(stage, "/World/Looks/BakedMat")
baked_sh = UsdShade.Shader.Define(stage, "/World/Looks/BakedMat/Shader")
baked_sh.CreateIdAttr("OmniPBR")
baked_sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
baked_sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
baked_sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
baked_out = baked_sh.CreateOutput("out", Sdf.ValueTypeNames.Token)
baked_mat.CreateSurfaceOutput("mdl").ConnectToSource(baked_out)
baked_mat.CreateVolumeOutput("mdl").ConnectToSource(baked_out)
baked_mat.CreateDisplacementOutput("mdl").ConnectToSource(baked_out)
baked_sh.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(
    "/opt/terrain_textures/TerrainBaked_4K.jpg")
baked_sh.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset).Set(
    "/opt/terrain_textures/TerrainBaked_4K_Normal.jpg")
baked_sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
baked_sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
baked_sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.0)
baked_sh.CreateInput("bump_factor", Sdf.ValueTypeNames.Float).Set(2.0)
# NO project_uvw - use mesh UVs instead (1:1 world mapping)
baked_sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(False)
UsdShade.MaterialBindingAPI(terrain_mesh).Bind(baked_mat, UsdShade.Tokens.strongerThanDescendants)
print("  baked 4K texture with UV mapping (no separate road/leaf mesh)")

# no separate road/leaf mesh - all baked
leaf_idx = 0

print(f"  heightfield: {nx}x{ny} = {len(points)} vertices")

# ROAD - just two brown tire tracks on earth, like a real forest dirt road
SPAWN_CLEAR = 8.0  # meters around spawn to keep free of bumps
SPAWN_X, SPAWN_Y = -95, -6

# road color handled by heightfield vertex colors - no 3D objects on road

# LIGHTING
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1500)
dome.CreateTextureFileAttr("/workspace/simulation/isaac/assets/hdri/meadow_2_2k.hdr")
dome.CreateTextureFormatAttr("latlong")

sun = UsdLux.DistantLight.Define(stage, "/World/SunLight")
sun.CreateIntensityAttr(5000)
sun.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 0.97))  # neutral white - no warm yellow tint
sun.CreateAngleAttr(3.0)  # wide angle = soft diffuse shadows, less specular
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-25, 25, 0))  # low angle - sun through gaps between trees

fill = UsdLux.DistantLight.Define(stage, "/World/FillLight")
fill.CreateIntensityAttr(1500)
fill.CreateColorAttr(Gf.Vec3f(0.7, 0.8, 1.0))
UsdGeom.Xformable(fill).AddRotateXYZOp().Set(Gf.Vec3f(-15, -160, 0))  # low fill from opposite side

print("  ground, road, lighting done")

# BUILDINGS - detailed with peaked roofs, foundations, porches, fences
# =============================================================================
# BUILDINGS - 3 ruins (rocket strike) + 2 intact (brick, detailed) + 1 shed
print("\nplacing buildings...")


def build_ruin(stage, path, mx, my, rng):
    """bombed building - 4 walls at different heights, collapsed roof, debris pile"""
    gz = terrain_height(mx, my)
    w, d = rng.uniform(6, 8), rng.uniform(5, 7)
    h_max = rng.uniform(3.0, 4.0)
    brick_c = Gf.Vec3f(0.42, 0.22, 0.14)
    # foundation
    found = UsdGeom.Cube.Define(stage, f"{path}/foundation")
    found.CreateSizeAttr(1.0)
    found.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.24, 0.18)])
    fxf = UsdGeom.Xformable(found)
    fxf.AddTranslateOp().Set(Gf.Vec3d(mx, my, gz + 0.12))
    fxf.AddScaleOp().Set(Gf.Vec3f(w + 0.3, d + 0.3, 0.25))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/foundation"))
    # 4 walls - each a different surviving height, jagged top
    wall_specs = [
        (mx - w/2, my, 0.22, d, "left"),
        (mx + w/2, my, 0.22, d, "right"),
        (mx, my - d/2, w, 0.22, "back"),
        (mx, my + d/2, w, 0.22, "front"),
    ]
    for wi, (wx, wy, ww, wd, name) in enumerate(wall_specs):
        # each wall = 2-3 segments at different heights (jagged broken top)
        n_seg = rng.randint(2, 4)
        seg_w = (d if name in ("left","right") else w) / n_seg
        for si in range(n_seg):
            seg_h = h_max * rng.uniform(0.15, 0.95)
            # soot gradient - lower = brick, upper = darker
            soot = min(1.0, seg_h / h_max)
            sc = Gf.Vec3f(
                brick_c[0] * (1 - soot * 0.4) + rng.uniform(-0.04, 0.04),
                brick_c[1] * (1 - soot * 0.3) + rng.uniform(-0.03, 0.03),
                brick_c[2] * (1 - soot * 0.3),
            )
            seg = UsdGeom.Cube.Define(stage, f"{path}/wall_{wi}_{si}")
            seg.CreateSizeAttr(1.0)
            seg.CreateDisplayColorAttr([sc])
            sxf = UsdGeom.Xformable(seg)
            if name in ("left", "right"):
                sy = wy - d/2 + seg_w * si + seg_w / 2
                sxf.AddTranslateOp().Set(Gf.Vec3d(wx, sy, gz + 0.25 + seg_h/2))
                sxf.AddScaleOp().Set(Gf.Vec3f(0.22, seg_w - 0.05, seg_h))
            else:
                sx = wx - w/2 + seg_w * si + seg_w / 2
                sxf.AddTranslateOp().Set(Gf.Vec3d(sx, wy, gz + 0.25 + seg_h/2))
                sxf.AddScaleOp().Set(Gf.Vec3f(seg_w - 0.05, 0.22, seg_h))
            UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/wall_{wi}_{si}"))
    # collapsed roof - large slab leaning on debris
    roof = UsdGeom.Cube.Define(stage, f"{path}/roof_slab")
    roof.CreateSizeAttr(1.0)
    roof.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.14, 0.09)])
    rxf = UsdGeom.Xformable(roof)
    rxf.AddTranslateOp().Set(Gf.Vec3d(mx, my + rng.uniform(-1, 1), gz + h_max * 0.3))
    rxf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(20, 40), rng.uniform(-8, 8), rng.uniform(-5, 5)))
    rxf.AddScaleOp().Set(Gf.Vec3f(w * 0.7, d * 0.6, 0.08))
    # debris pile inside and around
    for di in range(rng.randint(10, 18)):
        deb = UsdGeom.Cube.Define(stage, f"{path}/debris_{di}")
        deb.CreateSizeAttr(1.0)
        _dc = [(0.40,0.35,0.28),(0.44,0.24,0.15),(0.22,0.16,0.10),(0.30,0.26,0.20)]
        dc = _dc[rng.randint(0, len(_dc))]
        deb.CreateDisplayColorAttr([Gf.Vec3f(dc[0] + rng.uniform(-0.03, 0.03), dc[1], dc[2])])
        dxf = UsdGeom.Xformable(deb)
        dxf.AddTranslateOp().Set(Gf.Vec3d(
            mx + rng.uniform(-w * 0.6, w * 0.6),
            my + rng.uniform(-d * 0.6, d * 0.6),
            gz + rng.uniform(0.05, 0.5)))
        dxf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(0, 360)))
        dxf.AddScaleOp().Set(Gf.Vec3f(rng.uniform(0.15, 0.9), rng.uniform(0.15, 0.7), rng.uniform(0.05, 0.3)))
    # broken beams sticking out
    for bi in range(rng.randint(2, 4)):
        beam = UsdGeom.Cube.Define(stage, f"{path}/beam_{bi}")
        beam.CreateSizeAttr(1.0)
        beam.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.20, 0.10)])
        bxf = UsdGeom.Xformable(beam)
        bxf.AddTranslateOp().Set(Gf.Vec3d(
            mx + rng.uniform(-w/3, w/3), my + rng.uniform(-d/3, d/3),
            gz + rng.uniform(0.5, h_max * 0.6)))
        bxf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(-30, 30), rng.uniform(-20, 20), rng.uniform(0, 360)))
        bxf.AddScaleOp().Set(Gf.Vec3f(rng.uniform(1.5, 3.0), 0.12, 0.12))


def build_intact(stage, path, mx, my, rng, style_name):
    """intact building on terrain - brick or plaster, proper peaked roof"""
    gz = terrain_height(mx, my)
    if style_name == "brick":
        wall_c = Gf.Vec3f(0.48, 0.22, 0.13)   # red brick
        trim_c = Gf.Vec3f(0.60, 0.55, 0.45)
        roof_c = Gf.Vec3f(0.22, 0.13, 0.08)    # dark roof tiles
    else:
        wall_c = Gf.Vec3f(0.58, 0.54, 0.44)    # plaster
        trim_c = Gf.Vec3f(0.42, 0.32, 0.20)
        roof_c = Gf.Vec3f(0.28, 0.20, 0.12)    # wood shingles
    w, d, h = rng.uniform(6, 8), rng.uniform(5, 6), rng.uniform(3.0, 4.0)
    # foundation
    found = UsdGeom.Cube.Define(stage, f"{path}/foundation")
    found.CreateSizeAttr(1.0)
    found.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.26, 0.20)])
    fxf = UsdGeom.Xformable(found)
    fxf.AddTranslateOp().Set(Gf.Vec3d(mx, my, gz + 0.15))
    fxf.AddScaleOp().Set(Gf.Vec3f(w + 0.3, d + 0.3, 0.3))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/foundation"))
    # walls
    walls = UsdGeom.Cube.Define(stage, f"{path}/walls")
    walls.CreateSizeAttr(1.0)
    walls.CreateDisplayColorAttr([wall_c])
    wxf = UsdGeom.Xformable(walls)
    wxf.AddTranslateOp().Set(Gf.Vec3d(mx, my, gz + 0.3 + h / 2))
    wxf.AddScaleOp().Set(Gf.Vec3f(w, d, h))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/walls"))
    # corner pillars
    for ci, (cx_off, cy_off) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
        corner = UsdGeom.Cube.Define(stage, f"{path}/corner_{ci}")
        corner.CreateSizeAttr(1.0)
        corner.CreateDisplayColorAttr([Gf.Vec3f(0.34, 0.28, 0.22)])
        cxf = UsdGeom.Xformable(corner)
        cxf.AddTranslateOp().Set(Gf.Vec3d(mx + cx_off * w/2, my + cy_off * d/2, gz + 0.3 + h/2))
        cxf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, h + 0.1))
    # roof - two slabs tilting DOWN from ridge (not up!)
    ridge_z = gz + 0.3 + h + 1.2
    for ri, side in enumerate([1, -1]):
        slab = UsdGeom.Cube.Define(stage, f"{path}/roof_{ri}")
        slab.CreateSizeAttr(1.0)
        slab.CreateDisplayColorAttr([roof_c])
        sxf = UsdGeom.Xformable(slab)
        slab_len = (d / 2 + 0.6) / math.cos(math.radians(25))
        # slab center offset from ridge, tilted outward
        sxf.AddTranslateOp().Set(Gf.Vec3d(mx, my + side * d/4, ridge_z - 0.5))
        sxf.AddRotateXYZOp().Set(Gf.Vec3f(side * -25, 0, 0))
        sxf.AddScaleOp().Set(Gf.Vec3f(w + 0.8, slab_len, 0.1))
    # ridge beam
    ridge = UsdGeom.Cube.Define(stage, f"{path}/ridge")
    ridge.CreateSizeAttr(1.0)
    ridge.CreateDisplayColorAttr([Gf.Vec3f(roof_c[0] - 0.05, roof_c[1] - 0.03, roof_c[2])])
    rgxf = UsdGeom.Xformable(ridge)
    rgxf.AddTranslateOp().Set(Gf.Vec3d(mx, my, ridge_z))
    rgxf.AddScaleOp().Set(Gf.Vec3f(w + 1.0, 0.1, 0.08))
    # door
    door = UsdGeom.Cube.Define(stage, f"{path}/door")
    door.CreateSizeAttr(1.0)
    door.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.10, 0.05)])
    dxf = UsdGeom.Xformable(door)
    dxf.AddTranslateOp().Set(Gf.Vec3d(mx + w/2 + 0.02, my, gz + 1.4))
    dxf.AddScaleOp().Set(Gf.Vec3f(0.05, 0.85, 2.0))
    # windows
    for wi in range(4):
        wy_off = (d/4) * (1 if wi % 2 == 0 else -1)
        wx_side = w/2 + 0.02 if wi < 2 else -(w/2 + 0.02)
        wz = gz + 0.3 + h * 0.45
        win = UsdGeom.Cube.Define(stage, f"{path}/win_{wi}")
        win.CreateSizeAttr(1.0)
        win.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.13, 0.18)])
        UsdGeom.Xformable(win).AddTranslateOp().Set(Gf.Vec3d(mx + wx_side, my + wy_off, wz))
        UsdGeom.Xformable(stage.GetPrimAtPath(f"{path}/win_{wi}")).AddScaleOp().Set(Gf.Vec3f(0.05, 0.55, 0.7))
        # sill
        sill = UsdGeom.Cube.Define(stage, f"{path}/sill_{wi}")
        sill.CreateSizeAttr(1.0)
        sill.CreateDisplayColorAttr([trim_c])
        slxf = UsdGeom.Xformable(sill)
        slxf.AddTranslateOp().Set(Gf.Vec3d(mx + wx_side * 1.01, my + wy_off, wz - 0.4))
        slxf.AddScaleOp().Set(Gf.Vec3f(0.06, 0.7, 0.05))
    # chimney
    chim = UsdGeom.Cube.Define(stage, f"{path}/chimney")
    chim.CreateSizeAttr(1.0)
    chim.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.26, 0.20)])
    chxf = UsdGeom.Xformable(chim)
    chxf.AddTranslateOp().Set(Gf.Vec3d(mx - w/3, my + d/4, ridge_z + 0.6))
    chxf.AddScaleOp().Set(Gf.Vec3f(0.45, 0.45, 1.5))


# all 6 = intact buildings, varied styles, on terrain height
STYLES = [
    {"wall": (0.48, 0.22, 0.13), "roof": (0.22, 0.13, 0.08), "trim": (0.60, 0.55, 0.45), "name": "brick"},
    {"wall": (0.62, 0.58, 0.48), "roof": (0.28, 0.20, 0.12), "trim": (0.42, 0.32, 0.20), "name": "plaster"},
    {"wall": (0.55, 0.48, 0.38), "roof": (0.18, 0.12, 0.07), "trim": (0.50, 0.40, 0.28), "name": "wood"},
    {"wall": (0.70, 0.65, 0.55), "roof": (0.32, 0.22, 0.14), "trim": (0.65, 0.58, 0.45), "name": "cottage"},
    {"wall": (0.45, 0.25, 0.15), "roof": (0.25, 0.18, 0.10), "trim": (0.55, 0.45, 0.32), "name": "dark_brick"},
    {"wall": (0.58, 0.52, 0.42), "roof": (0.20, 0.14, 0.08), "trim": (0.48, 0.38, 0.25), "name": "cabin"},
]
house_models_list = [m for m in models if m["type"] == "house"]
house_idx = 0
for m in house_models_list:
    path = f"/World/Buildings/house_{house_idx:02d}"
    s = STYLES[house_idx % len(STYLES)]
    gz = terrain_height(m["x"], m["y"])
    wc = Gf.Vec3f(*s["wall"])
    rc = Gf.Vec3f(*s["roof"])
    tc = Gf.Vec3f(*s["trim"])
    w = rng.uniform(5.5, 8.0)
    d = rng.uniform(4.5, 6.5)
    h = rng.uniform(3.0, 4.0)
    # foundation
    f = UsdGeom.Cube.Define(stage, f"{path}/found")
    f.CreateSizeAttr(1.0)
    f.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.26, 0.20)])
    fxf = UsdGeom.Xformable(f)
    fxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], gz + 0.15))
    fxf.AddScaleOp().Set(Gf.Vec3f(w + 0.3, d + 0.3, 0.3))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/found"))
    # walls
    walls = UsdGeom.Cube.Define(stage, f"{path}/walls")
    walls.CreateSizeAttr(1.0)
    walls.CreateDisplayColorAttr([wc])
    wxf = UsdGeom.Xformable(walls)
    wxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], gz + 0.3 + h/2))
    wxf.AddScaleOp().Set(Gf.Vec3f(w, d, h))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/walls"))
    # horizontal lines on walls (brick courses / siding)
    for li in range(int(h / 0.6)):
        ln = UsdGeom.Cube.Define(stage, f"{path}/course_{li}")
        ln.CreateSizeAttr(1.0)
        ln.CreateDisplayColorAttr([Gf.Vec3f(wc[0] - 0.05, wc[1] - 0.04, wc[2] - 0.03)])
        lxf = UsdGeom.Xformable(ln)
        lxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], gz + 0.5 + li * 0.6))
        lxf.AddScaleOp().Set(Gf.Vec3f(w + 0.01, d + 0.01, 0.03))
    # corner pillars
    for ci, (cx, cy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
        cp = UsdGeom.Cube.Define(stage, f"{path}/corner_{ci}")
        cp.CreateSizeAttr(1.0)
        cp.CreateDisplayColorAttr([Gf.Vec3f(tc[0] - 0.1, tc[1] - 0.08, tc[2] - 0.06)])
        cxf = UsdGeom.Xformable(cp)
        cxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + cx*w/2, m["y"] + cy*d/2, gz + 0.3 + h/2))
        cxf.AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, h + 0.1))
    # peaked roof - two slabs from ridge going DOWN
    ridge_z = gz + 0.3 + h
    ridge_h = 1.5 + rng.uniform(-0.2, 0.3)
    for ri, side in enumerate([1, -1]):
        slab = UsdGeom.Cube.Define(stage, f"{path}/roof_{ri}")
        slab.CreateSizeAttr(1.0)
        slab.CreateDisplayColorAttr([rc])
        sxf = UsdGeom.Xformable(slab)
        slab_len = (d/2 + 0.5) / math.cos(math.radians(25))
        sxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"] + side * d/4, ridge_z + ridge_h * 0.4))
        sxf.AddRotateXYZOp().Set(Gf.Vec3f(side * -25, 0, 0))
        sxf.AddScaleOp().Set(Gf.Vec3f(w + 0.8, slab_len, 0.10))
    # ridge beam
    rb = UsdGeom.Cube.Define(stage, f"{path}/ridge")
    rb.CreateSizeAttr(1.0)
    rb.CreateDisplayColorAttr([Gf.Vec3f(rc[0] - 0.04, rc[1] - 0.03, rc[2])])
    rbxf = UsdGeom.Xformable(rb)
    rbxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], ridge_z + ridge_h * 0.78))
    rbxf.AddScaleOp().Set(Gf.Vec3f(w + 1.0, 0.08, 0.06))
    # gable ends
    for gi, gx in enumerate([-w/2, w/2]):
        ga = UsdGeom.Cube.Define(stage, f"{path}/gable_{gi}")
        ga.CreateSizeAttr(1.0)
        ga.CreateDisplayColorAttr([wc])
        gxf = UsdGeom.Xformable(ga)
        gxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + gx, m["y"], ridge_z + ridge_h * 0.3))
        gxf.AddScaleOp().Set(Gf.Vec3f(0.06, d * 0.5, ridge_h * 0.5))
    # door
    dr = UsdGeom.Cube.Define(stage, f"{path}/door")
    dr.CreateSizeAttr(1.0)
    dr.CreateDisplayColorAttr([Gf.Vec3f(0.18, 0.10, 0.05)])
    dxf = UsdGeom.Xformable(dr)
    dxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + w/2 + 0.02, m["y"], gz + 1.35))
    dxf.AddScaleOp().Set(Gf.Vec3f(0.05, 0.85, 2.0))
    # door frame
    for fi, fy in enumerate([-0.5, 0.5]):
        df = UsdGeom.Cube.Define(stage, f"{path}/dframe_{fi}")
        df.CreateSizeAttr(1.0)
        df.CreateDisplayColorAttr([tc])
        dfxf = UsdGeom.Xformable(df)
        dfxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + w/2 + 0.03, m["y"] + fy, gz + 1.35))
        dfxf.AddScaleOp().Set(Gf.Vec3f(0.06, 0.08, 2.1))
    # windows - 2 per side, dark glass + sill
    for wi in range(4):
        wy = (d/4) * (1 if wi % 2 == 0 else -1)
        wx = w/2 + 0.02 if wi < 2 else -(w/2 + 0.02)
        wz = gz + 0.3 + h * 0.45
        wn = UsdGeom.Cube.Define(stage, f"{path}/win_{wi}")
        wn.CreateSizeAttr(1.0)
        wn.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.10, 0.15)])
        wnxf = UsdGeom.Xformable(wn)
        wnxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + wx, m["y"] + wy, wz))
        wnxf.AddScaleOp().Set(Gf.Vec3f(0.05, 0.55, 0.65))
        # sill
        sl = UsdGeom.Cube.Define(stage, f"{path}/sill_{wi}")
        sl.CreateSizeAttr(1.0)
        sl.CreateDisplayColorAttr([tc])
        slxf = UsdGeom.Xformable(sl)
        slxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + wx * 1.01, m["y"] + wy, wz - 0.38))
        slxf.AddScaleOp().Set(Gf.Vec3f(0.06, 0.7, 0.05))
    # chimney
    ch = UsdGeom.Cube.Define(stage, f"{path}/chimney")
    ch.CreateSizeAttr(1.0)
    ch.CreateDisplayColorAttr([Gf.Vec3f(0.32, 0.25, 0.18)])
    chxf = UsdGeom.Xformable(ch)
    chxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] - w/3, m["y"] + d/4, ridge_z + ridge_h + 0.3))
    chxf.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 1.4))
    # step at door
    st = UsdGeom.Cube.Define(stage, f"{path}/step")
    st.CreateSizeAttr(1.0)
    st.CreateDisplayColorAttr([Gf.Vec3f(0.28, 0.24, 0.18)])
    stxf = UsdGeom.Xformable(st)
    stxf.AddTranslateOp().Set(Gf.Vec3d(m["x"] + w/2 + 0.4, m["y"], gz + 0.12))
    stxf.AddScaleOp().Set(Gf.Vec3f(0.5, 1.2, 0.25))
    house_idx += 1
print(f"  {house_idx} intact buildings")

# barrels
barrel_idx = 0
for m in models:
    if m["type"] == "barrel":
        b = UsdGeom.Cylinder.Define(stage, f"/World/Props/barrel_{barrel_idx:02d}")
        b.CreateHeightAttr(1.0)
        b.CreateRadiusAttr(0.3)
        b.CreateAxisAttr("Z")
        b.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.5, 0.0)])
        bz = terrain_height(m["x"], m["y"])
        UsdGeom.Xformable(b).AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], bz + 0.5))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"/World/Props/barrel_{barrel_idx:02d}"))
        barrel_idx += 1
# print(f"DEBUG len(traj)={len(traj)}")
print(f"  {barrel_idx} barrels")

# ALL TREES -> dsready photorealistic (no primitives)
print("\nplacing trees (thinned for performance)...")
# thin trees uniformly - keep ~200 of 297, skip every 3rd
# also remove 5 closest to each western corner (-120, -80) and (-120, +80)
all_trees = [m for m in models if m["type"] in ("pine", "oak")]
# sort by distance to western corners and remove closest 5 to each
west_sw = sorted(all_trees, key=lambda m: math.hypot(m["x"] + 120, m["y"] + 80))
west_nw = sorted(all_trees, key=lambda m: math.hypot(m["x"] + 120, m["y"] - 80))
remove_set = set()
for t in west_sw[:5]:
    remove_set.add(t["name"])
for t in west_nw[:5]:
    remove_set.add(t["name"])

tree_idx = 0
skip_counter = 0
for m in all_trees:
    if m["name"] in remove_set:
        continue
    skip_counter += 1
    if skip_counter % 3 == 0:
        continue  # skip every 3rd -> keep ~67%
    if m["type"] == "pine":
        asset = PINE_ASSETS[tree_idx % len(PINE_ASSETS)]
    else:
        asset = OAK_ASSETS[tree_idx % len(OAK_ASSETS)]
    add_dsready(stage, f"/World/Trees/tree_{tree_idx:03d}", asset,
                m["x"], m["y"], m["z"], m["yaw"])
    tree_idx += 1
print(f"  {tree_idx} photorealistic trees (thinned from {len(all_trees)})")

# tree trunk collision
# print(f"DEBUG state={state} pose={pose}")
print("  adding tree trunk collisions...")
skip_counter = 0
for m in all_trees:
    if m["name"] in remove_set:
        continue
    skip_counter += 1
    if skip_counter % 3 == 0:
        continue
    col_path = f"/World/TreeCollision/col_{m['name'].replace(' ', '_')}"
    tc = UsdGeom.Cylinder.Define(stage, col_path)
    tc.CreateRadiusAttr(0.5)
    tc.CreateHeightAttr(5.0)
    tc.CreateAxisAttr("Z")
    tc.CreatePurposeAttr("guide")
    tcxf = UsdGeom.Xformable(tc)
    tcxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], 2.5))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(col_path))

# BACKGROUND FOREST - multiple dense rings so trees fill to horizon
print("  adding BG trees at map edge...")
bg_idx = 0
# ring 1: dense at edge
for angle_deg in range(0, 360, 3):
    a = math.radians(angle_deg)
    for ring_off in [0, 6, -6, 12]:
        rx = (120 + ring_off) * math.cos(a) + rng.uniform(-3, 3)
        ry = (78 + ring_off) * math.sin(a) + rng.uniform(-3, 3)
        asset = PINE_ASSETS[bg_idx % len(PINE_ASSETS)] if bg_idx % 3 != 0 else OAK_ASSETS[bg_idx % len(OAK_ASSETS)]
        sc = rng.uniform(1.0, 1.6)
        add_dsready(stage, f"/World/BGTrees/bg_{bg_idx:04d}", asset,
                    rx, ry, 0, rng.uniform(0, 6.28), scale=sc)
        bg_idx += 1
print(f"  {bg_idx} BG trees")

# BG collision disabled (no BG trees)
bg_col_idx = 0
if False:
 for angle_deg in range(0, 360, 8):
    a = math.radians(angle_deg)
    for ring_off in [0]:
        rx = (125 + ring_off) * math.cos(a)
        ry = (80 + ring_off) * math.sin(a)
        col_path = f"/World/BGCollision/bgcol_{bg_col_idx:04d}"
        tc = UsdGeom.Cylinder.Define(stage, col_path)
        tc.CreateRadiusAttr(0.5)
        tc.CreateHeightAttr(5.0)
        tc.CreateAxisAttr("Z")
        tc.CreatePurposeAttr("guide")
        UsdGeom.Xformable(tc).AddTranslateOp().Set(Gf.Vec3d(rx, ry, 2.5))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(col_path))
        bg_col_idx += 1

# fallen trees - use standing tree models rotated 90° on their side
FALLEN_TREE_ASSETS = [
    f"{DSREADY}/veg_tree_pine_yellow_01/veg_tree_pine_yellow_01.usd",
    f"{DSREADY}/veg_tree_pine_yellow_02/veg_tree_pine_yellow_02.usd",
    f"{DSREADY}/veg_tree_oak_shumard_01/veg_tree_oak_shumard_01.usd",
    f"{DSREADY}/veg_tree_fir_douglas_01/veg_tree_fir_douglas_01.usd",
]
fallen_idx = 0
for m in models:
    if "fallen" not in m["type"]:
        continue
    asset = FALLEN_TREE_ASSETS[fallen_idx % len(FALLEN_TREE_ASSETS)]
    fallen_scale = 1.2 if fallen_idx % 3 != 2 else 0.9
    path = f"/World/Fallen/fallen_{fallen_idx:03d}"
    stage.DefinePrim(path, "Xform")
    prim = stage.GetPrimAtPath(path)
    wxf = UsdGeom.Xformable(prim)
    gz = terrain_height(m["x"], m["y"])
    wxf.AddTranslateOp().Set(Gf.Vec3d(m["x"], m["y"], gz))
    # rotate: 90° around Y to lay on side, then yaw around Z
    wxf.AddRotateXYZOp().Set(Gf.Vec3f(0, 90, math.degrees(m.get("yaw", 0))))
    wxf.AddScaleOp().Set(Gf.Vec3f(fallen_scale, fallen_scale, fallen_scale))
    UsdGeom.Imageable(prim).CreatePurposeAttr("render")
    child = stage.DefinePrim(f"{path}/mesh", "Xform")
    child.GetReferences().AddReference(asset)
    fallen_idx += 1
print(f"  {fallen_idx} fallen trees (laid-down tree models)")

# rocks -> dsready (push away from road)
rock_idx = 0
for m in models:
    if m["type"] != "rock":
        continue
    rx, ry = m["x"], m["y"]
    if is_near_road(rx, ry, 5.0):
        ry = road_y_at(rx) + 8.0 * (1 if ry >= road_y_at(rx) else -1)
    asset = ROCK_ASSETS[rock_idx % len(ROCK_ASSETS)]
    add_dsready(stage, f"/World/Rocks/rock_{rock_idx:03d}", asset,
                rx, ry, 0, m["yaw"], scale=0.15)
    # collision sphere for rock
    rc = UsdGeom.Sphere.Define(stage, f"/World/RockCol/rc_{rock_idx:03d}")
    rc.CreateRadiusAttr(0.2)
    rc.CreatePurposeAttr("guide")
    UsdGeom.Xformable(rc).AddTranslateOp().Set(Gf.Vec3d(rx, ry, 0.1))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"/World/RockCol/rc_{rock_idx:03d}"))
    rock_idx += 1
print(f"  {rock_idx} rocks")

# extra trees near road edges - makes road feel enclosed in forest
# roadside trees and props - placed from gazebo_models.json (deterministic)
roadside_trees = [m for m in models if m["name"].startswith("roadside_tree_")]
print(f"  placing {len(roadside_trees)} roadside trees from gazebo_models.json...")
roadside_idx = 0
for rt in roadside_trees:
    asset = PINE_ASSETS[roadside_idx % len(PINE_ASSETS)] if rt["type"] == "pine" else OAK_ASSETS[roadside_idx % len(OAK_ASSETS)]
    add_dsready(stage, f"/World/RoadsideTrees/rt_{roadside_idx:03d}", asset,
                rt["x"], rt["y"], 0, rt.get("yaw", 0), scale=1.0)
    col_path = f"/World/RoadsideTrees/rtcol_{roadside_idx:03d}"
    tc = UsdGeom.Cylinder.Define(stage, col_path)
    tc.CreateRadiusAttr(0.4)
    tc.CreateHeightAttr(4.0)
    tc.CreateAxisAttr("Z")
    tc.CreatePurposeAttr("guide")
    UsdGeom.Xformable(tc).AddTranslateOp().Set(Gf.Vec3d(rt["x"], rt["y"], 2.0))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(col_path))
    roadside_idx += 1
print(f"  {roadside_idx} roadside trees")

roadside_props = [m for m in models if m["name"].startswith("roadside_shrub_") or m["name"].startswith("roadside_rock_")]
print(f"  placing {len(roadside_props)} roadside props from gazebo_models.json...")
rsp_idx = 0
for rp in roadside_props:
    if "shrub" in rp["name"]:
        asset = SHRUB_ASSETS[rsp_idx % len(SHRUB_ASSETS)]
    else:
        asset = ROCK_ASSETS[rsp_idx % len(ROCK_ASSETS)]
    add_dsready(stage, f"/World/Roadside/rsp_{rsp_idx:04d}", asset,
                rp["x"], rp["y"], 0.0, rp.get("yaw", 0),
                scale=1.0 if "shrub" in rp["name"] else 0.12, purpose="render")
    rsp_idx += 1
print(f"  {rsp_idx} roadside props")

# GROUND COVER - shrubs, ferns, grass, leaves around trees
# + bare earth patches at tree bases + scattered fill between trees
print("\nscattering ground cover (very dense)...")
cover_idx = 0
all_tree_models = [m for m in models if m["type"] in ("pine", "oak")]

# earth under trees - baked into heightfield vertex colors, no separate objects
print("  (tree litter baked into terrain colors)")

# shrubs - placed from gazebo_models.json (deterministic positions, shared with collision)
shrub_models = [m for m in models if m["type"] == "shrub"]
print(f"  placing {len(shrub_models)} shrubs from gazebo_models.json...")
for sm in shrub_models:
    asset = SHRUB_ASSETS[cover_idx % len(SHRUB_ASSETS)]
    add_dsready(stage, f"/World/Cover/cover_{cover_idx:04d}", asset,
                sm["x"], sm["y"], 0.0, sm.get("yaw", 0), purpose="render")
    cover_idx += 1

# other vegetation around every tree - leaves, grass, ferns (no shrubs - those come from JSON)
for i, m in enumerate(all_tree_models):
    n_cover = rng.randint(2, 4)
    for _ in range(n_cover):
        dx, dy = rng.uniform(-6, 6), rng.uniform(-6, 6)
        new_x = m["x"] + dx
        new_y = m["y"] + dy
        if is_near_road(new_x, new_y, 2.5) or is_near_house(new_x, new_y, 8.0):
            continue
        cover_type = rng.choice([
            "leaves", "leaves", "leaves", "leaves", "leaves",
            "grass", "grass",
            "fern",
        ])
        if cover_type == "fern":
            asset = FERN_ASSETS[cover_idx % len(FERN_ASSETS)]
        elif cover_type == "grass":
            asset = GRASS_ASSETS[cover_idx % len(GRASS_ASSETS)]
        else:
            asset = LEAF_DEBRIS[cover_idx % len(LEAF_DEBRIS)]
        add_dsready(stage, f"/World/Cover/cover_{cover_idx:04d}", asset,
                    new_x, new_y, 0.0,
                    rng.uniform(0, 2 * math.pi), purpose="render")
        cover_idx += 1

# scattered fill - mostly leaves
print("  scattering fill leaves between trees...")
for fi in range(180):
    fx = rng.uniform(-105, 85)
    fy = rng.uniform(-60, 60)
    if is_near_road(fx, fy, 2.0) or is_near_house(fx, fy, 8.0):
        continue
    cover_type = rng.choice(["leaves", "leaves", "leaves", "leaves", "grass"])
    if cover_type == "grass":
        asset = GRASS_ASSETS[cover_idx % len(GRASS_ASSETS)]
    else:
        asset = LEAF_DEBRIS[cover_idx % len(LEAF_DEBRIS)]
    add_dsready(stage, f"/World/Cover/cover_{cover_idx:04d}", asset,
                fx, fy, 0.0, rng.uniform(0, 2 * math.pi), purpose="render")
    cover_idx += 1
print(f"  {cover_idx} ground cover pieces total")

# cameras
cam_fwd = UsdGeom.Camera.Define(stage, "/World/ForwardCamera")
cam_fwd.CreateFocalLengthAttr(18.0)
cam_fwd.CreateHorizontalApertureAttr(20.955)
cam_fwd.CreateClippingRangeAttr(Gf.Vec2f(0.3, 500.0))
tilt = math.radians(10)
cos_t, sin_t = math.cos(tilt), math.sin(tilt)
UsdGeom.Xformable(cam_fwd).AddTransformOp().Set(Gf.Matrix4d(
     0,      -1,  0,     0,
     sin_t,   0,  cos_t, 0,
    -cos_t,   0,  sin_t, 0,
    -105,    -8,  1.5,   1
))

cam_top = UsdGeom.Camera.Define(stage, "/World/TopDownCamera")
cam_top.CreateFocalLengthAttr(24.0)
cam_top.CreateHorizontalApertureAttr(36.0)
cam_top.CreateClippingRangeAttr(Gf.Vec2f(10.0, 3000.0))
# fit terrain edges (240x160m) into frame
UsdGeom.Xformable(cam_top).AddTranslateOp().Set(Gf.Vec3d(-10, 0, 140))

# oblique view - looking down at ~45° from south-east
cam_obl = UsdGeom.Camera.Define(stage, "/World/ObliqueCamera")
cam_obl.CreateFocalLengthAttr(24.0)
cam_obl.CreateHorizontalApertureAttr(36.0)
cam_obl.CreateClippingRangeAttr(Gf.Vec2f(1.0, 1000.0))
# position: above and south, looking north-west and down
obl_pitch = math.radians(40)  # 40° down from horizontal
cp, sp = math.cos(obl_pitch), math.sin(obl_pitch)
obl_yaw = math.radians(30)  # looking slightly north-west
cy2, sy2 = math.cos(obl_yaw), math.sin(obl_yaw)
UsdGeom.Xformable(cam_obl).AddTransformOp().Set(Gf.Matrix4d(
     sy2,       -cy2,        0,    0,
     cy2*sp,     sy2*sp,     cp,   0,
    -cy2*cp,    -sy2*cp,     sp,   0,
    -20,        -60,         80,   1
))

total = tree_idx + fallen_idx + rock_idx + cover_idx + house_idx + barrel_idx + bg_idx
print(f"\nsaving scene ({total} objects): {SCENE_OUT}")
stage.Export(SCENE_OUT)

print("\nloading dsready assets from S3 (this takes ~2min)...")
for i in range(1200):
    app.update()
    if i % 300 == 0:
        print(f"  {i}/1200", flush=True)

# render
rp_f = rep.create.render_product("/World/ForwardCamera", (640, 480))
rp_t = rep.create.render_product("/World/TopDownCamera", (1280, 960))
ann_f = rep.AnnotatorRegistry.get_annotator("rgb")
ann_f.attach([rp_f])
ann_t = rep.AnnotatorRegistry.get_annotator("rgb")
ann_t.attach([rp_t])

timeline = omni.timeline.get_timeline_interface()
timeline.play()
print("rendering...", flush=True)
for _ in range(500):
    app.update()

os.makedirs(OUT_DIR, exist_ok=True)
rp_obl = rep.create.render_product("/World/ObliqueCamera", (1920, 1080))
ann_o = rep.AnnotatorRegistry.get_annotator("rgb")
ann_o.attach([rp_obl])
for _ in range(200):
    app.update()

for name, ann_obj, fname in [
    ("forward", ann_f, "forest_forward_hq.png"),
    ("topdown", ann_t, "forest_topdown_hq.png"),
    ("oblique", ann_o, "forest_oblique_hq.png"),
]:
    d = ann_obj.get_data()
    m = np.mean(d[:, :, :3]) if d is not None else -1
    print(f"{name}: rgb={m:.0f}")
    if d is not None and m > 3:
        Image.fromarray(d[:, :, :3]).save(f"{OUT_DIR}/{fname}")
        print(f"  SAVED {fname}")

import time
count = 0
t0 = time.time()
while time.time() - t0 < 5:
    app.update()
    if ann_f.get_data() is not None:
        count += 1
print(f"\nrender rate: {count / (time.time() - t0):.1f} Hz")

print(f"""
=== FULL QUALITY CONVERSION ===
  trees:       {tree_idx} (ALL dsready photorealistic)
  bg trees:    {bg_idx} (4 rings + wall)
  fallen:      {fallen_idx} (dsready)
  rocks:       {rock_idx} (dsready)
  ground cover:{cover_idx} (shrubs, ferns, grass, leaves)
  buildings:   {house_idx} (peaked roofs, porches, fences)
  barrels:     {barrel_idx}
  TOTAL:       {total}
  scene:       {SCENE_OUT}
""")

timeline.stop()
app.close()
print("done")
