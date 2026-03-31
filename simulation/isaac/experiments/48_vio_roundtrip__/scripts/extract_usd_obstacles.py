from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
omni.usd.get_context().open_stage("/opt/husky_forest_scene.usd")
stage = omni.usd.get_context().get_stage()
app.update()
import json

obstacles = []

# Categories to check with their approximate collision radii
# These paths/patterns are what Isaac Sim uses
TYPES = {
    "Rocks": ("rock", 0.8),
    "RockCol": ("rock", 0.8),
    "Trees": ("tree", 0.7),
    "Pines": ("pine", 0.7),
    "Oaks": ("oak", 0.7),
    "ShrubCol": ("shrub", 0.4),
    "Shrubs": ("shrub", 0.4),
    "Roadside": ("roadside", 0.5),
    "Houses": ("house", 3.0),
}

# Walk stage
rock_count = 0
tree_count = 0
shrub_count = 0
for prim in stage.Traverse():
    path = str(prim.GetPath())
    name = prim.GetName()
    
    # Check if it's a top-level obstacle container child
    # e.g., /World/Rocks/rock_023 - depth 3
    parts = path.strip("/").split("/")
    if len(parts) != 3: continue
    
    parent = parts[1]  # Rocks, RockCol, etc
    
    obs_type = None
    radius = 0.5
    if "rock" in name.lower() or parent in ("Rocks", "RockCol"):
        obs_type, radius = "rock", 0.8; rock_count += 1
    elif "pine" in name.lower():
        obs_type, radius = "pine", 0.7; tree_count += 1
    elif "oak" in name.lower():
        obs_type, radius = "oak", 0.7; tree_count += 1
    elif "shrub" in name.lower() or parent in ("ShrubCol","Shrubs"):
        obs_type, radius = "shrub", 0.4; shrub_count += 1
    else:
        continue
    
    try:
        xform = UsdGeom.Xformable(prim)
        m = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        obstacles.append({"type": obs_type, "x": float(t[0]), "y": float(t[1]), "r": radius})
    except: pass

# Dedupe by (x, y, type) rounded to 0.1m
seen = set()
unique = []
for o in obstacles:
    key = (round(o["x"], 1), round(o["y"], 1), o["type"])
    if key not in seen:
        seen.add(key)
        unique.append(o)

print(f"Extracted: {len(obstacles)} -> {len(unique)} unique")
print(f"Rocks: {rock_count}, Trees: {tree_count}, Shrubs: {shrub_count}")

with open("/tmp/usd_obstacles.json", "w") as f:
    json.dump(unique, f)

# Show types
from collections import Counter
print(Counter(o["type"] for o in unique))
app.close()
