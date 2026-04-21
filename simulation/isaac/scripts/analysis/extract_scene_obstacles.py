#!/usr/bin/env python3
"""Extract ALL collision objects from the Isaac Sim scene USD directly.

Authoritative source: /opt/husky_forest_scene.usd. Avoids missing objects
that /tmp/gazebo_models.json lacks (e.g. /World/RoadsideTrees, which are
not in the Gazebo dump).

Writes /workspace/simulation/isaac/routes/_common/scene_obstacles.json as:
    [{"type": '...', "x": ..., "y": ..., "r": ...}, ...]
"""
import json
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics

SCENE_USD = "/opt/husky_forest_scene.usd"
OUT = Path("/workspace/simulation/isaac/routes/_common/scene_obstacles.json")

# radius per top-level collision root
ROOT_RADIUS = {
    "/World/ShrubCol": ("shrub", 0.4),
    "/World/TreeCollision": ("tree", 0.7),
    "/World/RockCol": ("rock", 0.8),
    "/World/RoadsideTrees": ("roadside_tree", 0.4),
    "/World/Buildings": ("house", 4.5),
    "/World/Props": ("barrel", 0.5),
}


def main():
    stage = Usd.Stage.Open(SCENE_USD)
    assert stage, f"cannot open {SCENE_USD}"
    found = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        # pick matching root
        root_match = None
        for root, (label, r) in ROOT_RADIUS.items():
            if path.startswith(root + "/"):
                root_match = (root, label, r)
                break
        if root_match is None:
            continue
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        found.append({
            "path": path,
            "type": root_match[1],
            "x": float(t[0]),
            "y": float(t[1]),
            "r": float(root_match[2]),
        })

    # Buildings have 2 colliders each (foundation + walls) at same XY - dedup by name
    dedup = {}
    for o in found:
        if o["type"] == "house":
            key = "/".join(o["path"].split("/")[:-1])  # /World/Buildings/house_NN
        else:
            key = o["path"]
        dedup[key] = o
    # RoadsideTrees have rt_* + rtcol_* at same XY too - keep only rtcol_*
    final = []
    seen_roadside = set()
    for o in dedup.values():
        if o["type"] == "roadside_tree":
            base = o["path"].split("/")[-1]
            if base.startswith("rt_"):
                continue  # mesh without collision? still - keep rtcol_* variant
        final.append({"type": o["type"], "x": o["x"], "y": o["y"], "r": o["r"]})

    # summary
    from collections import Counter
    c = Counter(o["type"] for o in final)
    print(f"extracted {len(final)} collision objects:")
    for k, v in c.most_common():
        print(f"  {k}: {v}")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(final, indent=1))
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
