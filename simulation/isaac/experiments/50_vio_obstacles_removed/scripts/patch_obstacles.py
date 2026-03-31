#!/usr/bin/env python3
"""Patch /workspace/simulation/isaac/scripts/spawn_obstacles.py with exp 50 positions.

Places 3 cone groups + 1 tent ON the south outbound route (not off-route).
Robot will encounter them going out, Nav2 replans bypass. Supervisor removes
them at turnaround so return path is clear.

Run BEFORE starting Isaac Sim.
"""
import re

SRC = "/workspace/simulation/isaac/scripts/spawn_obstacles.py"
with open(SRC) as f:
    code = f.read()

# New south obstacles - strategically placed on outbound route for Nav2 to bypass
new_south = '''    "south": {
        "cones": [
            # Group 1: cone wall in forest @ x=-75 ON route (route y=-24.6 here)
            # 3 cones, blocks y=-24..-26
            [(-75, -24.0), (-75, -25.0), (-75, -26.0)],
            # Group 2: cone wall in transition @ x=-18, route at y=-24
            # Reduced to 4 cones (pre-last group), blocks y=-23..-26
            [(-18, -23.0), (-18, -24.0), (-18, -25.0), (-18, -26.0)],
            # Group 3: AFTER first building (-5,-12), route at y=-17.8 @ x=5
            # 5 cones (last group, don't touch), blocks y=-16..-20
            [(5, -16.0), (5, -17.0), (5, -18.0), (5, -19.0), (5, -20.0)],
        ],
        # Tent: moved WEST to (-55, -37) - 8.3m from trees (most open spot on route)
        # Route at x=-55 is y=-36.8, tent blocks passage, easy to bypass
        "tent": (-55, -37.0),
    },'''

# Replace south section
pattern = r'    "south": \{\s*\n(.*?\n    \},\n)'
new_code = re.sub(pattern, new_south + '\n', code, count=1, flags=re.DOTALL)

with open(SRC, 'w') as f:
    f.write(new_code)
print("Patched south obstacles in spawn_obstacles.py")

# Verify
from importlib import reload
import sys
if 'spawn_obstacles' in sys.modules:
    reload(sys.modules['spawn_obstacles'])
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
import spawn_obstacles
print(f"\nSouth obstacles:")
obs = spawn_obstacles.OBSTACLES['south']
for i, group in enumerate(obs['cones']):
    xs = [g[0] for g in group]; ys = [g[1] for g in group]
    print(f"  Group {i+1}: {len(group)} cones @ x={xs[0]}, y={min(ys)}..{max(ys)}")
print(f"  Tent: {obs['tent']}")
