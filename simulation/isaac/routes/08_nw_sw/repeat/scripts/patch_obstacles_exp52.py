#!/usr/bin/env python3
import re

SRC = "/workspace/simulation/isaac/scripts/spawn_obstacles.py"
with open(SRC) as f:
    code = f.read()

# Exp 52 south obstacles - reuse exp 50's validated ON-route cone walls
# that sit perpendicular to the robot path, each forcing a real replan.
new_south = '''    "south": {
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
    },'''

# Replace south section
pattern = r'    "south": \{\s*\n(.*?\n    \},\n)'
new_code = re.sub(pattern, new_south + '\n', code, count=1, flags=re.DOTALL)

with open(SRC, 'w') as f:
    f.write(new_code)
print("Patched south obstacles in spawn_obstacles.py (exp 52)")

# Verify
import sys
sys.path.insert(0, '/workspace/simulation/isaac/scripts')
if 'spawn_obstacles' in sys.modules:
    del sys.modules['spawn_obstacles']
import spawn_obstacles
obs = spawn_obstacles.OBSTACLES['south']
print('\nSouth obstacles loaded:')
for i, grp in enumerate(obs['cones']):
    print(f'  group {i}: {len(grp)} cones at {grp}')
print(f"  tent: {obs['tent']} (off-route)")
