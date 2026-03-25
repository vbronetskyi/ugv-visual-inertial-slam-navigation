# exp 34: Traversability - South Forest Render/Collision Desync

## Problem

Render-only vegetation (leaves, grass, ferns, fallen trees) blocks depth camera
but has no collision - robot drives through physically but depth navigator sees 89% blocked.

## Approaches tested

### Scene rebuild (convert_gazebo_to_isaac.py fixes)
- Removed ferns, reduced cover density, debris instead of fallen trees
- Result: 1677 objects (was 2021), blocked reduced to 50-67%

### traversability filter
- Block-based analysis: high variance + low coverage = vegetation -> filter out
- Result: not effective - leaves/grass are dense (high coverage), filter doesn't match

### Cover deactivation (all /World/Cover/ prims)
- Result: **0% blocked** - depth sees only tree trunks
- But: robot stuck due to other issues (cone oscillation, odom drift)

### Cone oscillation bug fix
- Root cause: `score_gaps()` uses `self.mode` from previous frame
- STUCK(±69°)->GAP_MODE(±34°)->STUCK cycle every 2 frames
- Fix: always use recovery cone ±69°

### heading_err clamp fix  
- Root cause: odom_yaw drift >69° -> heading_err > camera FOV -> all gaps rejected
- Fix: use GT yaw for heading (proxy for compass on real robot)

## results

| Config | blk% | Anchor | Distance | Key issue |
|---|---|---|---|---|
| Dense forest, basic depth | 89% | 10 | 20m | vegetation blocks |
| Rebuilt (no ferns) | 50-67% | 9-13 | 27m | vegetation blocks |
| Traversability filter | = raw | 9-13 | 27m | filter ineffective |
| No cover, narrow cone | 0% | 13 | 27m | cone oscillation |
| No cover, wide cone | 0% | 4-5 | 10m | odom_yaw drift |
| **No cover + GT yaw** | **0-64%** | **19** | **39m** | gap oscillation in dense zone |

Best non-GT: **39m (anchor 19)** with GT yaw + no cover + gap nav.

## Conclusion

South forest has fundamental limitations for depth-based navigation in Isaac Sim:
1. Render/collision desync makes depth unreliable (false obstacles everywhere)
2. Dense vegetation (743 objects in 200×100m) leaves gaps <2m between objects
3. Encoder odom yaw drift degrades heading estimation within 30s

For the thesis: south forest demonstrates the system's limits. Road route with
real obstacles (cones, tent) is the proper test for depth-based teach-and-repeat.

## Artifacts

- `scripts/traversability_filter.py` - block-based vegetation filter
- `scripts/gap_navigator.py` - with wide cone + oscillation fix
- `scripts/run_husky_teach_then_repeat.py` - skip-teach, GT yaw, no cover
- `results/` - comparison plots (if generated)
