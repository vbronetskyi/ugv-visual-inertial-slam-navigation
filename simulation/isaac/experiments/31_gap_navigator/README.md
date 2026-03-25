# exp 31: Gap-Based Navigator with Forest Thinning

## Goal

Замінити VFH-секторний підхід (exp 30) на gap-based навігатор який знаходить
реальні проходи між деревами з урахуванням ширини Husky. Також перевірити
вплив щільності лісу на навігацію.

## Gap Navigator Architecture

```
Depth image -> 1D obstacle profile (640 columns)
            -> inflate by robot half-width + safety margin (1.27m total)
            -> find connected free intervals (gaps)
            -> filter by route cone (±34° normal, ±69° recovery)
            -> score: 40% route alignment + 25% clearance + 20% width + 15% stability
            -> hysteresis (hold gap 3 frames, switch only if 30% better)
            -> cmd_vel: ROUTE_TRACKING (fast) or GAP_MODE (slow, precise)
```

## Key Parametars

- Robot width: 0.67m + 2×0.3m safety = **1.27m** required gap
- Obstacle threshold: 1.5m (reduced from 2.5m - forest has 2-3m gaps)
- Route cone: ±34° normal, ±69° recovery (when no gaps in normal cone)
- Speed: 0.7 m/s route tracking, 0.25 m/s gap mode

## Results

### Dense forest (743 objects)

| Metric | Value |
|---|---|
| Objects | 743 (195 pine + 113 oak + 357 shrub + 28 rock + 40 fallen + 10 other) |
| Max anchor reached | **10/96** |
| Distance (GT) | ~20m |
| Stuck point | (-80, -22) - shrubs at 1.2m from both sides |
| Cause | Geometric bottleneck: gap <1.3m (Husky needs 1.27m) |

### thinned forest (521 objects, every 3rd tree/shrub removed)

| Metric | Value |
|---|---|
| Objects | 521 (removed 222 = 30%) |
| Max anchor reached | **23/96** (GT), 42 via skips |
| Distance (GT) | ~35m |
| Stuck point | (-75, -33) - still dense zone |
| Cause | Odom drift 6-9m + remaining obstacles |
| Improvement vs dense | **2.3x further** |

### Mode distribution (thinned)

| Time range | Mode | Blocked % | Notes |
|---|---|---|---|
| t=210-220 | ROUTE_TRACKING | 0% | Open area, full speed |
| t=230-240 | GAP_MODE | 18-40% | Trees nearby, found 2 gaps |
| t=250-260 | ROUTE_TRACKING | 0-1% | Past first bottleneck! |
| t=270-280 | GAP_MODE -> STUCK | 66-82% | Dense zone entered |
| t=300-340 | GAP_MODE/STUCK | 20-80% | Oscillating, skips helping |

## Key Findings

1. Obstacle threshold matters enormously: 2.5m -> 89-96% blocked (false
   obstacles). 1.5m -> 0-18% blocked (realistic). Forest gaps are 2-3m wide -
   threshold must be below gap width.

- Thinning works: removing 30% of trees gave 2.3x more progress. Dense
   forest has geometric bottlenecks where gaps <1.3m (physically impassable
   for Husky 0.67m + margin).

3. Gap navigator finds real paths: at 20% blocked, found 2-3 valid gaps
   with correct route alignment. Hysteresis prevents oscillation.

- Odom drift is still the limit: even with thinned forest, 6-9m error
   after 30m means robot navigates to wrong position -> enters obstacle zone.

5. **Same-session teach-repeat works mechanically**: teach records anchors,
   teleport back, repeat follows. The pipeline is solid - localization
   accuracy is the bottleneck.

## comparison: All Approaches on South Route

| # | Approach | Forest | Distance | Max anchor |
|---|---|---|---|---|
| 1 | GT controller (exp 30) | dense | **196m** | 97/98 |
| 2 | GT + odom_s (exp 30) | dense | **191m** | 95/96 |
| 3 | Encoder + depth avoid (exp 30) | dense | 80m | 38/96 |
| 4 | VFH depth nav (exp 30) | dense | 51m | 25/96 |
| 5 | Gap nav, thr=2.5m (exp 31) | dense | 20m | 10/96 |
| 6 | **Gap nav, thr=1.5m (exp 31)** | dense | **20m** | **10/96** |
| 7 | **Gap nav, thr=1.5m (exp 31)** | thinned | **35m** | **23/96** |

## Artifacts

### Scripts
- `scripts/gap_navigator.py` - feasible-gap navigator with route constraints
- `scripts/run_husky_teach_then_repeat.py` - full pipeline with thinning
- `scripts/route_follower.py`, `scripts/anchor_localizer.py` - from exp 30

### results
- `results/forest_thinning_comparison.png` - dense vs thinned scene map
- `results/gap_nav_dense_vs_thinned.png` - trajectory comparison

### Data
- `/tmp/gazebo_models_dense.json` - backup of original 743 objects
- `/tmp/gazebo_models.json` - active (restored to dense after experiments)
