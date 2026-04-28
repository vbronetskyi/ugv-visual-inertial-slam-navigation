# route categorization

15 routes were configured for the thesis campaign. presenting them in a single
flat table makes for a long, hard-to-read row stack. they fall naturally into
three groups by *what they were used for* and *what they test*. this file
defines the grouping and gives the per-group facts.

each route is a closed round trip: the robot drives out from the spawn, hits a
turnaround, and drives back. lengths in the tables below are total round-trip
length from the planner waypoints in `slam_routes.json`.

## three groups

### A - development testbeds (2 routes)

these are the two routes the iterative process in [main_experiments](main_experiments/)
ran against. matcher parameters, projection cap, sanity gate, finisher etc.
were all tuned here.

| route | length | environment | obstacles | notes |
|---|---|---|---|---|
| 01_road | 333 m | sinusoidal road, sparse trees on the sides | 17 cones in 3 walls + 1 tent | the easiest visual corridor. used in exp 02, 03, 27, 35, 39, 41, 70, 73 |
| 03_south | 398 m | deep forest, dense shrubs, feature-poor | 9 cones in 3 walls + 1 tent | the hardest visual corridor. used in exp 30, 49, 55-64 |

together these two bracket the difficulty range - open road on one end,
ambiguous forest on the other. anything that worked on both was considered
robust enough to run on the campaign routes.

### B - long forest crossings (5 routes)

corner-to-corner routes thorugh the full scene, +-380-400 m round trip. the
matcher was *not* tuned against any of these; they exist to show that what
worked on 03_south generalises.

| route | length | start → mid → end (m) | trees+shrubs near corridor |
|---|---|---|---|
| 02_north_forest | 405 m | (-91,-6) → (71,3) → (-91,-6) | 13 + 20 = 33 |
| 04_nw_se | 385 m | (-90,35) → (65,-37) → (-90,35) | 9 + 21 = 30 |
| 05_ne_sw | 407 m | (65,35) → (-90,-38) → (65,35) | 10 + 17 = 27 |
| 06_nw_ne | 394 m | (-90,35) → (66,37) → (-90,35) | 7 + 22 = 29 |
| 07_se_sw | 401 m | (65,-35) → (-90,-38) → (65,-35) | 12 + 16 = 28 |

note the diagonal nature of 04/05/07 - the robot crosses the entire scene from
one corner to its opposite. 06 stays along the north edge, 02 follows the
road corridor thorugh the north forest. obstacle setup is mostly props
(barrels / cardboxes / dumpster), 04_nw_se also has cone walls + tent.

### C - short cross-cuts (8 routes)

shorter routes (+-150-200 m round trip) used for robustness sweeps and
ablations. each runs faster than the long campaigns so a single Husky can
work through several in one afternoon.

#### C.1 corner cuts (2 routes)

short forest corners, no road, mostly clearing.

| route | length | start → mid → end (m) | terrain |
|---|---|---|---|
| 08_nw_sw | 164 m | (-90,35) → (-89,-37) → (-90,35) | west edge, sparse forest, 2 trees + 10 shrubs in corridor |
| 09_se_ne | 161 m | (65,-35) → (63,35) → (65,-35) | east edge, open clearing, 0 trees + 0 shrubs in corridor |

09_se_ne is the open-clearing baseline - the matcher has very little to lock
on to, but the path is also short and props are the only obstacles. exp 73
and exp 75 both ran on this route.

#### C.2 midline cross-cuts (6 routes)

these traverse the middle of the scene, where the road crosses with houses
and varied terrain. each +-165-200 m, all start from a different edge and end
at a midpoint or another edge.

| route | length | start → mid → end (m) |
|---|---|---|
| 10_nmid_smid | 182 m | (-20,30) → (23,-32) → (-20,30) |
| 11_nw_mid | 190 m | (-90,35) → (-24,-12) → (-90,35) |
| 12_ne_mid | 201 m | (65,35) → (-20,-3) → (65,35) |
| 13_cross_nws | 166 m | (-30,20) → (27,-16) → (-30,20) |
| 14_se_mid | 175 m | (65,-35) → (-1,17) → (65,-35) |
| 15_wmid_smid | 202 m | (-62,8) → (26,-32) → (-62,8) |

obstacle setup: 8-9 props per route, no cone walls, no tent.

## why this split

three reasons drove the categorization:

1. **purpose, not geometry.** A is what we tuned on, B is "what we tested
   for generalization, C is what we ran a lot to make confidence
   intervals tight". a length-based split would mix dev routes with
   generalization routes (both sit in the 320-400 m bucket) and dilute the
   story.
2. **table size.** a 15-row route facts table is unreadable in a thesis;
   three separate tables of 2 / 5 / 8 rows each fit on one page and make the
   purpose of each route group obvious from the surrounding text.
3. **obstacle composition correlates with the group.** A and the diagonals
   in B use cone walls + tent (the original campaign obstacles); the
   generalization routes use prop clusters (barrels, dumpsters). that is
   itself a story to tell - same matcher, different obstacle types.

## quick reference for chapter 5 prose

- when discussing iteration, parameter tuning, or matcher development:
  cite **A** (01_road, 03_south).
- when discussing generalization and the campaign result:
  cite **B** (02, 04, 05, 06, 07).
- when discussing obstacle bypass robustness, ablations, or per-route
  metrics with confidence intervals: cite **C** (08-15).

an alternate two-axis split is also handy when the prose calls for it:

| axis | values | who falls where |
|---|---|---|
| length | short (<=200 m) / long (>300 m) | C is short, A and B are long |
| terrain | road / forest / clearing | 01 is road, 02 + 03 + B-diagonals are forest, 09 is clearing, the rest are mixed |
