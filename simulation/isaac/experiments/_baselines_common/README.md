# Baselines vs Our T&R - consolidated results (15 routes)

## результат

table (with partial WP progress even for aborted runs)

Entries show **WPs reached / total WPs** before the run either completed
(green x, with duration) or was aborted by watchdog / early failure (red ✗,
with reason). For the OK rows, `sk=K` is the count of intermediately
SKIP-ped waypoints (e.g. those that ended inside obstacle inflation).

| route | our custom | exp 74 stock Nav2 | exp 76 RGB-D (no IMU) |
|---|---|---|---|
| 01_road          | **77/80** (96 %)  1357 s, sk=3  | ✗ stall 0.00 m   | ✗ stall 0.00 m |
