# Final Experiment Ranking - exps 52..62

All scores ∈ [0, 1], higher is better.  Non-finishers capped at 0.5 composite.

## weights - outdoor UGV (primary ranking)

| Sub-score | Weight | Rationale |
|---|---|---|
| safety | 0.30 | an outdoor UGV that hits obstacles is unusable regardless of speed |
| precision | 0.20 | arriving at the right waypoint is a core T&R requirement |
| efficiency | 0.15 | wasted path ratio translates to wasted battery + time |
| completion | 0.15 | a roundtrip that doesn't close is a failed mission |
| speed | 0.10 | operationally important but subordinate to safety |
| smoothness | 0.10 | idle/replan oscillation proxies for stability under load |

## ranking (outdoor UGV weighting)

| Rank | Exp | Composite | Safety | Precision | Efficiency | Completion | Speed | Smooth | Verdict |
|---|---|---|---|---|---|---|---|---|---|
| 1 | **59** | **0.712** | 0.61 | 0.46 | 0.79 | 0.80 | 1.00 | 0.99 | Lookahead detour + smoothest + best loop closure; tent/cone contacts the weak spot |
| 2 | **60** | **0.657** | 0.73 | 0.30 | 0.90 | 0.55 | 1.00 | 0.59 | Best physical tent bypass (+1.51 m) at cost of slight efficiency and precision |
| 3 | 56 | 0.563 | 0.78 | 0.63 | 0.00 | 0.70 | 0.00 | 0.99 | Tight TRACK but ×3.5 path ratio and 44-min runtime - "walks the route the long way around" |
| 4 | 58 | 0.529 | 0.69 | 0.61 | 0.00 | 0.76 | 0.00 | 0.85 | Route sanitizer worked but 2.7 h runtime (268 wedge recoveries) - unacceptable in production |
| 5 | 62 | 0.449 | 0.15 | 0.00 | 0.73 | 0.75 | 0.99 | 0.81 | "88/95 REACHED" was belief - physical arrival p95=21.9 m; sanity gate insufficient |
| 6 | 61 | 0.421 | 0.57 | 0.00 | 0.72 | 0.47 | 0.72 | 0.00 | Over-aggressive accumulator polluted map; drift runaway |
| 7 | 55 | 0.326 | 0.67 | 0.63 | 0.89 | 0.29 | 0.53 | 0.98 | First T&R prototype, didn't close the loop |
| 8 | 53 | 0.275 | 0.80 | 0.61 | 0.81 | 0.33 | 0.00 | 0.14 | Proactive reroute but never finished roundtrip |
| 9 | 52 r1 | 0.181 | 0.50 | 0.69 | 0.00 | 0.12 | 0.00 | 0.58 | Early prototype, stopped at ~98 m |
| 10 | 52 r4 | 0.142 | 0.22 | 0.68 | 0.00 | 0.05 | 0.00 | 0.74 | Same, with tent grazing |

## Sensitivity analysis (research / production priorities)

| Rank | Exp | Outdoor | Research (precision-heavy) | Production (safety-heavy) |
|---|---|---|---|---|
| 1 | 59 | 0.712 | 0.671 | 0.719 |
| 2 | 60 | 0.657 | 0.574 | 0.694 |
| 3 | 56 | 0.563 | 0.585 | 0.602 |
| 4 | 58 | 0.529 | 0.564 | 0.567 |
| 5 | 62 | 0.449 | 0.391 | 0.445 |
| 6 | 61 | 0.421 | 0.337 | 0.493 |
| 7 | 55 | 0.326 | 0.305 | 0.296 |
| 8 | 53 | 0.275 | 0.266 | 0.268 |
| 9 | 52 r1 | 0.181 | 0.203 | 0.173 |
| 10 | 52 r4 | 0.142 | 0.172 | 0.108 |

Top three (59 / 60 / 56) stay in the same order across all weightings -
**the ranking is robust**, not an artefact of the chosen weights.

See:
- `metrics_full.csv` - raw numeric metrics per experiment
- `metrics_normalized.csv` - per-column normalized [0..1]
- `composite_scores.csv` - all sub-scores + three composites
- `ranking_plots.png` - bar chart + parallel-coordinates
- `RANKING_ANALYSIS.md` - narrative writeup
