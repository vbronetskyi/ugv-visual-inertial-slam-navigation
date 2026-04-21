# Experiment Ranking Analysis (exps 52..62)

Offline synthesis accross 10 south-roundtrip runs with the same three
cone groups + tent. Each metric comes from logs already on disk; no
experiment was re-run.

## Headline result

**Exp 59 (WP look-ahead detour) tops all three weightings.**
Exp 60 (final-approach 10 cm spec) is a close second, particularly
strong on tent clearance. Exp 56 (projection-cap 1 m - the long-time
"looked best" run) drops to third because the new analysis caught the
×3.55 path ratio and 44-minute runtime that earlier TRACK/ARRIVAL
numbers hid.

## what the composite score exposed

The ranking that emerges from six sub-scores (safety, precision,
efficiency, completion, speed, smoothness) differs from every single
metric. Three concrete cases:

- **Exp 56 had TRACK p50 = 0.69 m, best of all.** Yet path_ratio 3.55
  and idle_ratio from pp stalls destroyed its efficiency and speed
  sub-scores. In plain English: it followed the teach path extremely
  tightly, but kept stalling and looping back on itself, taking
  ~44 minutes for a route teach ran in 6.5. Ranked 3rd.

- **Exp 62 reported "88 / 95 REACHED".** Arrival p95 was 21.9 m -
  the robot *believed* it was at the WP but was 22 m off in GT. Precision
  sub-score zero, composite 0.45, rank 5.

- **Exp 60 appeared mid-pack on drift but cleanly bypassed the tent
  (+151 cm clearance).** The exposure-weighted safety score captured
  this without penalising non-encounters - it ranks 2nd with the best
  production score under the safety-heavy weighting.

## Pareto dominance

Exps **59 and 60** are Pareto-optimal - no single experiment beats
them on all sub-scores. Every other run is dominated by one of the two
or by exp 56 (which dominates 58 on smoothness while both lose on
efficiency).

Exp 52 r4 is dominated on every sub-score by 52 r1. Exps 53 and 55
are both dominated by 59 and 60 on safety, efficiency and completion.

## Key trade-offs exposed

| Trade-off | Evidence |
|---|---|
| Tight tracking vs obstacle detour | exp 56 TRACK 0.69 m / clearance -35 cm   vs   exp 60 TRACK 2.59 m / tent +151 cm |
| Belief-REACHED vs physical arrival | exp 62: 88/95 REACHED but p95 arrival 21.9 m |
| Accumulator aggressiveness vs drift stability | exp 61 (2 s / 2 m accum) -> drift runaway to 21 m |
| Route sanitize vs stop-and-go | exp 58 had 268 wedge recoveries, idle_ratio ≈ 0.59 |

## Methodological caveats

1. **Single run per experiment.** No variance estimate; Isaac Sim is
   deterministic modulo physics-thread timing, but one run doesn't
   bound sensitivity to seed.
2. **Robot half-width = 0.35 m** as per the task spec. The real Husky
   is closer to 0.5 m; switching that constant inflates all clearance
   numbers by -15 cm and would change exp 60 tent status from "+151 cm"
   to "+136 cm" (still best) and exp 56 from "-10 cm" to "-25 cm".
3. **Teach time baseline = 391 s.** The teach trajectory CSV has no
   per-frame timestamp header, so we infer from path length / 1 m s⁻¹.
   Exact teach time might be 350..430 s; speed sub-score robust to
   this range.
4. Non-finishers get composite × 0.5 cap. Arbitrary but reflects
   the user requirement that a failed roundtrip cannot outrank a
   completed one on partial-credit metrics alone.

## Thesis-ready recommendation

For the thesis results chapter:

- Headline the composite ranking table as the main result.
- Highlight exp 59 and exp 60 as the two Pareto-optimal configurations.
- Use exp 62 as the cautionary example against measuring success by
  "REACHED count" alone.
- Use exp 56 as the cautionary example against using drift or
  single-point TRACK as the sole localisation quality metric.
- Sensitivity analysis (three weighting schemes agreeing on top 3)
  strengthens the robustness claim.

Future work (outside this analysis): combining exp 59's detour
geometry with exp 60's conservative clearance budget - their
sub-score profiles are complementary, so a hybrid could plausibly
beat both.
