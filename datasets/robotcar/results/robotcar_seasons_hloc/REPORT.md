# RobotCar Seasons - hloc Ablation Study Results

Generated: 2026-02-15 20:42

## Summary Table

| Method | (0.25m,2d) | (0.5m,5d) | (5m,10d) | Med.t(m) | Med.r(d) | Day 0.5m/5d | Night 0.5m/5d | Note |
|--------|-----------|----------|---------|---------|---------|------------|--------------|------|
| SuperPoint+SuperGlue | 42.9% | 62.4% | 83.8% | 0.301 | 0.782 | 71.5% | 30.4% |  |
| SuperPoint+LightGlue | 42.4% | 62.0% | 84.1% | 0.301 | 0.787 | 71.2% | 29.4% |  |
| DISK+LightGlue | 42.5% | 62.4% | 85.0% | 0.300 | 0.786 | 71.5% | 30.6% |  |
| ALIKED+LightGlue | 42.9% | 62.3% | 86.0% | 0.297 | 0.783 | 71.2% | 30.9% |  |
| SuperPoint+SuperGlue+OpenIBL | 43.6% | 64.2% | 86.8% | 0.296 | 0.761 | 71.2% | 39.4% |  |
| SIFT+NN-ratio | 38.6% | 56.6% | 75.9% | 0.367 | 0.872 | 69.6% | 10.7% |  |
| ALIKED+LG+OpenIBL | 44.6% | 64.5% | 89.6% | 0.288 | 0.756 | 71.2% | 40.6% |  |

## Comparison with Published Results

Published SuperPoint+SuperGlue on RobotCar Seasons (Sarlin et al. 2020):
- Day: ~49/70/88% at (0.25m,2d)/(0.5m,5d)/(5m,10d)
- Night: ~17/27/40% at the same thresholds

Note: Our evaluation uses the training split only (~1900 images). Official numbers use the full test set via visuallocalization.net.

## Analysis

**Best overall:** ALIKED+LG+OpenIBL (64.5% at 0.5m/5d)

**Best daytime:** SuperPoint+SuperGlue (71.5% at 0.5m/5d)

**Best nighttime:** ALIKED+LG+OpenIBL (40.6% at 0.5m/5d)

**LightGlue vs SuperGlue:** 62.0% vs 62.4% -> LightGlue matches SuperGlue.

**DISK+LightGlue:** 62.4% vs 62.4% (SP+SG baseline)

**ALIKED+LightGlue:** 62.3% vs 62.4% (SP+SG baseline)

**SIFT+NN-ratio:** 56.6% vs 62.4% (SP+SG baseline)

**OpenIBL boost for ALIKED:** 64.5% vs 62.3% (NetVLAD)

  Night: 40.6% vs 30.9%


## Recommendations

1. Use the best-performing method as the primary result for the thesis.
2. Report per-condition breakdown to show robustness across weather.
3. Submit test results to visuallocalization.net for official numbers.
4. Night performance gap is a key challenge for future work.
5. OpenIBL retrieval consistently outperforms NetVLAD, especially at night.
