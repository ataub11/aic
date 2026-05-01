# Ground Truth: run_2026-04-30b

**Date/time**: 2026-04-30 15:45 start, 18:36 end  
**Build**: local-61646a0  
**Total score**: 85.59 / ~130 possible  

## Trial Results

| Trial | Config | T1 | T2 | T3 | Notes |
|-------|--------|----|----|-----|-------|
| T1 | SFP +17° | 1 | 21.96 | 38.41 | **Success**; 8.02s descent (good duration); partial insertion 4cm |
| T2 | SFP −45° | 1 | 9.22 | 25 | **Success** (arrived within tolerance on first attempt, 21.8mm err); 131s duration (no time bonus); 25pt T3 for final dist 4cm |
| T3 | SC +17° | 1 | −12 | 0 | **Failed**; lateral arrival miss 69mm (>25mm tol), used all retries, still 69mm off; **−12 force penalty** (>20N for 4.34s, max 119N); final dist 19cm |

## Key Observations

- **T3 SC lateral arrival**: Stage 1 retry logic failed — 69mm error persisted through both retries (ff_y=9.0N, stiff bumped to 400→450 N/m). Robot missed the SC port approach entirely.
- **T3 force penalty**: After arrival failure, robot descended anyway and hit the board hard (119N peak, 4.34s over threshold → −12pt penalty).
- **T2 duration**: 131s is well over the threshold; no duration bonus. Likely slow search or excessive compliance behavior.
- **T1 partial insertion**: 4cm insertion yields only 38.41 T3 points (partial, not full). Full insertion would give 60+ pts.

## Parameters (from ant_diagnostics.jsonl)

```
lateral_feedforward_n: 6.0
t2_sfp_steps: 7
t3_sc_step_m: 0.025
lateral_stiff_n_per_m: 350.0
anchor_bias_m: 0.02
arrival_tolerance_m: 25mm
arrival_max_retries: 2
stage4_compliance: false
yaw_alignment: true
vision_sc: true
high_tension_threshold_n: 19.0
```

All 3 baselines measured >19N (high tension), so baseline subtraction was active every trial.
