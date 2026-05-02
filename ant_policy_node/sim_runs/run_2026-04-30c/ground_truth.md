# Ground Truth: run_2026-04-30c

**Date/time**: 2026-04-30 21:36 start, May 1 00:50 end  
**Build**: local-19e8a81 (Bug 99 SC port XY + yaw calibration applied)  
**Total score**: 132.87 — **significant improvement from 85.59 (run_2026-04-30b)**

## Trial Results

| Trial | Config | T1 | T2 | T3 | Notes |
|-------|--------|----|----|-----|-------|
| T1 | SFP +17° | 1 | 21.95 | 38.41 | Success; 8.04s descent; 4cm partial insertion |
| T2 | SFP −45° | 1 | 11.32 | 25 | Success; 131s (no duration bonus); 4cm final dist |
| T3 | SC +17° | 1 | 10.88 | 22.30 | **Near-miss: 3cm from port** (down from 19cm); 134s |

## Key Observations

- **T3 SC XY calibration fix working**: final plug-port distance 0.03m (was 0.19m in run_2026-04-30b). The arm reached the correct port XY after the position calibration update.
- **T3 yaw correction active** (build_version=local-19e8a81 confirms Bug 99 changes ran). Port distance 3cm = plug approaching but not inserting. Yaw correction of -97.93° is in effect.
- **T3 tier_3=22.30**: partial credit scored (plug within ~3cm). This is the first time T3 has earned meaningful tier_3 points.
- **T2 efficiency=6**: path length 0.00m shown (likely a scoring artifact) but got full efficiency bonus.
- **No force penalties**: all three trials clean.
- **T3 still no insertion** at 3cm: the remaining gap is likely the final approach depth (Stage 4 stall) or a residual yaw misalignment.

## Parameters (from ant_diagnostics.jsonl)

```
build_version: local-19e8a81
lateral_feedforward_n: 6.0
t2_sfp_steps: 7
t3_sc_step_m: 0.025
lateral_stiff_n_per_m: 350.0
anchor_bias_m: 0.02
arrival_tolerance_m: 25mm
arrival_max_retries: 2
stage4_compliance: false
yaw_alignment: true  ← Bug 99 active
vision_sc: true
high_tension_threshold_n: 19.0
```
