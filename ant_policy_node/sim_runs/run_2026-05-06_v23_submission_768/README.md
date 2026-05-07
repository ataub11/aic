# v23 — submission_768 (2026-05-06)

**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v23`
**Branch built from:** `claude/t2-joint-space-ik-v23`
**Total score:** **65.11** (vs v22=64.55, +0.56 = within noise)

## Per-trial breakdown

| Trial | tier_1 | tier_2 | tier_3 | Total | vs v22 |
|-------|--------|--------|--------|-------|--------|
| T1 (SFP +17°) | 1 | 22.15 | 6.90 | 30.05 | +0.57 |
| T2 (SFP −45°) | 1 | 0.00 | 0.00 | 1.00 | 0.00 |
| T3 (SC +17°)  | 1 | 11.80 | 21.26 | 34.06 | −0.02 |

## Headline finding

**Bug 123 (joint-space T2 lateral via UR5e DLS-Jacobian IK) did not close the
T2 gap on real hardware.** T2 scored 1.0 — identical to v18 and v22. The
joint-space path's design rationale was that `JointMotionUpdate` bypasses
the 15 N `maximum_wrench` Cartesian impedance ceiling and reaches ~47 N
joint-spring force at TCP for a 17 cm position error, which should
overpower the assumed 25 N cable equilibrium. T2 stalling at the same
distance as v22 means at least one of these assumptions held differently
on the real arm.

## What we cannot conclude from these artifacts

Submission_768 published only:

1. `image_uri.json` (this directory)
2. `score.json`
3. A human-readable summary TXT (no policy log content)

Bug 108's `~/aic_results/ant_diagnostics.jsonl` and the raw
`python_*.log` from the eval container are **not** in the public bundle.
Without them we cannot tell which of the following actually happened:

- **(A) IK rejected, fell back to Cartesian.** `_solve_ik_for_tcp` may
  have failed on real-HW kinematics that diverge from the nominal DH
  parameters (default_kinematics.yaml has mm-scale offsets), or
  `joint_space_max_total_delta_rad=0.6` may have rejected a viable
  solution, or the `joint_space_stage_timeout_sec=20.0` may have fired.
  In this case the v22 Cartesian path then runs and stalls identically.
- **(B) Joint move stalled.** If real-HW cable equilibrium is ~30–40 N
  rather than the ~25 N assumed in the v22 cost-benefit analysis, the
  ~47 N joint-spring force does not provide enough margin.
- **(C) Cartesian re-engagement caused a stall before plug reached the
  scoring radius.** The re-engagement step publishes a Cartesian pose at
  the arm's current TCP with `target_stiffness=approach_stiffness=85 N/m`,
  but if the controller had stale state from before the joint move the
  resulting Cartesian resumption could have re-introduced the 15 N stall.

## Next actions

1. Request `~/aic_results/ant_diagnostics.jsonl` from the AIC organisers
   for submission_768 (Bug 108 emits `event=joint_space_attempt`,
   `event=joint_space_arrival`, `event=joint_space_fallback`). One pass
   over the JSONL discriminates (A), (B), and (C).
2. If diagnostics are unavailable, request raw policy stdout
   (`python_*.log`) — same `ANT-DIAG` lines appear there as a fallback.
3. Pending diagnosis: do **not** ship a v24 with broader joint-space
   coverage (e.g., extending it to T3 SC WP2). The current data is
   consistent with the joint-space path being inert on real HW.

## Preserved gains (T1, T3)

- T1 went 21.23 (v18) → 29.48 (v22) → 30.05 (v23). Bug 106 (lateral
  arrival check + retry) is the likely contributor; v23 preserves it.
- T3 went 1.0 (v18) → 34.08 (v22) → 34.06 (v23). Bug 99/122 calibrated
  port-yaw alignment achieved a 4 cm final plug-port distance for the
  first time on real HW in v22; v23 preserves it.
- The +0.56 difference between v22 and v23 is consistent with sim/HW
  variance and should not be attributed to Bug 123.
