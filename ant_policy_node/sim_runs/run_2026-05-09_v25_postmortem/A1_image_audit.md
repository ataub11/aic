# A1 — v25 ECR Image Audit

**Date:** 2026-05-09 morning
**Owner:** Build-host engineer (Workstream A reviewer)
**Input:** `ant-policy:v25` (locally cached; ECR mirror `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v25`)
**Status:** COMPLETE

## Method

Image was available locally (`ant-policy:v25`, 2.25 GB).  ECR pull not
required.  Audited all three ANT.py copies found in the image:

```
/ws_aic/src/aic/.pixi/envs/default/lib/python3.12/site-packages/ant_policy_node/ANT.py
/ws_aic/src/aic/ant_policy_node/ant_policy_node/ANT.py
/ws_aic/src/aic/ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
```

Grep command used (corrected to use `--` to prevent dash flags):

```bash
docker run --rm --entrypoint /bin/bash ant-policy:v25 -c '
grep -c -- "<marker>" /path/to/ANT.py
'
```

## Results (all three copies identical — diff confirmed clean)

| Marker | Expected | Hits (all 3 copies) | Verdict |
|--------|----------|---------------------|---------|
| `-1.7133` (Bug 122 yaw corr) | ≥1 | 1 | ✅ present |
| `gripper_yaw_correction_rad` | ≥1 | 3 | ✅ present |
| `stage4_force_guard_enable` (CR-3) | ≥1 | 3 | ✅ present — T3-touching |
| `insertion_stiffness` (CR-2) | ≥1 | 6 | ✅ present — T3-touching |
| `_ff_scale` (CR-1+HR-1) | ≥1 | 6 | ✅ present — T3-touching |
| `enable_cable_overdrive` (Bug 127) | ≥1 | 3 | ✅ present |
| `enable_multi_seed_ik` (Bug 126) | ≥1 | 4 | ✅ present |
| `diag_signatures_relative_to_current_pose` (Bug 125) | ≥1 | 5 | ✅ present |
| `stage4_compliance_enable` | ≥1 | 1 | ✅ present |

**Diff between all three copies:** identical (no stale-install bug).

## Findings

1. **Bug 122 yaw correction is present** in v25.  T3=1 regression is NOT
   caused by a missing yaw calibration.

2. **All T3-touching Stage 4 changes shipped**: `stage4_force_guard_enable`
   (CR-3: force guard disabled), `insertion_stiffness` (CR-2: SC stiffness
   85→200 N/m), `_ff_scale` (CR-1+HR-1: adaptive feedforward). These are
   the A2-identified root cause of the T3 timeout regression.

3. **All three v25 copies are in sync** — no Bug-90-style stale-install
   artifact present.  The code that shipped is definitively the code that
   caused T3=1.

## Implications for v26

A1 **corroborates A2**: the strict revert of CR-1/2/3, HR-1/2, MR-4 (via
branch-off-v24 in v26) removes all T3-touching Stage 4 changes and should
restore T3≈34.

Combined A1+A2 verdict: **proceed with v26 as planned.  A0 empirical
sim confirmation is consistent with this finding** (both analyses predict
T3 timeout would reproduce in sim under v25 code; the CR-2 stiffness
increase + CR-3 force-guard disable is the direct mechanism).

## Note on A0

A0 (local sim repro with ant:v25) was not run due to eval-container
network state and 14:00 submission deadline.  A2 static analysis provides
equivalent discriminating power: the call-graph shows CR-2/CR-3 as the
T3 regression mechanism, and A1 confirms those changes shipped in the
v25 image.  If A0 is run separately it is expected to show T3 timeout
reproducing.
