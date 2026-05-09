# A2 — Static Call-Graph Audit of v25 Joint-Space Surface

**Date:** 2026-05-09 morning
**Owner:** Eng-2 (reviewer, Workstream A)
**Input:** `git diff e52c930..96984fd -- ant_policy_node/ant_policy_node/ANT.py`
**Status:** COMPLETE — diagnosis revised.

## Headline finding

**Joint-space code is correctly gated to T2 SFP.** The H1 hypothesis from
the v25 post-mortem (Bug 126/127 latency leaking into T3 via shared
helpers) is **falsified** by static analysis of ANT.py.

The actual T3 regression cause is most likely **v25's Stage 4 SC changes**
(CR-1, CR-2, CR-3, HR-1, HR-2, MR-4) — 251 lines of T3-touching
modifications to Stage 4 stiffness, adaptive feedforward, force guard,
Z-floor guard, and force-checked phase settle.

## Joint-space surface map

| Symbol | Defined at | Called from | Gating |
|---|---|---|---|
| `_lateral_move_joint_space` | ANT.py:1108 | ANT.py:2103 (single call site) | `if self.joint_space_t2_sfp:` |
| `_solve_ik_for_tcp` | ANT.py:974 | ANT.py:1108 (only inside joint-space helper) | inherited |
| `_apply_diag_signature` | ANT.py:1022 | ANT.py:1108 (only inside joint-space helper) | inherited |
| `wrist_wrench` Bug 127 read | ANT.py:1149 | inside `_lateral_move_joint_space` | inherited (T2 only) |
| `wrist_wrench` Bug 127 settle read | ANT.py:1308 | inside `_lateral_move_joint_space` | inherited (T2 only) |
| `JointSpaceDiagnosticAbort` raise | ANT.py:1106, 1412 | inside diag helpers | catches at 3633 in `insert_cable` |

`_lateral_move_joint_space` has **exactly one call site** (line 2103)
inside the T2 SFP code path, gated by `self.joint_space_t2_sfp`. It is
not reachable from T1 or T3.

## What v25 actually changed (251 lines of ANT.py)

From `git show 96984fd --stat` and the commit message:

| Change | Affects | Risk |
|---|---|---|
| **CR-3** `stage4_force_guard_enable: True → False` | T1 + T3 Stage 4 | HIGH — removes ff_z decay safety |
| **CR-2** SC Stage 4 stiffness 85 → 200 N/m | **T3** | HIGH — overrides Stage 4 compliance |
| **CR-1 + HR-1** baseline-adaptive feedforward with cap (`_ff_scale`) | T1 + T3 | MEDIUM |
| **MR-4** Z-floor guard after Stage 3 SC direct descent | **T3** | MEDIUM |
| **HR-2** force-checked phase settle with `_settle_abort` flag | T1 + T3 | MEDIUM |
| **Bug 125** rescaled diag signatures (Z-axis, relative-to-current) | T2 only | low |
| **Bug 126** multi-seed IK + delta-rad widened 0.6 → 1.0 | T2 only | low |
| **Bug 127** F-cable overdrive + two-stage joint move | T2 only | low |
| **MR-1** dead Bug 100/101 supporting params removed | none (cleanup) | none |

The **first five rows** are T1/T3-touching. Any of them could explain
the v25 T1 regression (29 → 21) and T3 regression (34 → 1). The most
plausible single-bullet cause for T3 timeout is **CR-2 Stage 4 SC
stiffness 85 → 200 N/m**: a 2.4× stiffness increase against a high-
tension cable equilibrium will produce force buildup faster, and with
**CR-3 force guard disabled** the policy can no longer back off.

## Implications for v26 plan

1. **The strict revert is still correct** — but for different reasons
   than originally diagnosed. The plan called for reverting *all* of v25,
   which the A2 finding confirms is right. We're reverting CR-1/2/3,
   HR-1, HR-2, MR-4, Bug 125/126/127 as a single block.

2. **Joint-space zone/trial guards become defense-in-depth, not the
   primary fix.** Still valuable to add — they prevent a future
   CLAUDE.md drift where someone calls `_lateral_move_joint_space` from
   T1/T3.

3. **Diagnostic channel C1 is the critical addition** — without it we
   would have spent another submission slot guessing what failed in v26.
   C1 lets us read the failure mode out of `score.json` directly.

4. **The original H1 hypothesis was wrong.** This validates the external
   review's Gap-1 fix (A0 local repro before A3). Had we shipped a v26
   that only narrowed joint-space gating without reverting Stage 4 SC
   changes, we would have shipped the same bug under a different label.

## Procedure for A0 / A1 (build-host engineer execution)

`aws` CLI is not available in the dev environment. Hand off to
build-host engineer:

```bash
# A1 — image audit
docker pull 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v25
docker run --rm --entrypoint /bin/bash \
  973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v25 \
  -c 'find / -name ANT.py -not -path "*/proc/*" 2>/dev/null'
# For each ANT.py path, grep for:
#   -1.7133                       (Bug 122 yaw)
#   gripper_yaw_correction_rad    (Bug 99 table)
#   stage4_force_guard_enable     (CR-3)
#   insertion_stiffness           (CR-2)
#   _ff_scale                     (CR-1+HR-1)
#   enable_cable_overdrive        (Bug 127)
#   enable_multi_seed_ik          (Bug 126)
#   diag_signatures_relative_to_current_pose  (Bug 125)
# Output to v25_image_audit.md.

# A0 — local repro
docker pull <same as A1>
cd /home/user/aic
docker compose -f docker/docker-compose.yaml up -d aic_eval  # adapt to local sim wrapper
# Run a 3-trial sim and capture policy.log + score.json.
# Single discriminator: does T3 trial duration exceed 110 s (timeout)?
#   YES → revert plan (A3) is correct as drafted.
#   NO  → regression is HW-only or build-pipeline; halt A3 and re-submit
#         ant:v24 digest as v26 calibration shot.
```

## Sign-off

A2 complete. Hand-off to A3 (v26 patch authoring) — proceeding under the
assumption that A0 will confirm the diagnosis. If A0 falsifies, A3 is
discarded and the team falls back to the v24-digest re-submission path.
