# v25 Postmortem — Submission 2026-05-08

**Score:** 23.03 (T1=21, T2=1, T3=1)
**vs prior:** -41.5 from v22–v24 baseline (~65)
**Regression:** T3 collapsed 34 → 1 (task timeout). T1 regressed 29 → 21 (high-tension day).

## What failed

**Root cause (A2 static callgraph audit + A1 image audit — CONFIRMED):**
The v25 submission included 7 "performance fixes" intended for SC Stage 4
(CR-1, CR-2, CR-3, HR-1, HR-2, MR-4) that modified T3-touching code paths:

| Change | Location | Impact |
|--------|----------|--------|
| CR-2: SC Stage 4 stiffness 85→200 N/m | Stage 4 insertion loop | 2.4× stiffer → force buildup faster |
| CR-3: `stage4_force_guard_enable=False` | Stage 4 insertion loop | Force guard disabled; can't back off |
| CR-1 + HR-1: adaptive feedforward (`_ff_scale`) | Stage 4 | Amplified force in wrong direction |
| HR-2: force-checked phase settle | Stage 2/3 settle step | Extra abort paths introduced |
| MR-4: Z-floor guard after Stage 3 SC direct descent | Stage 3→Stage 4 handoff | Incorrect Z clamp |

Combined effect: SC Stage 4 with CR-2 stiffness + CR-3 guard disabled hit the
force-penalty ceiling much faster, causing T3 to timeout (task duration > limit).

**Was Bug 122 (yaw correction) missing?** No — A1 confirmed `-1.7133` present in all
3 v25 image copies. T3 loss was from Stage 4 changes, not yaw regression.

**Was joint-space code gating the issue?** No — A2 static callgraph shows
`_lateral_move_joint_space` has exactly one call site gated by `self.joint_space_t2_sfp`.
It is unreachable from T1 and T3.

## What we learned

1. T3 Stage 4 SC is hypersensitive to stiffness changes. 85 N/m was chosen specifically
   because SC cable tension is ~7.7 N at stall — spring(85)=7.6 N ≈ equilibrium → slow
   creep. At 200 N/m the arm fights the cable aggressively and triggers force abort.

2. The "performance fixes" from the 2026-05-07 review session (CR-1/2/3, HR-1/2, MR-4)
   were never sim-validated before shipping. This violates guiding principle 2:
   "Every change is flag-gated, default-off, sim-validated."

3. Diagnostic infrastructure was insufficient: the submission only returned score.json
   and a summary TXT. Without policy stdout / ant_diagnostics.jsonl, diagnosing the
   timeout took forensic audit of the static call graph and image content.

## What v26 does

- **Strict revert** to v24 baseline commit `e52c930`.
  Drops Bug 125/126/127 AND CR-1/2/3, HR-1/2, MR-4 as one block.
- **Additive instrumentation**: zone/trial guard (defense-in-depth), C1 score-encoded
  failure channel, C5 hw_variance.jsonl, trial budget accountant.
- **ANT_ANTI_MARKERS** in submit.sh abort build if any v25 Stage 4 change survives the
  revert.

## Expected v26 outcome

Floor: T1≈29, T2=1, T3≈34 → 64 (same as v22/v23/v24 baseline).
Upside: No v25 regression markers → T3 should fully recover.

## Process improvement (for v27+)

- Every T3-touching change must run a full 3-trial sim before commit.
- T3 wall-clock gate in submit.sh (Workstream C, already implemented in v26's
  submit.sh) prevents re-submitting T3-timeout code.
- Postmortem template must be present and filled before next-slot submit.sh runs.
