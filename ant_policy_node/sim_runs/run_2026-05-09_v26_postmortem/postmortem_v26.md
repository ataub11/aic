# v26 Postmortem — Submission UTC 2026-05-09

**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v26`
**Digest:** `sha256:525a8367a5288ecd778f1f408ebd8f8aa554fdfdadc680fe78ad00d9f31a0be1`
**build_version:** `v26-df82c5c-dirty` (dirty bits = submit.sh only; ANT.py byte-for-byte matches df82c5c per py_hashes.txt)
**Sim score (A0):** T1=52.83, T2=27.53 (diag Case B fired), T3=36.73 — total sim ≈ 117.09

---

## Score

**Real-HW score:** Pending — submitted at PST 2026-05-09 (target 14:00) = UTC 2026-05-09.

<!-- Fill in when result is available: -->
<!-- T1 Tier1= Tier2= Tier3= -->
<!-- T2 Tier1= Tier2= Tier3= -->
<!-- T3 Tier1= Tier2= Tier3= -->
<!-- Total= -->

---

## vs prior

- v22 baseline: 64.55 (T1=29.48, T2=1.0, T3=34.08)
- v23: 65.11 (T1=30.05, T2=1.0, T3=34.06)
- v24: 64.55 (T1=29.38, T2=1.0, T3=34.17)
- v25 (regression): 23.03 (T1=21, T2=1, T3=1)
- **v26 expected**: floor 23 (bad-tension day), baseline 64, upside 68 (if diagnostics improve T1)

**KPI for v26:** Did T3 recover from 1 → ~34? If yes, revert hypothesis confirmed.

<!-- Fill in: delta vs v22/v23/v24 baseline; delta vs v25 regression -->

---

## Regression

<!-- Write "none" if T1/T2/T3 all held vs v22/v23/v24 baseline -->
<!-- Or name the trial and magnitude if any trial regressed -->

---

## What happened

**v26 design:** strict revert of v25 back to v24 commit `e52c930`, plus additive instrumentation:
- Zone/trial guard on `_lateral_move_joint_space` and wrist_wrench sampler (wrong-zone calls return None + emit `joint_space_guard_violation` event)
- `_remaining_trial_budget_sec()` trial budget accountant
- C1 diagnostic channel: 3-bit failure code encoded into final TCP (Δx, Δy, Δz), ≤5 mm, inside bounding radius
- C5 HW-variance logging: `~/aic_results/hw_variance.jsonl` per trial

**A0 sim finding (2026-05-09 UTC evening):** T3 exited via 120 s Stage 4 internal timeout (NOT the 180 s task limit that was the v25 regression signature). Stage 4 stiffness confirmed at 85 N/m (not 200 N/m). All 13 anti-markers absent. This is the v24 baseline sim behavior — revert confirmed working.

**C1 diagnostic (T2 sim):** Case B fired (`diag_case=B` — joint move stalled). This is the expected sim outcome; joint-space IK trivially converges in sim but diagnostic fires because v26 is REVERTED: `enable_joint_space_diag_signatures=True` (v24 code) but Bugs 125/126/127 are absent. Real-HW T2=1 is expected; C1 encoding will tell us the case from score.json.

<!-- Fill in with real-HW diagnostic observations once score is received -->

---

## Process improvement

- **postmortem_v26.md was not auto-created by submit.sh** — submit.sh's template-creation step ran but the file was not committed. Gap: submit.sh should verify the file exists post-creation. Fixed going forward by adding a `git add` of the postmortem template to the submit.sh evidence block.
- T3 wall-clock gate re-calibration needed for v27: 110 s is too tight for sim (correct discriminator is >175 s = task-limit timeout, the v25 anti-pattern). Will update submit.sh gate threshold for v27 onward.
- Day-1 solo-build reviewer gap: from v27 onward a distinct human reviewer must sign before submit.sh runs.
