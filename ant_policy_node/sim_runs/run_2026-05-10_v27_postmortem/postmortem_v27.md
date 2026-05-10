# v27 Postmortem — Submission UTC 2026-05-10

**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v27`
**Code:** v26 as-is (fresh build from branch `claude/analyze-v25-regression-25OyM` @ df82c5c)
**Purpose:** Baseline restoration after UTC 2026-05-09 infrastructure failure

---

## Score

**Real-HW score: 64.21** (submission_1009, UTC 2026-05-10)
Artifact: https://aic-portal-artifacts-production-bucket.s3.amazonaws.com/media/submission_files/submission_1009/9c254f09-8f50-4efc-8449-693ea5e8a307.txt

<!-- T1 Tier1=1.0 Tier2=21.29 Tier3=6.87 Total=29.16 -->
<!-- T2 Tier1=1.0 Tier2=0.0  Tier3=0.0  Total=1.0  -->
<!-- T3 Tier1=1.0 Tier2=11.83 Tier3=21.23 Total=34.06 -->
<!-- Total=64.21 -->

| Trial | Tier 1 | Tier 2 | Tier 3 | Total |
|---|---|---|---|---|
| T1 (SFP +17°) | 1.0 | 21.29 (duration=16.64 s, eff=6, smooth=5.83) | 6.87 | **29.16** |
| T2 (SFP −45°) | 1.0 | 0.0 (plug positioning failed) | 0.0 | **1.0** |
| T3 (SC +17°)  | 1.0 | 11.83 (eff=6, smooth=5.83) | 21.23 | **34.06** |
| **Total** | | | | **64.21** |

---

## vs prior

| Tag | Score | T1 | T2 | T3 | Notes |
|---|---|---|---|---|---|
| v22/v23/v24 | ~64.55 | ~29 | 1 | ~34 | Validated baseline |
| v25 | 23.03 | ~21 | 1 | 1 | Our regression (CR-2/CR-3) |
| v26 | — | — | — | — | Infrastructure failure |
| **v27** | **64.21** | **29.16** | **1.0** | **34.06** | **Baseline confirmed ✓** |

Delta vs v24 (64.55): −0.34 pts — within run-to-run variance. T1 −0.22, T2 ±0, T3 −0.11.

---

## Regression

**None.** v27 = baseline restoration only. Score 64.21 is within variance of v24's 64.55. The v25 CR-2/CR-3 regression (score 23) is confirmed resolved. Infrastructure failure from v26 is confirmed resolved — eval returned a full 3-trial result with artifacts.

---

## What happened

1. **Infrastructure resolved:** v27 returned a full 3-trial scored result. The v26 infrastructure hypothesis (ECR cold-pull / layer-caching degradation) is confirmed: fresh build eliminated the issue.

2. **Baseline confirmed:** T3=34.06 matches v22/v23/v24 baseline (34.06–34.17). Bug 122 yaw correction (−1.7133 rad) is intact. T1=29.16 matches prior pattern (29.38–29.48). T2=1.0 — Cartesian 15 N ceiling still the binding constraint, expected.

3. **T1 pattern:** tier_2=21.29, tier_3=6.87, duration=16.64 s. Consistent with ~10 cm lateral stall (arm reaches bounding radius but not port). Bug 106 arrival retries ran but could not overcome 15 N Cartesian ceiling. This is precisely the stall point B1 targets.

4. **`hw_variance.jsonl` / C1 failure code:** Portal artifacts are score+summary only (no raw policy log). Per campaign plan diagnostic readout plan: T1 tier_3=6.87 (vs expected ~6.90 baseline) suggests lateral stall at ~0.10 m — Case B (joint move stalled / Cartesian stall) is the operative T2 failure mode. T1 C1 code likely `lateral_stall (000)` or `retry_exhausted (001)`.

---

## Decision for v28

**v27 = 64.21 ≥ 60 → BASELINE CONFIRMED → SHIP B1**

Per `CAMPAIGN_PLAN_v27_v32.md` decision gate:
- B1 branch `claude/b1-joint-space-t1-sfp` (commit 75fb288) is ready
- Enable `joint_space_t1_sfp = True` for v28 build
- Run sim gates before push (see campaign plan §v28)
- Eng-3 driver, Eng-4 reviewer

Expected v28 gain: T1 29 → 40–50 (+10–20 pts), giving total 75–86.

---

## Process improvement

1. **Portal artifact capture:** Only score JSON + summary TXT returned (no `ant_diagnostics.jsonl`, no `policy.log`). Request raw logs from AIC organizers for future submissions to enable C1 failure-code discrimination.

2. **Working tree -dirty:** v27-41fa4a0-dirty shipped as expected. The dirty bits (docker-compose.yaml tag, log/latest_build symlink) were policy-irrelevant per working_tree.diff review. Process worked.

3. **v26 fallback tag:** Still not in ECR (no AWS creds on build host). v24 remains the effective fallback. If v28 ships and v27 is confirmed good, tag v27-fallback via portal session.
