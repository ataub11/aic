# v27 Postmortem — Submission UTC 2026-05-10

**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v27`
**Code:** v26 as-is (fresh build from branch `claude/analyze-v25-regression-25OyM` @ df82c5c)
**Purpose:** Baseline restoration after UTC 2026-05-09 infrastructure failure

---

## Score

**Real-HW score:** TODO — fill when result is received
<!-- T1 Tier1= Tier2= Tier3= -->
<!-- T2 Tier1= Tier2= Tier3= -->
<!-- T3 Tier1= Tier2= Tier3= -->
<!-- Total= -->

---

## vs prior

- v22/v23/v24 baseline: ~64 (T1≈29, T2=1, T3≈34)
- v25 regression: 23
- v26: infrastructure failure (no score)
- **v27 expected:** floor 23 (bad-tension day), baseline 64, KPI = "did T3 recover to ~34?"

---

## Regression

TODO

---

## What happened

TODO — fill after result. Key questions:
1. Did the eval infrastructure issue resolve? (v27 returning any score = yes)
2. Did T3 hold at ~34 (baseline confirmed)?
3. What did the `hw_variance.jsonl` / C1 failure code show?

---

## Decision for v28

Per `CAMPAIGN_PLAN_v27_v32.md`:
- v27 ≥ 60 → ship B1 (Eng-3 `joint_space_t1_sfp` implementation)
- v27 23–59 → ship B1 anyway (escalation-only, safe regardless of tension day)
- v27 infra failure again → halt code work; re-tag v27 as v28; escalate to leads

TODO — fill decision after score arrives.

---

## Process improvement

TODO
