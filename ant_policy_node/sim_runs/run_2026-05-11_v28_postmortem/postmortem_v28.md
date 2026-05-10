# v28 Postmortem — Submission UTC 2026-05-11

**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v28`
**Code:** B1 branch `claude/b1-joint-space-t1-sfp` with `joint_space_t1_sfp=True`
**Purpose:** T1 joint-space escalation after Bug 106 retries exhaust

---

## Score

**Real-HW score:** TODO — fill when result is received
<!-- T1 Tier1= Tier2= Tier3= -->
<!-- T2 Tier1= Tier2= Tier3= -->
<!-- T3 Tier1= Tier2= Tier3= -->
<!-- Total= -->

---

## vs prior

- v27=64.21 (T1=29.16, T2=1.0, T3=34.06)
- **v28 expected:** T1 gain +10–20 pts from joint-space escalation; T2/T3 unchanged
- Expected range: 74–84

---

## Regression

TODO

---

## What happened

TODO — fill after result. Key questions:
1. Did `joint_space_t1_sfp=True` engage on T1 after Bug 106 retry exhaustion?
2. Did T1 close lateral gap and score > 29.16?
3. Did T2 and T3 hold at v27 baseline (T2=1.0, T3=34.06)?
4. What did `ant_diagnostics.jsonl` / C1 failure code show for T1 joint-space path?

---

## Decision for v29

Per `CAMPAIGN_PLAN_v27_v32.md`:
- B2: `sc_arrival_max_retries=4` (B2a validated; see `B2a_sc_retry_analysis.md`)
- B2b: `joint_space_sc_wp2` if B2b sim-validated by Eng-4 before v29 build
- v28 >= 84 -> combine B1 (as-shipped) + B2a + B2b; target >= 100
- v28 < 74 -> bisect B1 failure mode (IK rejected vs joint stall); pivot v29 to B2 only

TODO — fill decision after score arrives.

---

## Process improvement

TODO
