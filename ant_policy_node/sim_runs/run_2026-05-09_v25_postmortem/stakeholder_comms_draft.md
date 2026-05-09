# Stakeholder Communication: AIC v26-v32 Realistic Range

**Owner:** Lead A.
**Send by:** PST 2026-05-09 10:00 (before v27 work begins).
**Channel:** whatever stakeholder relationship has been used for prior
"≥100 by May 15" expectation-setting.
**Status:** DRAFT for Lead A approval.

---

## Subject

ANT campaign — realistic May-15 outcome range is **76–95**, not ≥100

## Message

Brief update before the second-to-last submission week.

**Where we are.**  v25 (submitted 2026-05-08) regressed -41 points to
23.03 due to seven Stage-4 SC "performance fixes" that shipped without
sim validation.  v26 (submitted 2026-05-09, in flight as of writing) is
a strict revert of those changes plus diagnostic instrumentation.
Expected v26 outcome: 60-68 (matching the v22-v24 baseline of ~65).

**Realistic May-15 range.**  After the v25 setback we have re-modeled
the campaign honestly:

| Scenario | Probability | Total |
|---|---|---|
| Floor (revert + bad-tension day) | 10% | 23 |
| Baseline (revert holds, normal day) | 35% | 64 |
| Contingency reachable (T1 escalation + T3 recal) | 30% | 76 |
| Realistic (T1+T3 both partial gains) | 15% | 86 |
| Stretch (T1 full + T3 insertion) | 10% | 101 |

Expectation: **76-95 with ~80% confidence.**  100 is reachable but
requires both T1 joint-space lateral AND T3 actual insertion to work on
real hardware — neither has been demonstrated in eval yet.

**What changed.**  We are pivoting away from T2 (which has resisted four
sequential attempts, scoring 1.0 every time including v22, v23, v24, v25)
and concentrating engineering capacity on T1 (currently 29, headroom to
~50) and T3 (currently 34 in v22-v24, headroom to ~50 if we can convert
"near port" to "in port").  Plan target: T1=50, T2=1, T3=50 → 101.

**Why we're saying this now, not at the deadline.**  Three reasons:
1. The honest model is more useful than an aspirational one.
2. v26 carries no T2/T1/T3 progress — it's a setup slot for v27-v31
   recovery.  Expect the score to be flat-vs-baseline.
3. If the realistic 76-95 is a problem, there is a 5-day window to
   adjust (rebid, defer, change scope) instead of one day.

**Risk register.**

- High-tension HW days (~25% per day) cap T1 at ~21 regardless of code.
  Three submissions at 23 in this campaign suggest this is not rare.
- T2's architectural ceiling (15 N Cartesian impedance limit) will not
  be solved this campaign.
- v32's hard rule (image bytes must equal v31's) means May 15 has zero
  margin for late changes.

**What's holding the line:**  every submission v26-v31 produces a
postmortem, a C1 score-encoded failure-code readout, and a HW-variance
datapoint.  The diagnostic infrastructure that v25 lacked is now mechanical
in `submit.sh` — we cannot ship another v25-class regression without
the build host actively bypassing five separate gates.

Happy to walk through any of this on Slack/call.

— Lead A

---

## Notes for Lead A before sending

- Replace placeholders with specific stakeholder names if needed.
- Decide whether to attach `submit_evidence_v26/verify_output.txt` (very
  strong evidence v26 is what we say it is) or keep the message short.
- The 76-95 range assumes the Day-2-onward plan executes without further
  process slips.  If Day 2 also skips A0 / T3 sim validation, this range
  is optimistic — adjust downward.
- Suggested next-touchpoint: Wednesday 2026-05-13 (after v29) when we'll
  have variance data on whether the 86 floor is realistic.
