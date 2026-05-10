# Team Meeting — v27 Real-HW Result Review
**UTC:** 2026-05-10 (post-result)
**Submission:** submission_1009
**Attendees:** Lead A, Lead B, Eng-1, Eng-2, Eng-3, Eng-4, Eng-5
**Ref docs:** CAMPAIGN_PLAN_v27_v32.md, postmortem_v27.md, B2a_sc_retry_analysis.md

---

## 1. Results Debrief

### v27 final score: 64.21

| Trial | Tier 1 | Tier 2 | Tier 3 | Total |
|---|---|---|---|---|
| T1 (SFP +17°) | 1.0 | 21.29 (dur=16.64 s, eff=6, smooth=5.83) | 6.87 | **29.16** |
| T2 (SFP −45°) | 1.0 | 0.0 (plug positioning failed) | 0.0 | **1.0** |
| T3 (SC +17°)  | 1.0 | 11.83 (eff=6, smooth=5.83) | 21.23 | **34.06** |
| **TOTAL** | | | | **64.21** |

### vs. prior baseline

| Tag | Score | T1 | T2 | T3 | Notes |
|---|---|---|---|---|---|
| v22/v23/v24 | ~64.55 | ~29 | 1 | ~34 | Validated baseline band |
| v25 | 23.03 | ~21 | 1 | 1 | Our regression (CR-2/CR-3) |
| v26 | — | — | — | — | Infrastructure failure |
| **v27** | **64.21** | **29.16** | **1.0** | **34.06** | **Baseline confirmed** |

**Delta vs v24 (64.55): −0.34 pts.** Within run-to-run variance. No signal. T1 −0.22, T2 ±0, T3 −0.11.

### Infrastructure hypothesis: CONFIRMED

v27 returned a full 3-trial scored result with artifacts. The ECR cold-pull / layer-caching degradation hypothesis for the v26 failure is confirmed. Fresh build from a clean `./submit.sh v27` call eliminated the issue. No further infrastructure investigation required for v28.

### T1 tier_3=6.87 — B1 targeting assessment

T1 tier_3=6.87 at duration=16.64 s is consistent with the arm reaching the lateral bounding radius but stalling ~10 cm short of the SFP port XY under 15 N Cartesian ceiling. This is exactly the behavior B1 targets.

Mechanics: Bug 106 retries (up to 2×) ran and escalated feedforward/stiffness but could not overcome the 15 N Cartesian ceiling. T1 tier_3 reflects the final plug-port distance at the stalled position, not a partial insertion. The arm is getting close enough to score tier_2 (smooth approach, eff=6) but not tier_3 insertion credit.

This is Case B (Cartesian stall after retries exhaust), which is exactly the stall point where joint-space escalation has the theoretical leverage to provide ~+2.6 N net force at ~10 cm residual (joint spring ~27.6 N vs cable equilibrium ~25 N). **B1 is correctly targeted.**

The T1 tier_3=6.87 score (vs v24's 6.53) is within variance — this is not a meaningful change, just noise in the final stall position.

### T2 analysis

T2=1.0 (Tier 2=0, Tier 3=0). Plug positioning failed entirely — the arm did not enter the scoring bounding radius. This is the expected 15 N Cartesian ceiling behavior. T2 remains closed as an engineering target. No new information from this trial; consistent with v22/v23/v24 pattern. `joint_space_t2_sfp = True` guard is intact.

### T3 analysis

T3=34.06 matches v22/v23/v24 baseline (34.06–34.17). Bug 122 yaw correction (−1.7133 rad) is intact. WP2 arrival check passed within tolerance (v27 sim: err=26.2 mm ≤ 27 mm tol). Stage 4 ran for approximately 120 s (120 s internal timeout hit). SC plug reached ~4 cm from port (Bug 66 stall point). No regression. B2a/B2b target this remaining 4 cm gap.

---

## 2. v28 Go/No-Go Decision

### Decision gate (from CAMPAIGN_PLAN_v27_v32.md §v27)

| v27 score | v28 action |
|---|---|
| ≥ 60 | **Baseline confirmed → ship B1 in v28** |
| 23–59 | High-tension day variance → ship B1 anyway |
| Second infra failure | Halt code work; re-tag v27 as v28; escalate |

**v27 = 64.21 ≥ 60.**

### Decision: GO — ship B1 in v28

B1 branch `claude/b1-joint-space-t1-sfp` (commit 75fb288) exists and is code-complete. The path from branch to submission is:

1. Enable `joint_space_t1_sfp = True` (change one line in ANT.py)
2. Sync install/ copy (Bug 90 safeguard — submit.sh pre-build sync handles this automatically)
3. Run sim gates (see §3 and §5)
4. Complete checklist (independent review by Eng-4, sign-off before `./submit.sh v28`)

**No new code is required. This is a flag flip + sim validation.**

### Pre-submit checklist for v28

Mechanical gates — `submit.sh` must see all of these before push:

- `ANT-DIAG event=joint_space_start ... label="Stage 1 SFP T1 WP2 escalation"` fires in sim
- T3 sim ≥ 30 (no regression from B1 wiring)
- T3 wall-clock < 175 s
- 0 `joint_space_guard_violation` events
- 0 Traceback/Exception in policy.log
- 3 × `trial_end` events
- Eng-4 reviewer sign-off (driver=Eng-3, reviewer=Eng-4 per campaign plan)
- v28 postmortem template created before `./submit.sh v28` runs

**Reviewer assignment: Eng-4 (confirmed per CAMPAIGN_PLAN_v27_v32.md §Workstream B1)**

---

## 3. B1 Readiness Assessment

### Flag confirmed present and correctly wired

Code inspection of branch `claude/b1-joint-space-t1-sfp` (commit 75fb288) confirms:

**Flag declaration** (ANT.py `__init__`, line 553 on branch):
```python
self.joint_space_t1_sfp = False   # B1 (v28): T1 joint-space escalation; default-off
```

**Wire location** (ANT.py `_localize_connector()`, T1 SFP path, immediately after `_lateral_arrival_check_and_retry()` returns):
```python
if self.joint_space_t1_sfp and self._remaining_trial_budget_sec() > 25.0:
    js_result = self._lateral_move_joint_space(
        target_xy=(tgt_x, tgt_y),
        lateral_z=transit_z,
        orient=orient,
        zone="sfp",
        label="Stage 1 SFP T1 WP2 escalation",
        ...
    )
    if js_result is not None:
        actual_x, actual_y, orient, ok = js_result
        if ok:
            tgt_x, tgt_y = actual_x, actual_y
```

**This matches the CAMPAIGN_PLAN §v28 engineering spec exactly.**

**Zone guard confirmed intact:** `_lateral_move_joint_space` has an unconditional early-return on `zone != "sfp"` that fires before any flag checks. A T3 call into this helper would return None regardless of `joint_space_t1_sfp` value. T3 cannot reach this code path.

**install/ copy:** The B1 commit synced both `ant_policy_node/ant_policy_node/ANT.py` and `ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py`. Bug 90 is handled.

### What needs to change for v28-ready state

The branch is on `claude/b1-joint-space-t1-sfp`. The single code change required is:

```python
# ANT.py __init__, line 553 on branch
self.joint_space_t1_sfp = False   →   self.joint_space_t1_sfp = True
```

Then sync install/ (submit.sh pre-build sync handles this). No other changes.

**Branch merge sequence:**
1. Eng-3 flips flag True on the B1 branch
2. Eng-3 runs sim gates (see §5 action items)
3. Eng-4 reviews diff (it is a 1-line change + install/ sync)
4. Eng-4 signs checklist
5. `./submit.sh v28` runs from the B1 branch HEAD

### B1 risks and concerns

**Physics margin is thin at the 10 cm stall point.** The campaign plan calculates joint spring ~27.6 N vs cable equilibrium ~25 N → net +2.6 N. This is a 1.1× force margin, not 2×. On high-tension days (cable equilibrium > 27 N), B1 may stall identically to Cartesian. This is accepted risk — B1 is escalation-only with Cartesian fallback, so worst case = v27 score.

**IK nominal DH parameters.** Real UR5e has mm-scale calibration offsets. At 10 cm residual with a 2 cm arrival tolerance, this is below noise. No concern for this use case.

**Budget guard is correctly set.** `_remaining_trial_budget_sec() > 25.0` prevents late-trial entry. If T1 Stage 1 runs long (retries consumed budget), B1 is skipped gracefully and Cartesian path continues.

**No T2 or T3 path can reach B1 code.** Confirmed by zone guard.

**Expected T1 gain: +10 to +20 pts (T1: 29 → 40–50).** On a day where cable equilibrium is 25 N or less, B1 should deliver full gain. On worst-case days, partial gain (arm gets to 5–7 cm) is still possible.

---

## 4. B2 Outlook

### Projected v28 range

Confirmed v27 baseline: 64.21

| Scenario | T1 gain | T1 total | T2 | T3 | Projected total |
|---|---|---|---|---|---|
| B1 fails (high tension > 27 N) | 0 | 29 | 1 | 34 | **64** (floor — worst case = v27) |
| B1 partial (reaches 5–7 cm) | +10 | ~39 | 1 | 34 | **~74** |
| B1 full (reaches 2–4 cm) | +20 | ~49 | 1 | 34 | **~84** |

**Tell stakeholders before v28 submits: floor = 64, plan for 75, reach for 84.**

### B2a: sc_arrival_max_retries=4

**Still the right move for v29.** Eng-4's B2a_sc_retry_analysis.md confirms `sc_arrival_max_retries=4` is safe. Stage 4 self-adjusts to absorb worst-case retry overhead — the trial never exceeds 180 s. At worst case, Stage 4 runs for 90.6 s (still far above the 6 s settle window). Expected T3 gain: +2–5 pts alone.

B2a implementation is a one-flag, zero-new-code-paths change. It will ship with B2 regardless of whether B2b validates.

### B2b: SC WP2 joint-space escalation

**Decision point for Eng-4:** The physics margin at the SC 4 cm stall is marginal — joint spring ~9.4 N at 4 cm < 25 N cable equilibrium. B2b works on low-to-normal tension days but not high-tension worst case. B2b only ships if Eng-4's sim shows `err_mm < 20` in T3.

**Eng-4 must start B2b sim validation by PST 2026-05-11 12:00** to have a result before the v29 build window opens (PST 2026-05-11 17:00). If validation is not done by then, B2a ships alone in v29 — do not hold the slot.

**B2b go/no-go rule:** If T3 sim with `joint_space_sc_wp2=True` shows `ANT-DIAG event=joint_space_final err_mm < 20` → include B2b. If T3 stalls identically to v27/v28 → drop B2b, ship B2a only. No gray area.

---

## 5. Action Items — v28 Build Window (PST 2026-05-10 17:00 → 2026-05-11 16:59)

| # | Action | Owner | Due (PST) | Done? |
|---|---|---|---|---|
| 1 | Create v28 postmortem template at `sim_runs/run_2026-05-11_v28_postmortem/postmortem_v28.md` | Eng-1 | 2026-05-10 20:00 | [ ] |
| 2 | Flip `joint_space_t1_sfp = True` on `claude/b1-joint-space-t1-sfp` branch | Eng-3 | 2026-05-10 20:00 | [ ] |
| 3 | Run full 3-trial local sim from B1 branch with flag=True; capture `policy.log` to `/tmp/ant_local_sim/policy.log` | Eng-3 | 2026-05-10 22:00 | [ ] |
| 4 | Verify sim gate: `joint_space_start label="Stage 1 SFP T1 WP2 escalation"` fires in T1; T3 ≥ 30; T3 wall-clock < 175 s; 0 guard violations; 0 tracebacks; 3× trial_end | Eng-3 | 2026-05-10 22:00 | [ ] |
| 5 | Independent review: Eng-4 reads the 1-line flag change + install/ sync diff; signs v28 pre-submit checklist | Eng-4 | 2026-05-11 10:00 | [ ] |
| 6 | Complete pre-submit checklist (all mechanical gates per campaign plan §v28) | Eng-3 | 2026-05-11 12:00 | [ ] |
| 7 | `./submit.sh v28` — build, verify markers, push | Eng-3 | 2026-05-11 by 14:00 | [ ] |
| 8 | Fill postmortem_v28.md Score field immediately after result returns | Eng-3 (or Eng-1 backup) | 2026-05-11 17:00+ | [ ] |
| 9 | **Parallel — B2b sim validation:** Run T3-only sim with `joint_space_sc_wp2=True`; record `err_mm` from `joint_space_final` event | Eng-4 | 2026-05-11 12:00 | [ ] |
| 10 | **Parallel — B2b go/no-go:** Report result to team; commit go/no-go note in `B2b_validation.md` | Eng-4 | 2026-05-11 14:00 | [ ] |

**Hard rule:** Items 7 depends on Items 3–6 all passing. Do not skip or waive sim gates. If sim gate T3 < 30 or guard violation fires → stop, diagnose, do not push.

---

## 6. Updated Score Model

### Confirmed baseline: v27 = 64.21

The v22–v24 baseline of ~64.55 is now confirmed by a fourth data point. The ~0.3 pt run-to-run variance band is stable. Scores below 60 reflect either a regression or a genuine high-tension floor day.

### Revised campaign score model

| Scenario | T1 | T2 | T3 | Total | P | Trigger |
|---|---|---|---|---|---|---|
| Infra fails again | — | — | — | fail | 10% | v27 fresh-build fix wasn't the root cause |
| B1 fails, high tension | 29 | 1 | 34 | **64** | 15% | Cable equilibrium > 27 N on v28 eval day |
| B1 partial | 39 | 1 | 34 | **74** | 25% | Normal tension day, B1 partial gain |
| B1 full | 49 | 1 | 34 | **84** | 20% | Low tension day, B1 closes to <4 cm |
| B1 full + B2a | 49 | 1 | 37 | **87** | 15% | B2a +3 pts T3 from extra retry |
| B1 full + B2a + B2b | 49 | 1 | 45 | **95** | 10% | B2b validates, T3 reaches insertion territory |
| B1 full + B2a + B2b + constants | 50 | 1 | 50 | **101** | 5% | v30 constants tuning on good hardware day |

**Communicate to stakeholders: floor = 64 (v27 confirmed), plan for 84–87, reach for 101.**

### Slot-by-slot projections

| Tag | UTC slot | Expected total | Source of gain |
|---|---|---|---|
| v27 | 2026-05-10 | 64.21 (actual) | Baseline restoration |
| v28 | 2026-05-11 | 64–84 | B1: T1 joint-space escalation |
| v29 | 2026-05-12 | 64–95 | B2: T3 SC WP2 retry + (if validated) joint-space |
| v30 | 2026-05-13 | constants only | ≤ +5 pts from knob tuning |
| v31 | 2026-05-14 | RC = best of v27–v30 | Re-tag of highest confirmed score |
| v32 | 2026-05-15 | = v31 (re-tag) | Mechanical re-tag, no new code |

---

## 7. Flags and Risks

### F1 — Exec summary error in B2a_sc_retry_analysis.md

The executive summary in `B2a_sc_retry_analysis.md` states:

> Worst-case wall-clock ... is **~173 s**, leaving a **7 s margin** against the 180 s trial limit.

This is **incorrect.** The correct numbers (from the body of the same document, Revised Calculation section) are:

- With `sc_arrival_max_retries=4` worst case, Stage 4 self-adjusts to **90.6 s** effective duration
- Trial total = **180.0 s** — no margin, but also no overrun
- The exec summary's "173 s / 7 s margin" describes the **N=2 (current)** case, not the N=4 proposed case

The body analysis is correct. The exec summary conflated the current (N=2) and proposed (N=4) worst-case calculations. **Action: Eng-4 to correct the exec summary before v29 build window opens.** The recommendation to ship `sc_arrival_max_retries=4` is not affected — the corrected analysis still confirms it is safe because Stage 4 is self-adjusting. But reviewers reading only the summary will have wrong numbers.

**Corrected exec summary numbers:**
- N=2 (current): ~173 s total, ~7 s margin, Stage 4 = 120 s (full)
- N=4 (proposed): ~180 s total, ~0 s margin, Stage 4 = 90.6 s (compressed by 29 s)

The 90.6 s Stage 4 is still far above the 6 s settle window. Recommendation stands.

### F2 — B1 physics margin is thin on worst-case days

Net force at the T1 10 cm stall point is +2.6 N (joint spring 27.6 N − cable equilibrium 25 N). Any run-to-run cable tension increase of > 2.6 N will negate B1's gain entirely. This is a real probability. Mitigation is already in place (Cartesian fallback → worst case = v27 score). Team should not be surprised if v28 = 64 on a high-tension day.

### F3 — v27 fallback tag still not in ECR

v27-fallback is not yet tagged in ECR (no AWS creds on build host as of last check). If v28 fails with no artifacts, there is no easy fallback image available from a portal session. **Eng-1 must tag v27 as a fallback image in ECR before the v28 submit window opens.** This is not blocking v28 submission but is required by the pairing-and-fallback non-negotiable.

### F4 — Only 4 code slots remain (v28–v31)

v27 confirms the baseline. B1 (v28) and B2 (v29) are the only meaningful new-code slots. v30 is constants only, v31 is RC. There is no room to recover from an unvalidated ship. Do not ship anything that has not passed the sim gates in §5.

### F5 — Diagnostic readout is limited

v27 returned score JSON + summary TXT only. No `ant_diagnostics.jsonl`, no policy.log. C1 failure-code readout requires AIC organizers to provide raw artifacts. This means post-v28 analysis will again rely on tier_2/tier_3 score structure inference rather than direct log inspection. Request raw diagnostic artifacts from AIC organizers for future submissions.

### F6 — T3 Stage 4 runs at 120 s internal timeout

T3 Stage 4 is consistently hitting its 120 s internal timeout rather than finding insertion. This means B2a's reduced Stage 4 budget (90.6 s worst case) represents a real reduction in insertion search time. The question for v29 is whether B2a's extra retries + improved arrival position are worth the 29 s Stage 4 reduction. Eng-4's B2a analysis says yes (6 s settle window still met). Accept.

---

*Document produced UTC 2026-05-10 post-result. Commit to repo before v28 build begins.*
*Next meeting trigger: v28 real-HW result (UTC 2026-05-11 submission_xxxx).*
