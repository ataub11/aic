# Team Meeting — UTC 2026-05-10, Pre-Submission
**Attendees:** Lead A, Lead B, Eng-1, Eng-2, Eng-3, Eng-4, Eng-5 (+ Claude co-pilot)
**Purpose:** Code review of B1, B2a analysis review, status update, go/no-go for v27

---

## 1. Status — overnight actions scorecard

| # | Action | Status | Evidence |
|---|---|---|---|
| 1 | Fill `postmortem_v26.md` Score field | ✅ Done | commit `ba9650b` |
| 2 | Startup timing test + `A0_startup_timing.md` | ✅ Done | `A0_startup_timing.md`, 12.6 s |
| 3 | v27 local sim + `/tmp/ant_local_sim/policy.log` | ✅ Done | policy.log, 4979 lines |
| 4 | submit.sh T3 gate 110 s → 175 s | ✅ Done | commit `ba9650b` (Eng-5) |
| 5 | Eng-2 independent reviewer assigned + signed | ✅ Done | commit `28d5080` (Eng-2) |
| 6 | B1 feature branch implemented | ✅ Done | branch `claude/b1-joint-space-t1-sfp`, commit `75fb288` |
| 7 | B2a parameter analysis | ✅ Done | `B2a_sc_retry_analysis.md`, commit `0d494c1` |
| 8 | Both leads review and sign plan | ⏳ Pending | — |

7 of 8 complete. Item 8 (lead sign-off) is the only outstanding overnight action.

---

## 2. v27 go/no-go

All v27 acceptance gates pass (Eng-2 verification, commit `28d5080`):

| Gate | Required | Observed | |
|---|---|---|---|
| trial_end count | == 3 | 3 | ✅ |
| T1 sim score | ≥ 50 | 52.83 | ✅ |
| T3 sim score | ≥ 30 | 36.73 | ✅ |
| T3 wall-clock | < 175 s | 137.43 s | ✅ |
| guard_violation | == 0 | 0 | ✅ |
| Traceback/Exception | == 0 | 0 (shutdown only) | ✅ |
| build_version | != unknown | v26-df82c5c-dirty | ✅ |
| Startup (Active state) | ≤ 60 s | 12.6 s | ✅ |
| Eng-2 sign-off | required | signed UTC 2026-05-10 | ✅ |

**Decision: GO. `./submit.sh v27` clears to run. Target ≤ 14:00 PST.**

---

## 3. Code review — B1 (`claude/b1-joint-space-t1-sfp`)

**Diff summary:** 22 lines added across 2 files (source + install/ sync, Bug 90).

### __init__ flag (line 551)
```python
self.joint_space_t1_sfp = False  # B1 (v28): T1 joint-space escalation; default-off
```
Placed immediately after `self.joint_space_t2_sfp = True`. Correct grouping. ✅

### T1 WP2 escalation block (lines 2272–2293)
Wiring is correct per campaign plan spec:

1. **Position in call chain:** Immediately after `_lateral_arrival_check_and_retry()` returns
   at line 2259–2271. B1 fires only when Cartesian retries have already exhausted — escalation-only ✅
2. **Guards:** `self.joint_space_t1_sfp and self._remaining_trial_budget_sec() > 25.0` ✅
3. **Zone:** `zone="sfp"` — passes the existing `_lateral_move_joint_space` guard (which
   returns None for `zone != "sfp"`). T3 cannot reach this block via guard alone ✅
4. **Label:** `"Stage 1 SFP T1 WP2 escalation"` — matches the sim gate in the campaign plan ✅
5. **Success path:** `tgt_x, tgt_y = actual_x, actual_y` updates Stage 1's return value so
   Stage 2 approaches the actual joint-space landed position ✅
6. **Failure fallthrough:** If `js_result is None` or `ok is False`, `tgt_x, tgt_y` unchanged
   → Stage 1 returns the commanded target → Stage 2 proceeds identically to v27 ✅
7. **Orient handling:** `orient` updated from B1 result but then overwritten by `orient_final`
   from `obs_final` immediately after — actual current orientation always used ✅
8. **install/ sync:** Both copies identical (Bug 90) ✅
9. **T2 and T3 untouched:** Block is inside the `else` branch that handles T1 SFP fast path
   only. No T2 or T3 code affected ✅

### Observation (not blocking)
Line 2259 discards the actual XY after Bug 106 retries: `_, _, orient = self._lateral_arrival_check_and_retry(...)`.
This is a **pre-existing** behavior — Stage 2 always approaches the commanded target even if
Bug 106 retries fell short. B1 partially corrects this on success (updates `tgt_x, tgt_y` to
actual). Logged as a future cleanup item; does not affect B1 correctness.

### Verdict: **PASS — B1 ready for v28 enable after v27 real-HW result**

---

## 4. Analysis review — B2a (`B2a_sc_retry_analysis.md`)

**Summary:** Eng-4 traced T3 stage timing from `/tmp/ant_local_sim/policy.log` and
`ANT.py` parameters. Conclusion: `sc_arrival_max_retries = 4` is safe.

### Core finding — Stage 4 is self-adjusting
Stage 4 exits on `min(120 s internal timeout, remaining trial budget)`. With 4 worst-case
18 s retries, pre-Stage-4 overhead = 89.4 s → Stage 4 effective = 90.6 s → trial = 180.0 s.
The trial never exceeds 180 s; it just gets a shorter Stage 4.

### Timing table (worst case = all retries consume full 18 s)

| N retries | Pre-Stage-4 | Stage 4 effective | Trial total | Stage 4 vs 6 s settle |
|---|---|---|---|---|
| 2 (current) | 53.4 s | 120.0 s | 173.4 s | 114 s above ✅ |
| 3 | 71.4 s | 108.6 s | 180.0 s | 102.6 s above ✅ |
| **4 (proposed)** | **89.4 s** | **90.6 s** | **180.0 s** | **84.6 s above ✅** |

Even at worst case, Stage 4 runs 90.6 s — far above the 6 s spiral settle window.

### Flag: executive summary has incorrect numbers
The exec summary states "worst-case ~173 s, 7 s margin" — these are the **2-retry (current)**
numbers, not the 4-retry proposed numbers. The correct 4-retry worst case is **~180 s
(0 s margin, Stage 4 self-adjusts to 90.6 s)**. The conclusion is still correct; the
mechanism description in the summary is wrong.

**Action for Eng-4:** Correct the executive summary in `B2a_sc_retry_analysis.md`
before B2b analysis begins. Change "~173 s / 7 s margin" → "~180 s (Stage 4
self-adjusts to 90.6 s effective — trial never exceeds limit)".

### Verdict: **B2a approved. Ship `sc_arrival_max_retries = 4` in v29.**

---

## 5. Issues and flags

| # | Issue | Severity | Owner | Action |
|---|---|---|---|---|
| F1 | B2a exec summary has wrong worst-case numbers | Low | Eng-4 | Fix before B2b analysis |
| F2 | B1 `joint_space_t1_sfp` not in `ANT_MARKERS` | Medium | Eng-5 | Add `joint_space_t1_sfp` (or diag label) to `ANT_MARKERS` before `submit.sh v28` runs |
| F3 | Campaign plan item 8 (lead sign-off) pending | Low | Lead A + Lead B | Sign today |
| F4 | B2b sim validation not yet run | Info | Eng-4 | After v27 ships, run T3-only sim with `joint_space_sc_wp2=True`; gate: `err_mm < 20` |

---

## 6. Decisions

| Decision | Rationale |
|---|---|
| **v27 GO — submit today** | All 9 acceptance gates pass, Eng-2 signed |
| **B1 branch holds — no merge until v27 HW result** | Campaign plan non-negotiable: B1 and B2 never in same slot |
| **B2a approved — `sc_arrival_max_retries = 4` for v29** | Stage 4 self-adjusts; 90.6 s effective Stage 4 is sufficient |
| **B2b contingent — run sim after v27** | Not validated; do not commit B2b before Eng-4 sim gate |
| **submit.sh v28 ANT_MARKERS update required** | Flag F2 must be resolved before `./submit.sh v28` |

---

## 7. Sequence for today

1. `./submit.sh v27` — Eng-1 drives, Eng-2 observes, target ≤ 14:00 PST
2. Portal click → record submission timestamp
3. Both leads sign `CAMPAIGN_PLAN_v27_v32.md` (item 8)
4. After v27 result arrives → fill `postmortem_v27.md` Score/Regression/What happened/Decision
5. v27 ≥ 60 OR 23–59 → merge B1 branch, set `joint_space_t1_sfp = True`, run v28 sim gates

---

## 8. v28 readiness checklist (to complete after v27 score)

- [ ] v27 real-HW score received and postmortem filled
- [ ] B1 branch merged to main (`claude/analyze-v25-regression-25OyM`)
- [ ] `joint_space_t1_sfp` set `True` in `ANT.__init__`
- [ ] `joint_space_t1_sfp` or diag label added to `ANT_MARKERS` in `submit.sh` (F2)
- [ ] v28 sim gates: `joint_space_start ... label="Stage 1 SFP T1 WP2 escalation"` fires, T3 ≥ 30, T3 wall-clock < 175 s, 0 guard violations
- [ ] Eng-4 (reviewer) signs v28 checklist
