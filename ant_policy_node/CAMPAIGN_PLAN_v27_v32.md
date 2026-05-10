# Campaign Plan — v27 through v32
**Issued:** UTC 2026-05-09  
**Authority:** Lead A + Lead B (joint sign-off)  
**Supersedes:** CLAUDE.md §"9-slot competition schedule" (deprecated)  
**Living reference:** CLAUDE.md §"ACTIVE CAMPAIGN PLAN — v26–v32" (updated inline)

Read this file first each session. It is the team's single source of truth for
what to build, who owns it, and what gates must pass before each push.

---

## Situation summary

| UTC slot | Tag | Outcome |
|---|---|---|
| 2026-05-08 | v25 | 23.03 — **our regression** (CR-2/CR-3 Stage 4 changes, unvalidated) |
| 2026-05-09 | v26 + v24-fallback | **Both failed — eval infrastructure degraded** |
| 2026-05-10 | v27 | **Next. Must restore baseline.** |

Six slots remain (v27–v32). Current validated real-HW ceiling: ~64 pts
(T1≈29, T2=1, T3≈34). Target: ≥86 primary, ≥101 stretch. Gap: +22 pts.

**T2 is closed as an engineering target for this campaign.**
Four slots produced zero T2 improvement. The 15 N Cartesian ceiling is real.
All remaining engineering effort goes to T1 and T3.
`joint_space_t2_sfp = True` stays (guard in place; no new T2 code ships).

---

## Score model (honest — communicate to stakeholders before v28 ships)

| Scenario | T1 | T2 | T3 | Total | P |
|---|---|---|---|---|---|
| Infra still broken | — | — | — | fail | 10% |
| Bad tension, no new leverage | 21 | 1 | 1 | 23 | 10% |
| Baseline holds, no new leverage | 29 | 1 | 34 | 64 | 20% |
| B1 partial (T1 +10) | 40 | 1 | 34 | 75 | 20% |
| B1 partial + B2 partial | 40 | 1 | 45 | 86 | 20% |
| B1 full + B2 partial | 50 | 1 | 40 | 91 | 10% |
| B1 full + B2 full | 50 | 1 | 50 | 101 | 10% |

**Tell stakeholders: floor 64, plan for 86, reach for 101.**

---

## Technical leverage

### B1 — T1 joint-space lateral (escalation-only)

**Root cause of T1 stalling at ~10 cm on real HW:**
Cartesian impedance `max_wrench = 15 N` is hit before arm reaches port XY.
Bug 106 retries (up to 2×) with escalating feedforward/stiffness recover some
ground but cannot exceed the 15 N ceiling.

**Why joint-space works as escalation at the 10 cm stall point:**
- TCP residual ≈ 10 cm → angular error ≈ 0.12 rad
- Joint spring (200 Nm/rad): ~23.5 Nm → ~27.6 N at TCP via Jacobian
- Cable equilibrium (bad day): ~25 N → net **+2.6 N** — barely positive
- At 5 cm: ~13.9 N < 25 N — cannot overcome alone

Joint-space fires **only after Bug 106 retries exhaust** (escalation-only, never
primary). Gets arm from ~10 cm stall → ~4–6 cm from port → partial insertion
territory. Fallback to Cartesian if joint-space fails (worst case = v27 score).

**Expected T1 gain: 29 → 40–50 (+10 to +20 pts)**

**Implementation scope:** ~60 lines, new flag `joint_space_t1_sfp = False`
(default-off). One call site after Bug 106 retry block in T1 WP2 path.
Reuses existing `_lateral_move_joint_space` + `ur5e_kinematics.py`.

---

### B2 — T3 lateral improvement

**Current T3 ceiling:** WP2 stalls ~4 cm from SC port XY (Bug 66). Bug 122
yaw correction (−1.7133 rad) is working — T3 scores ~34 with dist=0.04 m.
Closing the 4 cm WP2 gap pushes T3 toward 40–50.

**B2a — SC WP2 retry increase (always ships with B2):**
`lateral_arrival_max_retries` currently = 2 for all zones. New SC-specific
parameter `sc_arrival_max_retries = 4`. Two additional escalation retries,
each ≤ 18 s, with existing feedforward/stiffness escalation logic.
One flag, zero new code paths. Expected gain: modest (+2–5 pts alone).

**B2b — SC WP2 joint-space escalation (ships if v28 sim validates):**
Same architecture as B1 applied to SC WP2. At the ~4 cm stall point:
joint spring ≈ 9.4 N < 25 N on high-tension days — marginal. Works on
low-to-normal tension days. Flag-gated `joint_space_sc_wp2 = False`.
Guard update: one line in `_lateral_move_joint_space` to allow `zone == "sc"`
when flag is True. B2b ships only if Eng-4 sim shows `err_mm < 20` in T3.

**Expected T3 gain (B2a + B2b): 34 → 40–50 (+6 to +16 pts)**

---

## Slot-by-slot schedule

### v27 — UTC 2026-05-10 | Baseline restoration
**Owner:** Workstream A — Eng-1 (driver) + Eng-2 (reviewer, independent)

Code: **v26 as-is. Fresh `./submit.sh v27`. No new features. No re-tags.**

Fresh build eliminates ECR cold-pull and layer-caching variables that may have
contributed to the UTC 2026-05-09 infrastructure failures.

**Tonight (Eng-1, before build):**
1. Run eval-compose stack with ECR-pulled `ant:v26`. Measure wall-clock from
   `docker compose up` to ROS node Active state. Record in
   `sim_runs/run_2026-05-10_v27_postmortem/A0_startup_timing.md`.
   Target: Active state ≤ 60 s. If > 120 s → investigate pixi init.
2. Run full 3-trial local sim. Capture `policy.log` to
   `/tmp/ant_local_sim/policy.log` for submit.sh T3 gate (now 175 s).

**v27 acceptance gates:**
- T1 sim ≥ 50, T3 sim ≥ 30
- T3 wall-clock < 175 s (revised gate — see submit.sh Workstream C2)
- 0 `joint_space_guard_violation`, 0 Traceback/Exception, 3 × `trial_end`
- Independent reviewer (Eng-2) signs checklist before `submit.sh v27` runs

**Decision gate after v27 real-HW result:**
| v27 score | v28 action |
|---|---|
| ≥ 60 | Baseline confirmed → ship B1 in v28 |
| 23–59 | High-tension day variance → ship B1 anyway (B1 is pure escalation) |
| Second infra failure (no artifacts / timeout on known-good image) | Halt code work; re-tag v27 as v28; escalate to leads |

---

### v28 — UTC 2026-05-11 | B1: T1 joint-space escalation
**Owner:** Workstream B1 — Eng-3 (driver) + Eng-4 (reviewer)

Code: v27 + B1.

**Engineering spec:**

New flag in `ANT.__init__`:
```python
self.joint_space_t1_sfp = False   # v27 default; set True for v28 build
```

Wire location: `_localize_connector()` T1 SFP path, immediately after the
Bug 106 `_lateral_arrival_check_and_retry()` call returns with residual still
exceeding tolerance. If `joint_space_t1_sfp` is True and budget allows:

```python
if self.joint_space_t1_sfp and self._remaining_trial_budget_sec() > 25.0:
    js_result = self._lateral_move_joint_space(
        target_xy=(tgt_x, tgt_y),
        lateral_z=transit_z,
        orient=orient,
        zone="sfp",
        label="Stage 1 SFP T1 WP2 escalation",
        move_robot=move_robot,
        get_observation=get_observation,
        start_time=start_time,
        time_limit_sec=time_limit_sec,
    )
    if js_result is not None:
        actual_x, actual_y, orient, ok = js_result
        if ok:
            # proceed to Stage 2 from joint-space final position
            ...
```

No modification to Bug 106 block, T2 path, or T3 path.
The existing zone guard in `_lateral_move_joint_space` blocks T3 from
reaching this helper (guard fires on `zone != "sfp"` → returns None).

**Sim gates before `./submit.sh v28`:**
- `ANT-DIAG event=joint_space_start ... label="Stage 1 SFP T1 WP2 escalation"` fires
- T3 sim ≥ 30 (no regression from B1 wiring)
- T3 wall-clock < 175 s
- 0 guard violations, 0 tracebacks

**Risk mitigations:**
- Wrong-zone call → zone guard returns None, Cartesian fallback
- Budget exhaustion → `_remaining_trial_budget_sec() > 25.0` guard skips entry
- Force spike on mode switch → same Cartesian re-engagement as Bug 123

---

### v29 — UTC 2026-05-12 | B2: T3 lateral improvement
**Owner:** Workstream B2 — Eng-4 (driver) + Eng-3 (reviewer)

Code: v28 + B2.

**B2a (always ships):**
```python
self.sc_arrival_max_retries = 4   # new SC-specific parameter
```
Read at the SC WP2 `_lateral_arrival_check_and_retry()` call site instead of
`self.lateral_arrival_max_retries`. One flag, no new code paths.

**B2b (ships only if Eng-4 sim validates):**
```python
self.joint_space_sc_wp2 = False   # default-off
```
Guard update in `_lateral_move_joint_space`:
```python
# OLD:
if zone != "sfp":
    ...emit guard_violation...; return None
# NEW:
sc_allowed = self.joint_space_sc_wp2 and (zone == "sc")
if zone != "sfp" and not sc_allowed:
    ...emit guard_violation...; return None
```
Wire after SC WP2 Cartesian sub-step + B2a retry block.

**B2b validation gate (Eng-4, before committing B2b):**
Run a T3-only local sim with `joint_space_sc_wp2=True`.
If `ANT-DIAG event=joint_space_final ... err_mm < 20` fires → include B2b.
If T3 stalls identically to v28 → drop B2b, ship B2a only.
Do not stack unvalidated code.

**v29 decision gate:**
| v29 score | v30 action |
|---|---|
| ≥ 100 | Freeze; re-submit v29 as v30 variance check |
| 86–99 | v30 = Stage 4 SC constants tuning only |
| 75–85 | v30 = constants + one B2c knob (port-frame compliance, pre-approved) |
| < 75 | v30 = re-submit highest of v27/v28/v29; infra freeze |

---

### v30 — UTC 2026-05-13 | Integration + constants only
**Owner:** Workstream A — Eng-1 (driver) + Eng-2 (reviewer) + Eng-5 (line-by-line diff)

**Hard rules (both leads, non-negotiable):**
- No new functions
- No new flags
- No new control-flow branches
- Diff from v29 must be reviewable by one human in < 30 minutes

**Permitted changes (constants only):**
- `lateral_feedforward_n` ± 1–2 N
- `lateral_arrival_max_retries` / `sc_arrival_max_retries` ± 1
- Stage 4 SC `feedforward_fz` value (if T3 Stage 4 gap remains)
- `joint_space_arrival_tol_m` ± 5 mm
- `c1_signature_magnitude_m` tuning

**Infra freeze:** No `submit.sh` or `Dockerfile` changes after v29 commits.
Eng-5 enforces. Any exception requires Lead A + Lead B written sign-off.

---

### v31 — UTC 2026-05-14 | Release candidate
**Owner:** Lead A (driver) + Lead B (reviewer) — leads sign this one personally

**Selection rule:** Highest-scoring tag with ≥ 2 real-HW readings. If no tag
has two readings, use the best single reading.

Write selected image digest to `RELEASE_CANDIDATE_DIGEST` file, commit by
18:00 PST. Communicate final range to stakeholders by 19:00 PST if < 86.

---

### v32 — UTC 2026-05-15 | Final (mechanical re-tag only)
`./submit.sh v32` reads `RELEASE_CANDIDATE_DIGEST`, re-tags via AWS CLI in
ECR, pushes nothing new. Verifies digest equality before portal click.
**No `docker build` runs on May 15.**

---

## Workstream assignments

| WS | Scope | Driver | Reviewer | Active slots |
|---|---|---|---|---|
| A | Reliability, process, RC | Eng-1 | Eng-2 | v27, v30, v31 |
| B1 | T1 joint-space escalation | Eng-3 | Eng-4 | v28 |
| B2 | T3 SC WP2 + Stage 4 | Eng-4 | Eng-3 | v29 |
| C | submit.sh / infra (freeze after v29) | Eng-5 | Eng-1 backup | tonight → v29 |

---

## Non-negotiables (both leads)

1. **Every submission gets a postmortem.** Template must exist before
   `submit.sh` runs. `submit.sh` refuses push if prior postmortem has TODOs.

2. **Pairing rule is mandatory from v27 onward.** Driver and reviewer must be
   different humans. Engineer + Claude = one human. No exceptions without a
   Lead-signed waiver recorded in the checklist.

3. **No blind retries of a failing image.** If v27 also fails with no artifacts
   → stop code work, treat as systemic, re-tag for v28, escalate to leads.

4. **B1 and B2 never ship in the same slot.** Compounded risk is the enemy.
   New code paths and new diagnostic infrastructure never share a slot.

5. **No new code after v29.** v30 is constants only. v31/v32 are re-tags.

6. **May 15 = May 14.** `RELEASE_CANDIDATE_DIGEST` written by Lead A,
   co-signed by Lead B, committed before v32 executes. No source changes on
   May 15 under any circumstances.

---

## Risk register

| Risk | P | Mitigation |
|---|---|---|
| Eval infra fails again | 20% | Re-tag v27 as v28; do not burn code slots |
| B1 doesn't help T1 on high-tension days | 40% | Escalation-only; worst case = v27 score; B2 still ships v29 |
| B2b SC joint-space insufficient | 50% | B2a retries ship regardless; B2b validated in sim before commit |
| High-tension day on v31 RC | 20% | No mitigation — accept variance; target ≥ 2 readings before v31 |

---

## Immediate tonight actions

| # | Action | Owner | By |
|---|---|---|---|
| 1 | Fill `postmortem_v26.md` Score field: "infrastructure failure — v26 no artifacts, v24-fallback timeout" | Eng-1 | Tonight |
| 2 | Startup timing test: eval-compose + ECR-pulled `ant:v26`; record to `A0_startup_timing.md` | Eng-1 | Tonight |
| 3 | Run v27 local sim; capture `/tmp/ant_local_sim/policy.log` | Eng-1 | Tonight |
| 4 | Fix submit.sh T3 gate 110 s → 175 s with calibration comment | Eng-5 | **Done** (this session) |
| 5 | Assign Eng-2 as independent reviewer for v27 | Lead A | Tonight |
| 6 | Eng-3: begin B1 implementation in feature branch (no merge yet) | Eng-3 | Tonight/morning |
| 7 | Eng-4: B2a parameter analysis — SC WP2 retry budget vs trial wall-clock | Eng-4 | Tonight/morning |
| 8 | Both leads review and sign this plan | Lead A + Lead B | Before bed |
