# B2a — T3 SC Gripper-Yaw Sweep (Eng-4)

**Branch:** `claude/v27-b2a-yaw-sweep`
**Off:** `claude/analyze-v25-regression-25OyM` @ `bfd4e25`
**Status:** Override mechanism + sweep harness landed; sim sweep tonight.
**Acceptance:** Eng-4 RESULTS.md committed before bake-off (10:00 PST May 10).

---

## What B2a does

T3 SC currently uses `gripper_yaw_correction_rad[("sc", 3)] = -1.7133`
(Bug 122 ground-truth calibration from sim 2026-04-30b). v22-v24 score
T3 ≈ 34 with this value, plug ending 0.04 m from port.

B2a hypothesis: a slightly different yaw value may produce actual
insertion (T3 ≥ 50) by aligning the plug's flat with the port slot
more precisely. Real-HW board yaw may differ from sim board yaw by a
fraction of a degree, which the current single-point calibration cannot
capture.

This branch adds an **env-var override mechanism** (`ANT_T3_YAW_OVERRIDE_RAD`)
that lets Eng-4 sweep candidate values in sim without re-committing
ANT.py for each iteration. The committed default value stays `-1.7133`,
so v26 behavior is exactly preserved when the env var is unset.

---

## Files changed

- `ant_policy_node/ant_policy_node/ANT.py` — `_gripper_yaw_correction`
  reads `ANT_T3_YAW_OVERRIDE_RAD` when zone="sc" and trial=3; logs the
  override on first use; emits a `b2a_yaw_override` diag event.
- `ant_policy_node/install/.../ANT.py` — kept in sync (Bug 90 protocol).
- `scripts/b2a_yaw_sweep.sh` — sweep harness: 11 values × ~5 min each
  ≈ 60 min total. Captures per-value policy.log into the sweep dir.
- `ant_policy_node/sim_runs/run_2026-05-09_b2a_sweep/RESULTS.md` —
  template for Eng-4's recommendation.

---

## Required tonight (Eng-4)

1. **Sync install/** before any sim:
   ```bash
   cp ant_policy_node/ant_policy_node/ANT.py \
      ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   diff -q ant_policy_node/ant_policy_node/ANT.py \
           ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   ```

2. **Sanity test the override mechanism** with ONE sim run before
   committing to the full sweep:
   ```bash
   ANT_T3_YAW_OVERRIDE_RAD=-1.7133 ./scripts/local_sim.sh
   grep "B2A yaw override active" /tmp/ant_local_sim/policy.log
   # Should print:  B2A yaw override active: ('sc', 3) = -1.7133 ...
   grep "ANT-DIAG event=b2a_yaw_override" /tmp/ant_local_sim/policy.log
   # Should print:  ANT-DIAG event=b2a_yaw_override ... override_rad=-1.7133 baseline_rad=-1.7133
   ```
   If the override doesn't log, the mechanism is broken — STOP and
   investigate before sweeping.

3. **Run the full sweep**:
   ```bash
   ./scripts/b2a_yaw_sweep.sh ./scripts/local_sim.sh
   # Adjust the second arg if your local-sim wrapper is elsewhere.
   ```
   This runs 11 sims (yaw values from -1.7633 to -1.6633 in 0.01 steps),
   capturing each policy.log to `log_<value>.log` and a tab-separated
   summary to `sweep_summary.tsv`.

4. **Fill in T1/T2/T3 score columns** in `sweep_summary.tsv` from each
   per-value log. The harness's grep pattern depends on the local sim's
   score-reporting format — Eng-4 may need to add scores manually.

5. **Pick the best yaw value** by these tiebreakers (in order):
   - Highest T3 sim score.
   - T1 ≥ 50 (no T1 regression — yaw shouldn't affect T1 but verify).
   - T3 wall-clock < 110s.
   - Zero `joint_space_guard_violation` events.
   - Zero Traceback / Exception.

6. **Write RESULTS.md** with the recommendation. Format in the template.

7. **Commit and push**:
   ```bash
   git add ant_policy_node/sim_runs/run_2026-05-09_b2a_sweep/
   git commit -m "B2a sweep: best yaw=<value>, T3 sim=<score>"
   git push -u origin claude/v27-b2a-yaw-sweep
   ```

---

## Stop-rules

| Sweep outcome | Action |
|---|---|
| Sweep doesn't run (sim launcher missing) | Surface to Lead A immediately — same blocker affects A-track sim. |
| Override mechanism doesn't log | **HALT.** B2a is non-functional; investigate. Don't proceed to sweep. |
| Best value is the current `-1.7133` (no improvement) | **DISABLE B2a for v27.** Note in RESULTS.md. v27 ships without B2a. |
| Best value is materially better (T3 ≥ +5 sim score) | **ENABLE B2a for v27.** v27 integration commits the new value to the table (NOT the env var override; the env var is a dev tool only). |
| Best value differs from current by > 0.05 rad (outside sweep range) | **EXTEND SWEEP.** Run a wider sweep (±0.10 rad) before recommending. |
| Multiple values within noise of each other | **STAY WITH `-1.7133`.** Don't optimize on noise. |

---

## What NOT to do tonight

- Do NOT commit a change to the `gripper_yaw_correction_rad[("sc", 3)]`
  table value tonight. The committed change happens TOMORROW in the v27
  integration commit, after the bake-off engineer reads RESULTS.md.
- Do NOT touch v26 source on `claude/analyze-v25-regression-25OyM`.
- Do NOT modify `submit.sh` or `pre_submit_checklist.md`.
- Do NOT push to `claude/analyze-v25-regression-25OyM` from this branch.
- Do NOT integrate with B1 tonight; bake-off is tomorrow.

---

## Tomorrow morning bake-off input from B2a

Eng-4 produces three things by 10:00 PST:

1. `sweep_summary.tsv` with all 11 rows populated.
2. `RESULTS.md` with one-line recommendation:
   "RECOMMEND ENABLE (yaw=<value>, T3 sim=<score>) / DISABLE / EXTEND-SWEEP for v27."
3. The branch HEAD, no committed value change (table still at -1.7133).

The bake-off engineer reads RESULTS.md, decides yes/no, and either:
- Cherry-picks the override mechanism into v27 integration AND updates
  the table value to the chosen best.
- Or leaves both the mechanism and the override unused for v28+.

Either way, the env-var override mechanism is harmless to ship (it's a
no-op when the env var is unset, which is always the case in eval).
