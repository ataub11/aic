# B1 — T1 SFP Joint-Space Escalation (Eng-3)

**Branch:** `claude/v27-b1-t1-joint-space`
**Off:** `claude/analyze-v25-regression-25OyM` @ `bfd4e25`
**Status:** Code scaffold landed; sim validation pending tonight.
**Acceptance:** must close before tomorrow's bake-off (10:00 PST May 10).

---

## What B1 does

Adds a single new code path in T1 SFP WP2: **after** the existing Bug 106
arrival-check-and-retry, if the residual XY distance from target is
> `joint_space_t1_residual_threshold_m` (default 2.5 cm), invoke
`_lateral_move_joint_space` as an escalation. Joint-space delivers
~47 N at the TCP versus Cartesian's 15 N ceiling, which is the only
known mechanism for moving the arm against a high-tension cable
equilibrium.

Default-OFF flag (`joint_space_t1_sfp = False`) — v26 behaviour is
exactly preserved. v27 bake-off decides whether to flip it on.

---

## Files changed

- `ant_policy_node/ant_policy_node/ANT.py` — add flag in `__init__`,
  add escalation block after T1's `_lateral_arrival_check_and_retry`.
- `ant_policy_node/install/.../ANT.py` — keep in sync (Bug 90 protocol).
- `ant_policy_node/tests/test_v27_b1_t1_joint_space.py` — flag-default
  + call-site-ordering tests (host-side, no rclpy).

---

## Required tonight (Eng-3)

1. **Sync install/ tree** before any sim:
   ```bash
   cp ant_policy_node/ant_policy_node/ANT.py \
      ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   diff -q ant_policy_node/ant_policy_node/ANT.py \
           ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   ```

2. **Run host-side unit tests** — must be green:
   ```bash
   python3 -m unittest discover -s ant_policy_node/tests -v
   ```

3. **Sim run 1 — flag OFF** (verifies B1 is invisible when disabled):
   ```bash
   # Confirm joint_space_t1_sfp = False in source
   grep "self.joint_space_t1_sfp" ant_policy_node/ant_policy_node/ANT.py
   # Run local sim (whatever local_sim path the host uses)
   ./scripts/local_sim.sh
   # Capture log to this directory
   cp /tmp/ant_local_sim/policy.log \
      ant_policy_node/sim_runs/run_2026-05-09_b1_sweep/sim_flag_off.log
   ```
   **Acceptance:** T1 sim score within ±2 of v26 baseline; zero
   `b1_t1_joint_space_escalation` events; T1 path identical to v26.

4. **Sim run 2 — flag ON** (verifies escalation fires only when needed):
   ```bash
   # Temporarily flip the flag for this sim only
   sed -i 's/self.joint_space_t1_sfp = False/self.joint_space_t1_sfp = True/' \
       ant_policy_node/ant_policy_node/ANT.py
   cp ant_policy_node/ant_policy_node/ANT.py \
      ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   ./scripts/local_sim.sh
   cp /tmp/ant_local_sim/policy.log \
      ant_policy_node/sim_runs/run_2026-05-09_b1_sweep/sim_flag_on.log
   # IMPORTANT: revert the flag back to False before commit
   sed -i 's/self.joint_space_t1_sfp = True/self.joint_space_t1_sfp = False/' \
       ant_policy_node/ant_policy_node/ANT.py
   cp ant_policy_node/ant_policy_node/ANT.py \
      ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   ```
   **Acceptance gates:**
   - 3× `event=trial_end` events.
   - T1 sim score ≥ 50 (B1's reason to exist).
   - T3 sim score ≥ 30 (no T3 regression).
   - T3 wall-clock < 110s.
   - Zero `joint_space_guard_violation` events.
   - At least one `b1_t1_joint_space_escalation` event in T1 (otherwise
     the escalation never fired — sim cable tension probably too low to
     trip it; flag this as a sim-vs-HW gap, not a code bug).
   - No Traceback / Exception.

5. **Write results MD**: `ant_policy_node/sim_runs/run_2026-05-09_b1_sweep/RESULTS.md`
   following the template in this directory.

6. **Commit and push**:
   ```bash
   git add ant_policy_node/ant_policy_node/ANT.py
   git add ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py
   git add ant_policy_node/sim_runs/run_2026-05-09_b1_sweep/
   git commit -m "B1 sim validation: T1=<X> with flag on, no regression with flag off"
   git push origin claude/v27-b1-t1-joint-space
   ```

---

## Stop-rules (Eng-3 must surface to Lead A or Lead B)

| Sim outcome | Action |
|---|---|
| Flag-OFF sim regresses any trial vs v26 baseline | **HALT.** B1 has a side effect with flag off. Investigate before any further work. |
| Flag-ON T1 < 50 in sim | **Note for bake-off.** B1 may not lift T1 in sim; could still help on real-HW where cable equilibrium is the actual problem. Lead A/B decide. |
| Flag-ON T3 regresses below 30 | **HALT.** B1 broke T3 somehow (probably via shared state or timing). |
| Guard violations on flag-ON T1 | **Investigate.** The guard exists exactly for this — but if it fires on T1 SFP it means the guard's `zone != "sfp"` check has been compromised. |
| All gates pass | **Ready for bake-off.** Tomorrow's v27 candidate decision can include B1. |

---

## What NOT to do tonight

- Do NOT change the default flag value in the committed code (must stay
  `False`).
- Do NOT touch v26 source (anything in `claude/analyze-v25-regression-25OyM`
  past `bfd4e25`).
- Do NOT modify `submit.sh` or `pre_submit_checklist.md`.
- Do NOT push to `claude/analyze-v25-regression-25OyM` from this branch.
- Do NOT integrate with B2a tonight; bake-off is tomorrow.

---

## Tomorrow morning bake-off input from B1

Eng-3 produces three things by 10:00 PST:

1. The unit tests pass (`python3 -m unittest discover -s ant_policy_node/tests`).
2. `RESULTS.md` with sim results and a one-line recommendation:
   "RECOMMEND ENABLE / DISABLE / RESEARCH-MORE for v27."
3. The branch HEAD with flag = False (default-off ready to integrate).

The bake-off engineer reads RESULTS.md, decides yes/no, and either
cherry-picks the B1 commit into the v27 integration branch (with the
flag flipped on) or leaves it for v28+.
