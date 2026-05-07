# Bug 123 (v23) Validation Plan

**Status**: Implementation complete (commit 4debc38). Ready for simulation validation.

**Primary Objective**: T2 SFP lateral move should close from 17 cm (v22) to <2 cm (target).

## Sim Run 1: Full 3-Trial Baseline

### Success Criteria
| Trial | Metric | Target | v22 Baseline |
|-------|--------|--------|-------------|
| T1 SFP | Score | 29–31 pts | 29.48 |
| T1 | Lateral distance | ≤10 cm | (converges) |
| **T2 SFP** | **Score** | **≥37 pts** | **1.0 pt** ❌ |
| **T2** | **Lateral distance** | **<2 cm** | **17 cm** ❌ |
| T3 SC | Score | 34–35 pts | 34.08 |
| T3 | Lateral distance | ≤10 cm | (converges) |
| **Overall** | **Total Score** | **≥100 pts** | **64.55 pts** ❌ |

### Key Diagnostics to Inspect

Extract from `ant_diagnostics.jsonl`:

1. **Joint-space invocation** (T2 WP2):
   ```json
   {"event": "joint_space_start", "zone": "sfp", "label": "Stage 1 SFP T2 WP2",
    "tgt_x": ..., "tgt_y": ..., "delta_q_norm": ...}
   ```
   - Should see this event for T2 only (not T1, T3)
   - `delta_q_norm` should be in the 0.3–0.8 rad range for a 6 cm move

2. **IK convergence** (success path):
   ```json
   {"event": "joint_space_final", "zone": "sfp", "label": "Stage 1 SFP T2 WP2",
    "tgt_x": ..., "tgt_y": ..., "actual_x": ..., "actual_y": ...,
    "err_mm": ..., "within_tol": true, "ok": true}
   ```
   - **CRITICAL**: `err_mm < 20` (within 2 cm tolerance). Threshold for T2 success.
   - `within_tol=true` means the move converged cleanly.

3. **IK failure paths** (fallback triggers):
   ```json
   {"event": "joint_space_ik_failed", "zone": "sfp", "label": "Stage 1 SFP T2 WP2",
    "tgt_x": ..., "tgt_y": ...}
   ```
   - If this appears: check whether the target XY is in a singularity region or if damping is too aggressive.

4. **Stage 1 completion** (path marker):
   ```json
   {"event": "trial_stage1_complete", "zone": "sfp", "trial": 2, "path": "joint_space"}
   ```
   - If `"path": "joint_space"`: T2 succeeded, skipped Cartesian split/retry.
   - If absent or shows `"path": "cartesian_fallback"`: T2 fell back to v22 behavior.

5. **Force log during T2 lateral** (check for mode-switch impulse):
   - From `controller_state.tcp_error` and wrist wrench during the joint-space move phase
   - Should NOT exceed 20 N during the Cartesian re-engagement step (line 1044)
   - Expected: clean handoff with <5 N transient at mode switch

### Log File Locations

Run directory structure:
```
ant_policy_node/sim_runs/run_2026-05-0X/
├── ant_diagnostics.jsonl       ← Event stream (parse as JSONL)
├── ant_policy.log              ← Full stdout/stderr from ANT policy
└── launch.log                  ← ROS 2 launch logs
```

### Parse Commands

```bash
# Extract T2 joint-space events:
grep '"zone": "sfp".*"stage 1 sfp t2 wp2"' \
  ant_policy_node/sim_runs/run_2026-05-0X/ant_diagnostics.jsonl | \
  jq '{event, tgt_x, tgt_y, actual_x, actual_y, err_mm, within_tol, ok}'

# Check T1/T3 unaffected (should see NO joint_space events):
grep '"zone": "sfp"' ant_diagnostics.jsonl | \
  jq '.event' | sort | uniq -c

# Extract stage-1 completion paths:
grep 'trial_stage1_complete' ant_diagnostics.jsonl | \
  jq '{trial, zone, path}'

# Force spike check (grep raw log for Mode switches):
grep -i "cartesian re-engage" ant_policy.log
grep -i "joint-space final err" ant_policy.log
```

## Interpretation Guide

### Outcome A: T2 Closes to <2 cm (EXPECTED SUCCESS)

**Diagnostics signature**:
- `event=joint_space_final` with `err_mm=5..19 mm, within_tol=true, ok=true`
- `event=trial_stage1_complete` with `path=joint_space`
- T1, T3 show **no** `joint_space_*` events (Cartesian paths unchanged)
- Total score **≥100 pts** (expected 100–106)

**Next step**: Proceed directly to real-HW submission (`./submit.sh v23`).

### Outcome B: T2 Reaches 2–5 cm (MARGINAL SUCCESS)

**Diagnostics signature**:
- `err_mm=20..50 mm, within_tol=false, ok=false`
- **BUT** still shows `path=joint_space` (moved via joints, didn't fallback)
- Controller converged, but outside 2 cm tolerance
- Total score **≥90 pts** (expected 90–99)

**Diagnosis**: IK solver is working; error is > arrival tolerance. Options:
1. **Damping too aggressive** — increase λ from 0.05 to 0.02 (looser, faster)
2. **Arrival tolerance too tight** — loosen from 0.020 m to 0.025 m (test empirically)
3. **Step cap too small** — increase max_step_rad from 0.15 to 0.20 for early iterations
4. **Singularity near target** — IK pushed into elbow-straight region

**Action**: Apply optimization #1 from "Code Optimization Opportunities" above and re-run Sim 2.

### Outcome C: T2 Stalls; Falls Back to Cartesian (FAILURE / NO PROGRESS)

**Diagnostics signature**:
- `event=joint_space_ik_failed` OR `event=joint_space_final err_mm=170..200`
- `path=cartesian_fallback` (fell back to v22 Cartesian path)
- T2 score **≈1.0 pt** (same as v22)
- Total score **≈64–65 pts** (no improvement)

**Diagnosis by event type**:

#### C1: `joint_space_ik_failed`
- DLS solver did not converge or solution violates `max_total_delta_rad=0.6`
- Check if target_xy is in a joint-singular region (wrist aligned, elbow straight)

**Actions**:
1. Increase `max_iter` from 50 to 100 (more iteration budget)
2. Decrease `damping` from 0.05 to 0.02 (less stiffness, faster convergence near target)
3. Increase `max_step_rad` from 0.15 to 0.20 (aggressive early steps)
4. Check if target (tgt_x, tgt_y) is geometrically reachable for UR5e from home pose

#### C2: `joint_space_timeout`
- Exceeded 20 sec budget in the `while True` loop (line 990–1031)
- Convergence is slow or stall detector never fired

**Actions**:
1. Increase `joint_space_stage_timeout_sec` from 20 to 30
2. Apply damping reduction (looser → faster convergence)
3. Apply step-scaling optimization (aggressive early, fine late)

#### C3: `joint_space_final err=170+ mm, ok=false`
- Controller moved but to wrong place (post-move actual_xy far from target)
- May indicate post-move drift due to compliance or force-feedback oscillation

**Actions**:
1. Check `joint_space_park_settle_sec` — increase from 0.6 to 1.0 sec (more dwell before reading final)
2. Check if Cartesian re-engagement has stale stiffness from prior mode
3. Verify Jacobian conditioning near the target (may need damping increase for stability)

## Sim Run 2: Damping / Convergence Tuning (if Run 1 shows Outcome B or C)

If Run 1 shows marginal success (B) or failure (C) with IK/timeout clues:

1. **Hypothesis A: Damping too stiff (λ=0.05)**
   - Edit ANT.__init__: `self.joint_space_damping_override = 0.02` (if not already parameterized in ur5e_kinematics)
   - Re-run T2 only
   - Expected: faster convergence, <1mm error in 15–20 iterations instead of 30+

2. **Hypothesis B: Max-step cap too conservative (max_step_rad=0.15)**
   - Edit ur5e_kinematics.solve_ik_dls: add early-iteration scaling
   - Re-run T2
   - Expected: 25–30% convergence speedup

3. **Hypothesis C: Singular target geometry**
   - Manually check home-to-target IK offline: 
     ```python
     from ant_policy_node import ur5e_kinematics
     q_home = [0, -pi/2, -pi/2, -pi/2, pi/2, 0]
     T_target = ur5e_kinematics.forward_kinematics(q_home)
     T_target[:3,3] += [0.06, 0, 0]  # 6 cm offset
     q_sol = ur5e_kinematics.solve_ik_dls(T_target, q_home)
     print(f"IK solution: {q_sol}")
     ```
   - If None: target is unreachable or in singularity

## Force-Spike Instrumentation (Sim Run 3, if Runs 1–2 succeed)

**Goal**: Verify no >20 N impulse during the Cartesian re-engagement phase.

**Instrumentation**:
1. Enable high-frequency wrench logging (100 Hz or higher) during T2 WP2
2. Mark timestamps for:
   - `t_js_start`: when JointMotionUpdate is published (line 999)
   - `t_js_final`: when joint move completes / stall detected (line 1031)
   - `t_cartesian_reeng`: when Cartesian MotionUpdate is published (line 1044)
   - `t_settle`: after `park_settle_sec` sleep (line 1049)

3. Extract wrist_wrench.wrench.force magnitude for each interval:
   - `[t_js_start, t_js_final]`: should be smooth ramp (joint servo feedback)
   - `[t_cartesian_reeng - 0.1s, t_cartesian_reeng + 0.5s]`: **CRITICAL** zone
     - Should NOT spike >20 N (hard limit for competition force penalty)
     - Expected: <5 N transient, settling within 2 sec

**Plot** (optional but valuable):
```
Force magnitude vs. time for T2 WP2
- Mark mode switches (joint→cartesian)
- Check for impulses >20 N
- Compare to v22 baseline (should be much lower in joint mode due to torque-limited servoing)
```

## Real-HW Submission Gate

**Criteria to ship v23**:
- ✅ Sim Run 1: T2 score ≥37 pts (distance <2 cm) OR Outcome B marginal + Run 2 tuned successfully
- ✅ Force-spike instrumentation: no >20 N transients during re-engagement
- ✅ T1, T3 unchanged (no regression)
- ✅ Total expected score ≥100 pts

**Abort criteria**:
- ❌ Run 1: Outcome C (fallback every trial)
- ❌ Run 2 tuning fails to improve past 5 cm
- ❌ Force spike >20 N observed (indicates mode-switch issue)

## Post-Submission (Real-HW Observation)

Once v23 is submitted to competition and scored:

1. **If score ≥100**: Success. Document in CLAUDE.md, close Bug 123.
2. **If score 85–99**: Marginal (T2 improved but not by +36). Investigate outliers in real-HW force data. May need optimization #5 (stiffness tuning) applied for the next submission.
3. **If score <85**: Fallback triggered or worst-case forces. Review `ant_diagnostics.jsonl` for `event=joint_space_*` records. Compare real-HW damping behavior vs. sim.

---

## Code Pointers for Debugging

- **IK convergence**: `ur5e_kinematics.py:163-219` (`solve_ik_dls`)
- **DLS pseudoinverse**: `ur5e_kinematics.py:205-209` (damping factor application)
- **Joint-space lateral main loop**: `ANT.py:990–1031` (stall detector, convergence check)
- **Cartesian re-engagement**: `ANT.py:1032–1049` (mode-switch impulse mitigation)
- **T2 wiring**: `ANT.py:1752–1790` (fallback logic)

## Tunables Summary

All defaults should work for Sim 1. If tuning needed (Runs 2+):

```python
# ur5e_kinematics.py (solve_ik_dls defaults)
damping: float = 0.05              # ← reduce to 0.02 if slow convergence
max_iter: int = 50                 # ← increase to 100 if timeout
max_step_rad: float = 0.15         # ← increase to 0.20 for aggressive phase

# ANT.py __init__ (currently set)
self.joint_space_arrival_tol_m = 0.020         # 2 cm arrival threshold
self.joint_space_stage_timeout_sec = 20.0      # 20 sec budget
self.joint_space_park_settle_sec = 0.6         # dwell before final read
```

---

**Next Step**: Run Sim 1 with the current code. If Outcome A (T2 <2 cm), proceed directly to `./submit.sh v23`. If B or C, apply targeted tuning from Sim Run 2 section.
