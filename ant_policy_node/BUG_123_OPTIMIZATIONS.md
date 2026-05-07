# Bug 123 Code Optimizations

## Priority 1: Immediate if Sim Run 1 Shows Slow Convergence (Outcome B/C)

### Opt-1A: Adaptive Damping Based on Jacobian Condition

**Current code** (`ur5e_kinematics.py:205-209`):
```python
damping: float = 0.05  # Fixed
JJt_damped = JJt + (damping ** 2) * I6
```

**Problem**: Fixed λ=0.05 is conservative (stable but slow). Near singularities (high condition number), it prevents overshoot but slows convergence. In well-conditioned regions, we can be more aggressive.

**Optimization** (effort: low, +2–5 points expected):
```python
# In solve_ik_dls loop, after computing J:
def _adaptive_damping(JJt, base_damping=0.05):
    """Scale damping based on Jacobian conditioning."""
    eigvals = np.linalg.eigvalsh(JJt)
    cond = eigvals[-1] / (eigvals[0] + 1e-8)
    if cond < 100:
        return base_damping * 0.5  # well-conditioned: use 0.025
    elif cond < 500:
        return base_damping  # moderate: use 0.05
    else:
        return base_damping * 2.0  # singular-adjacent: use 0.1
```

**Where to add** (line 204 in ur5e_kinematics.py):
```python
J = geometric_jacobian(q)
cond_number = ...  # compute condition number
damping_adaptive = _adaptive_damping(J @ J.T, base_damping=damping)
JJt_damped = JJt + (damping_adaptive ** 2) * I6
```

**Expected impact**: 1–2 fewer iterations (50 ms speedup per move). Convergence from 25+ iterations down to 18–20 in the well-conditioned zone.

---

### Opt-1B: Per-Iteration Step-Scaling

**Current code** (`ur5e_kinematics.py:213-216`):
```python
max_step_rad: float = 0.15  # Fixed per-iteration cap
if step_norm > max_step_rad:
    delta_q *= max_step_rad / step_norm
```

**Problem**: Fixed cap is conservative early (when error is large) and overly cautious late (when fine-tuning). This delays convergence.

**Optimization** (effort: low, +1–3 points):
```python
# Adaptive step cap: aggressive early, fine late
progress_pct = iteration / max_iter
if progress_pct < 0.3:
    step_cap = 0.20  # early: allow big steps
elif progress_pct < 0.7:
    step_cap = 0.15  # mid: moderate steps
else:
    step_cap = 0.10  # late: fine-tuning

if step_norm > step_cap:
    delta_q *= step_cap / step_norm
```

**Where to add** (line 213):
Replace the fixed `max_step_rad` line with the logic above.

**Expected impact**: 25–50 ms convergence speedup. More relevant for larger moves (6+ cm).

---

### Opt-1C: Tighter Joint-Limit Margin

**Current code** (`ur5e_kinematics.py:198–201`):
```python
if np.any(np.abs(q) > _JOINT_LIMIT_RAD):  # _JOINT_LIMIT_RAD = π
    return None  # Solution violates ±π limit
```

**Problem**: Allows solution to approach ±π, which is a joint singularity. Numeric instability can cause failures near the boundary.

**Optimization** (effort: low, +1–2 points):
```python
JOINT_LIMIT_MARGIN = 0.95 * _JOINT_LIMIT_RAD  # 0.95π ≈ 2.99 rad
if np.any(np.abs(q) > JOINT_LIMIT_MARGIN):
    return None  # Reject near-limit solutions
```

**Where to add** (line 200 in ur5e_kinematics.py):
Replace `_JOINT_LIMIT_RAD` with `JOINT_LIMIT_MARGIN` in the check.

**Expected impact**: Eliminates 1–2% of near-singular IK solutions that are geometrically valid but numerically unstable. Expected result: fewer IK_failed events, especially on edge-case move geometries.

---

## Priority 2: If Outcome C (Timeout or Stall)

### Opt-2A: Increase Iteration Budget

**Current code** (`ur5e_kinematics.py:163`):
```python
max_iter: int = 50
```

**Optimization**:
```python
max_iter: int = 100  # Double the budget
```

**Rationale**: If the solver times out at 20 sec with only 50 iterations, more iterations (especially with damping reduction) may help. Worst case: still converges in <20 sec but with more stable trajectory.

**Where to add**: Update the default parameter in the function signature.

**Expected impact**: Solves the timeout path if convergence is slow but on-track.

---

### Opt-2B: Reduce Default Damping (Looser Damping)

**Current code** (`ur5e_kinematics.py:169`):
```python
damping: float = 0.05
```

**Optimization** (applies globally):
```python
damping: float = 0.02  # Looser; less stable but faster
```

**Rationale**: Lower damping means faster convergence but can cause oscillation near singularities. In well-conditioned regions (most T2 moves), this is harmless.

**Alternative** (per-call tuning): Update ANT.py to pass damping parameter:
```python
# ANT.py _solve_ik_for_tcp:
q_sol = ur5e_kinematics.solve_ik_dls(
    target_pose, q_current,
    damping=self.joint_space_damping_override,  # new tunableadding to __init__
)
```

**In __init__**:
```python
self.joint_space_damping_override = 0.02  # Master control
```

**Expected impact**: 2–3x speedup in convergence for well-conditioned moves.

---

## Priority 3: Fine-Tuning (If Run 1 shows Outcome B at 2–5 cm)

### Opt-3A: Tighten Stall-Detector Threshold

**Current code** (`ANT.py:984–986`):
```python
stall_window = 5
stall_tol_m = 0.002  # 2 mm stall tolerance
```

**Problem**: Stall detector waits 5 iterations of <2mm change before giving up. On a 6 cm move, 2 mm may be too coarse a threshold.

**Optimization**:
```python
stall_tol_m = 0.001  # 1 mm — more aggressive exit
```

**Where to add** (ANT.py line 985):
```python
stall_tol_m = 0.001  # Updated threshold
```

**Expected impact**: Detects stalls 1–2 sec earlier, allowing fallback to Cartesian sooner if joint servo is genuinely stuck. Minimal risk.

---

### Opt-3B: Extend Joint-Space Timeout (if hitting 20 sec budget)

**Current code** (`ANT.py:483`):
```python
self.joint_space_stage_timeout_sec = 20.0
```

**Optimization**:
```python
self.joint_space_stage_timeout_sec = 30.0  # 30 sec budget
```

**Where to add** (ANT.py __init__, line 483):

**Expected impact**: Allows 50% more time for slow convergence. Use only if Outcome C shows `joint_space_timeout`.

---

### Opt-3C: Increase Arrival Tolerance (if consistently ~2–3 cm off)

**Current code** (`ANT.py:482`):
```python
self.joint_space_arrival_tol_m = 0.020  # 2 cm
```

**Optimization**:
```python
self.joint_space_arrival_tol_m = 0.025  # 2.5 cm (or 0.030 = 3 cm)
```

**Rationale**: If the solver converges to 2.5 cm and stalls (joints hit limit), declaring success at 2.5 cm still beats the v22 17 cm by 6.8x.

**Warning**: Loosening this gate allows stage 2 to start further from the port, risking Stage 3 insertion difficulty. Use only as a last resort if Outcome B (not quite converging to 2 cm) persists.

**Expected impact**: Gains 2–5 points if T2 consistently reaches 2–3 cm but IK refuses to go further.

---

## Priority 4: Architecture / Mode-Switch Debugging (If Outcome C with >100 mm error)

### Opt-4A: Verify Cartesian Re-Engagement Mode Switch

**Symptom**: `joint_space_final err > 100 mm` despite IK success.

**Cause**: Cartesian re-engagement may have a transient before mode switch completes.

**Debug code** (`ANT.py` around line 1044):
```python
park_motion = self._build_motion_update(
    park_pose, self.approach_stiffness,
    feedforward_fy=0.0, feedforward_fz=0.0,
)
self.get_logger().info(
    f"{label}: Cartesian re-engage: tcp_pose=({park_pose.position.x:.4f}, "
    f"{park_pose.position.y:.4f}, {park_pose.position.z:.4f}), "
    f"orient=({park_pose.orientation.x:.3f}, {park_pose.orientation.y:.3f}, "
    f"{park_pose.orientation.z:.3f}, {park_pose.orientation.w:.3f})"
)
move_robot(motion_update=park_motion)
```

**Read back controller state immediately after publish**:
```python
self.sleep_for(0.1)  # very short settle
obs_check = get_observation()
actual_after_reeng = (
    obs_check.controller_state.tcp_pose.position.x,
    obs_check.controller_state.tcp_pose.position.y,
)
self.get_logger().info(
    f"{label}: Post re-engage check: actual={actual_after_reeng}"
)
```

**Expected**: Re-engagement pose should match the joint-move final pose (no jump).

---

### Opt-4B: Check Orientation Consistency

**Problem**: Joint move may rotate the gripper slightly. If re-engagement doesn't preserve the new orientation, there's a pose mismatch.

**Current code** (`ANT.py:1055`):
```python
orient = obs_final.controller_state.tcp_pose.orientation
```

**Verify** the orient is being carried through correctly to the next stage (WP3 descent).

---

## Performance Baseline (for Reference)

**Expected iteration counts (typical moves)**:
- 6 cm lateral move: 15–25 iterations (1.0–1.5 sec wall-clock)
- 10 cm move: 20–35 iterations
- 17 cm move (worst-case): 30–50 iterations

**Convergence window**:
- Position error typically drops exponentially: 30 mm → 10 mm → 3 mm → 1 mm over ~20 iterations.
- Final 1 mm takes the last 5–10 iterations (fine-tuning phase).

**With optimizations Opt-1A + Opt-1B**:
- Expected: 25% faster convergence (6–10 fewer iterations)
- Practical impact: 6 cm move in 0.8–1.0 sec instead of 1.0–1.5 sec

---

## Regression Testing Checklist

After applying any optimization, verify:

- [ ] T1 SFP: unchanged behavior (not wired to joint-space)
- [ ] T3 SC: unchanged behavior
- [ ] T2 WP2: joint-space move completes
- [ ] **Force spike check**: no >20 N transient during re-engagement
- [ ] **Stage 3 descent**: proceeds normally from actual TCP pose
- [ ] **Overall score**: meets target (≥100 pts)

---

## Commit Strategy

Create separate commits for each optimization to isolate regressions:

```bash
# Apply Opt-1A
git add ur5e_kinematics.py
git commit -m "Bug 123: Opt-1A adaptive damping based on Jacobian condition"

# Apply Opt-1B
git add ur5e_kinematics.py
git commit -m "Bug 123: Opt-1B per-iteration step scaling (aggressive early, fine late)"

# Apply Opt-1C
git add ur5e_kinematics.py
git commit -m "Bug 123: Opt-1C tighter joint-limit margin (avoid ±π singularity)"

# Re-run Sim 1 after each commit, track T2 distance.
```

This allows rollback if a particular optimization causes regression.

---

## Expected Optimization Order

If Sim 1 shows Outcome B/C:

1. **First try**: Opt-1A (adaptive damping) — safest, +3 points expected
2. **If still slow**: Opt-1B (step scaling) — +2 points, faster early convergence
3. **If still slow**: Opt-2B (reduce global damping) — more aggressive, +2 points
4. **If timeout**: Opt-2A (increase max_iter) + Opt-2B combined
5. **If singularity near target**: Opt-1C (tighter margin) + Opt-1A (adaptive damping)
6. **Last resort**: Opt-3C (loosen arrival tolerance) — trade precision for passing

Do NOT apply all at once — isolate which optimization actually helps.
