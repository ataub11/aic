# B1 Code Review — v28 Submission
**Reviewer:** Eng-4 (independent)
**Branch reviewed:** `claude/b1-joint-space-t1-sfp`
**Commit reviewed:** `75fb288` ("B1: add joint_space_t1_sfp flag + T1 WP2 escalation block (default-off)")
**Review date (UTC):** 2026-05-10

---

## Summary

The B1 commit adds a single new flag `joint_space_t1_sfp` (defaulting to
`False` in the commit, enabled to `True` for the v28 build) and wires a
joint-space escalation block into the T1 SFP lateral path in
`_localize_connector()`. For v28, Eng-3 flips the flag to `True`. The
change is surgically contained: 22 lines added to `ANT.py` (source tree and
install/ copy identically), zero other control-flow paths modified.

**One-sentence summary:** Setting `joint_space_t1_sfp = True` causes the arm
to attempt a joint-space lateral move on T1 SFP after the Bug 106
Cartesian retry loop exhausts (escalation-only, never primary), bypassing
the 15 N Cartesian impedance ceiling that stalls the arm under high cable
tension; it is safe because the zone guard in `_lateral_move_joint_space`
blocks any non-SFP caller with an unconditional early return, the 25 s
budget guard prevents entry when time is short, and any failure falls
through to the existing v27 Cartesian path leaving worst-case score
unchanged.

---

## Check 1 — Flag declaration in `__init__`

**PASS**

At commit `75fb288`, `ANT.__init__` declares:

```python
self.joint_space_t1_sfp = False   # B1 (v28): T1 joint-space escalation; default-off
```

This is at line 553 of the source file (immediately after
`self.joint_space_t2_sfp = True`). The flag is `False` in the B1 commit as
required — "default-off" per the campaign plan. The v28 build line-change
(flipping to `True`) is Eng-3's responsibility and is not in this commit,
which is correct per the spec ("Eng-3 driver + Eng-4 reviewer" split).

Note: the current working tree on `claude/b2-sc-wp2-improvement` shows
`self.joint_space_t1_sfp = True` because Eng-3 has already applied the
v28 flag flip on the downstream branch. This is expected and does not
indicate a problem with the B1 commit itself.

---

## Check 2 — Escalation block: wire location, budget guard, fallthrough

**PASS**

The escalation block at lines 2273–2293 (source ANT.py) is wired
immediately after the `_lateral_arrival_check_and_retry()` return in the
T1 SFP path (`else` branch of the `trial == 2` condition, inside `if
zone == "sfp"`). This matches the spec exactly.

Spec excerpt (CAMPAIGN_PLAN §v28):

> Wire location: `_localize_connector()` T1 SFP path, immediately after the
> Bug 106 `_lateral_arrival_check_and_retry()` call returns ...

Actual wiring in commit `75fb288` diff:

```python
                _, _, orient = self._lateral_arrival_check_and_retry(...)  # Bug 106
+               # B1 (v28): joint-space escalation after Bug 106 retries exhaust.
+               if self.joint_space_t1_sfp and self._remaining_trial_budget_sec() > 25.0:
+                   js_result = self._lateral_move_joint_space(
+                       target_xy=(tgt_x, tgt_y),
+                       lateral_z=transit_z,
+                       orient=orient,
+                       zone="sfp",
+                       label="Stage 1 SFP T1 WP2 escalation",
+                       ...
+                   )
+                   if js_result is not None:
+                       actual_x, actual_y, orient, ok = js_result
+                       if ok:
+                           tgt_x, tgt_y = actual_x, actual_y
```

Budget guard is `self._remaining_trial_budget_sec() > 25.0` — matches spec.

Fallthrough logic is correct:
- If `js_result is None` (zone guard fired, IK failed, etc.): falls through
  to `obs_final = self._wait_for_observation(...)` and the existing
  Cartesian return — i.e., v27 behaviour.
- If `js_result is not None` but `ok is False` (joint-space stalled outside
  tolerance): same fallthrough, `tgt_x, tgt_y` unchanged.
- Only if `ok is True` does `tgt_x, tgt_y` update to the joint-space
  landing position before Stage 2.

This matches the "proceed to Stage 2 from joint-space final position"
snippet in the campaign plan exactly.

The spec's `label="Stage 1 SFP T1 WP2 escalation"` is present verbatim,
so the sim gate grep `ANT-DIAG event=joint_space_start ... label="Stage 1
SFP T1 WP2 escalation"` will fire correctly when the flag is enabled.

---

## Check 3 — Zone guard in `_lateral_move_joint_space`

**PASS**

The helper at line 1163 contains an unconditional early-return guard at
lines 1192–1198:

```python
if zone != "sfp":
    self._diag_event(
        "joint_space_guard_violation",
        zone=zone, label=label, reason="non_sfp_zone",
    )
    self._c1_record_failure("guard_violation")
    return None
```

This guard:
1. Runs FIRST — before any flag checks or IK computation.
2. Is unconditional — it cannot be disabled by `enable_lateral_arrival_check`,
   `enable_joint_space_lateral`, or any other toggle.
3. Emits a C1-encodable diagnostic event on violation (guard_violation code),
   ensuring any errant call is observable in `ant_diagnostics.jsonl`.

The code comment explicitly documents this design:
> "NOTE: this guard runs FIRST so it cannot be bypassed by future toggles of
> `enable_lateral_arrival_check` or `enable_joint_space_lateral`
> (Eng-3/Eng-4 feedback on b078aee)."

---

## Check 4 — T3 protection (defense-in-depth)

**PASS**

T3 cannot reach the B1 escalation block via two independent mechanisms:

**Primary: structural branch isolation.**
The entire SFP fast path block is guarded by `if zone == "sfp":` (line
2038). `zone` is set once at the top of `_localize_connector()` via
`zone = "sfp" if task.plug_type == "sfp" else "sc"`. T3 has `plug_type =
"sc"`, so `zone = "sc"`, and the SFP block (which contains the B1
escalation at line 2277) is never entered.

**Defense-in-depth: zone guard in helper.**
Even if a future refactor accidentally passed `zone="sc"` to
`_lateral_move_joint_space`, the unconditional guard at line 1192
(`if zone != "sfp": return None`) would terminate the call immediately
with a logged guard_violation event and C1 code.

There are exactly two callers of `_lateral_move_joint_space` in the
codebase:
- Line 2076: T2 SFP path, `zone="sfp"`.
- Line 2278: B1 T1 SFP escalation, `zone="sfp"`.

Neither call originates from the SC/T3 path.

---

## Check 5 — install/ sync (Bug 90)

**PASS**

The B1 commit diff shows that both copies were updated atomically in the
same commit:

```
 ant_policy_node/ant_policy_node/ANT.py             | 22 ++++++++++++++++++++++
 .../site-packages/ant_policy_node/ANT.py           | 22 ++++++++++++++++++++++
 2 files changed, 44 insertions(+)
```

The install/ diff is byte-for-byte identical to the source diff (both add
the same 22 lines at the same offsets). No sync discrepancy.

The `submit.sh` pre-build sync would also catch any future drift before a
push, but the committed state is already clean.

---

## Check 6 — Campaign plan spec conformance (CAMPAIGN_PLAN §v28)

**PASS** (with one advisory note — see F2 below)

| Spec requirement | Actual implementation | Status |
|---|---|---|
| Flag in `__init__`, default `False` | `self.joint_space_t1_sfp = False` at line 553 | PASS |
| Wire location: after `_lateral_arrival_check_and_retry()` in T1 SFP path | Lines 2259–2293, `else` branch of `trial==2` inside `if zone=="sfp"` | PASS |
| Budget guard `> 25.0 s` | `self._remaining_trial_budget_sec() > 25.0` | PASS |
| `zone="sfp"` passed to helper | `zone="sfp"` at line 2283 | PASS |
| `label="Stage 1 SFP T1 WP2 escalation"` | Verbatim match | PASS |
| Fallthrough on `None` or `ok=False` | No mutation of `tgt_x, tgt_y` unless `ok=True` | PASS |
| install/ copy synced | Identical 22-line delta in same commit | PASS |
| No modification to Bug 106 block | Bug 106 call at 2259 untouched | PASS |
| No modification to T2 path | T2 path (`trial==2` branch) untouched | PASS |
| No modification to T3 path | SC block (after `if zone=="sfp"` exits) untouched | PASS |

---

## Advisory: Flag F2 — Thin physics margin at 10 cm stall

**NOTE (not a blocking issue)**

The campaign plan notes:

> "B1 caveat: T1 WP2 distance is short (~5–8 cm); joint-spring force at TCP
> ~14 N may be < 25 N cable equilibrium."

At a 5–8 cm position error, joint spring force at the shoulder
(200 Nm/rad × ~0.06/0.85 m ≈ 0.07 rad ≈ 14 Nm → ~16 N at TCP via
Jacobian) may not exceed the worst-case cable equilibrium of ~25 N. This
is a meaningful physics concern:

- T2 operates at 17 cm error where the spring force (~47 N) comfortably
  exceeds the cable equilibrium.
- T1's residual stall after Bug 106 retry exhaustion is typically
  5–10 cm, putting the spring force at 14–28 N — straddling the cable
  equilibrium band.
- On a worst-case high-tension day (cable equilibrium at the upper end of
  the observed 18–25 N range), B1 may fail to converge and fall back to
  the Cartesian path anyway.

Mitigation already in place: the fallthrough logic guarantees worst-case
= v27 score. B1 is escalation-only, never primary. If T1 residual is
regularly < 5 cm after Bug 106 retries, the spring force margin is
adequate and the flag earns its keep. The sim gate (T3 sim >= 30, 0
guard violations) will surface any unexpected regression before
submission.

Recommendation: after the first v28 sim run, grep
`event=joint_space_arrival` in the diagnostic log for T1 to see whether
the helper is converging. If residuals cluster near 8–10 cm, note the
thin margin and consider narrowing the budget guard to `> 30.0 s` or
widening `joint_space_max_total_delta_rad` before the real-HW submission.

This flag-F2 concern is recorded here for the team's awareness; it does
not block approval because the fallthrough safety net is robust and the
sim gate is the mechanical enforcement layer.

---

## Sign-off

Eng-4 B1 code review: **APPROVED for v28 submission** — UTC 2026-05-10

All four structural checks pass. The B1 commit is a clean, minimal change
(22 lines, 2 files) that introduces no new control-flow for T2, T3, or
any non-SFP path. The zone guard and budget guard provide defense-in-depth.
The install/ copy is in sync. The only open item is the F2 physics margin
advisory, which is a known-and-accepted risk per the campaign plan and
does not warrant a blocking change given the robust fallthrough path.
