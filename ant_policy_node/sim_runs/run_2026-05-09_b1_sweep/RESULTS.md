# B1 Sim Results — 2026-05-09 evening (Eng-3)

**Engineer:** TODO (Eng-3 fill in)
**Branch HEAD at sim-time:** TODO (paste `git rev-parse HEAD`)
**Local sim env:** TODO (sim version / commit if known)

---

## Sim run 1 — `joint_space_t1_sfp = False` (flag OFF, v26 parity)

| Metric | v26 baseline (expected) | This sim | Pass? |
|---|---|---|---|
| 3× trial_end events | yes | TODO | TODO |
| T1 sim score | 50–60 | TODO | TODO |
| T2 sim score | ~37 (sim doesn't reproduce real-HW T2 stall) | TODO | TODO |
| T3 sim score | ≥ 30 | TODO | TODO |
| T3 wall-clock | < 110s | TODO | TODO |
| `b1_t1_joint_space_escalation` event count | == 0 (flag off) | TODO | TODO |
| `joint_space_guard_violation` event count | == 0 | TODO | TODO |
| Traceback / Exception | none | TODO | TODO |

Log: `sim_flag_off.log`

## Sim run 2 — `joint_space_t1_sfp = True` (flag ON, B1 active)

| Metric | Acceptance gate | This sim | Pass? |
|---|---|---|---|
| 3× trial_end events | yes | TODO | TODO |
| T1 sim score | ≥ 50 (B1's reason to exist) | TODO | TODO |
| T2 sim score | ~37 unchanged | TODO | TODO |
| T3 sim score | ≥ 30 | TODO | TODO |
| T3 wall-clock | < 110s | TODO | TODO |
| `b1_t1_joint_space_escalation` event count | ≥ 0 (note: 0 means cable tension in sim too low to trip; not a code bug) | TODO | TODO |
| `joint_space_guard_violation` event count | == 0 (guard correctly admits T1 SFP) | TODO | TODO |
| Traceback / Exception | none | TODO | TODO |

Log: `sim_flag_on.log`

---

## Recommendation for v27 bake-off

**ENABLE / DISABLE / RESEARCH-MORE** (Eng-3 picks one, deletes the other two)

One sentence why:

TODO — e.g. "ENABLE: T1 sim 52 with flag on (escalation fired once,
converged), no regression with flag off, no guard violations."

---

## Notes for bake-off engineer

- B1 is a runtime-only change. The post-build marker check needs no
  new entries.
- The escalation diag event name is `b1_t1_joint_space_escalation`. If
  v27 ships with B1 enabled, the v27 postmortem should grep for this
  event in the eval-returned `policy.log` (if any) to confirm the
  escalation actually fired on real HW.
- B1 + B2a integrate cleanly — they touch different code paths
  (T1 SFP WP2 vs T3 SC yaw table) and have orthogonal flags.

---

## Post-bake-off cleanup

If B1 ships in v27, this branch can be deleted after the v27
integration commit lands. If B1 is deferred, leave the branch open
for v28+ consideration.
