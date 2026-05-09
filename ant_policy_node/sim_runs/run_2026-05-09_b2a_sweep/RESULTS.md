# B2a Sim Sweep Results — 2026-05-09 evening (Eng-4)

**Engineer:** TODO (Eng-4 fill in)
**Branch HEAD at sim-time:** TODO (paste `git rev-parse HEAD`)
**Sim launcher used:** TODO (e.g. `./scripts/local_sim.sh`)
**Sweep window:** -1.7633 to -1.6633 in 0.01 rad steps (11 values)

---

## Sweep summary table

(Copy/paste from `sweep_summary.tsv` and fill in score columns from each
per-value `log_<value>.log`.)

| yaw_rad | trial_count | T1 | T2 | T3 | T3_dur_sec | guard_violations | notes |
|---|---|---|---|---|---|---|---|
| -1.7633 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.7533 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.7433 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.7333 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.7233 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| **-1.7133 (baseline)** | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.7033 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.6933 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.6833 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.6733 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |
| -1.6633 | TODO | TODO | TODO | TODO | TODO | TODO | TODO |

---

## Best value picked

**Yaw value:** TODO rad
**T3 sim score:** TODO (vs baseline -1.7133 = TODO)
**Improvement:** TODO points
**Tiebreaker considerations:** TODO

---

## Recommendation for v27 bake-off

**ENABLE / DISABLE / EXTEND-SWEEP** (Eng-4 picks one)

One sentence why:

TODO — e.g. "ENABLE: yaw=-1.703 produced T3 sim=42 (+8 vs baseline 34),
T1=51 unchanged, no guard violations, sweep peak is unambiguous."

---

## Notes for bake-off engineer

If ENABLE:
- v27 integration commits the new value to the table:
  `("sc", 3): <chosen value>` (replace -1.7133).
- The env-var override mechanism stays in code (harmless when unset);
  it's a useful dev tool for v28+ if more sweeps are needed.
- v27 postmortem should grep eval-returned `policy.log` for
  `event=b2a_yaw_override` — must NOT appear (env var should not be
  set in eval container).

If DISABLE:
- v27 ships v26 unchanged on T3.
- The env-var override mechanism still ships (harmless).
- Note in v27 postmortem that B2a sweep returned no improvement; do
  not retry without new information (e.g. real-HW board yaw measurement).

If EXTEND-SWEEP:
- Slot is too tight for another sweep tonight. Either:
  (a) v27 ships without B2a; B2a re-sweeps in a wider range Day 3.
  (b) Wait until Day 3 to ship B2a at all.
- Lead A and Lead B decide tomorrow morning based on v26 score.

---

## Sim-vs-real-HW caveat

T3's plug-port distance is ultimately a real-HW measurement. Sim's MuJoCo
cable model is rigid axially (Bug 86) — it under-models the force the
gripper-yaw correction needs to overcome. A 0.04m → 0.02m sim improvement
may translate to insertion on real HW; an unchanged sim distance may
still see real-HW improvement if the limiting factor is yaw alignment
not cable tension.

Recommend: even a +2 point T3 sim improvement at one yaw value is worth
shipping if it doesn't regress T1 or T2. The cost is one line of code;
the upside is potentially +20 real-HW points.
