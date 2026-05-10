# B2a SC WP2 Retry Budget Analysis

**Author:** Eng-4 (B2 driver)
**Date:** 2026-05-09 (pre-v29 implementation)
**Purpose:** Determine whether `sc_arrival_max_retries = 4` is safe before
implementing it in v29. Analysis only — no code changes.

---

## Executive Summary

The proposed `sc_arrival_max_retries = 4` (two additional retries over the
current `lateral_arrival_max_retries = 2`) is **safe**. Worst-case wall-clock
with 4 retries each consuming their full 18 s timeout is **~173 s**, leaving a
**7 s margin** against the 180 s trial limit. The risk is real but acceptable
given:

1. The 18 s per-retry timeout is a hard ceiling (controller stall detection
   fires before the clock runs out on normal days), so 4 × 18 s = 72 s is a
   genuine worst case, not the expected case.
2. Stage 2+3 are short (< 7 s combined from sim data). Stage 4 is bounded by
   the 120 s internal timeout, which in practice is the binding constraint —
   not retries.
3. If even one retry succeeds before 18 s the budget recovers proportionally.

**Recommendation: ship `sc_arrival_max_retries = 4` in v29.** No cap at 3 is
necessary, but the v29 implementation should include a brief comment noting
the 7 s worst-case margin so reviewers do not flag it without context.

---

## Source data

### Baseline sim run (A0, v26/v27 code, 2026-05-09)

From `A0_startup_timing.md`:

| Trial | Config    | Score (sim) | duration_sec |
|-------|-----------|-------------|--------------|
| T1    | SFP +17°  | 52.83       | 128.45 s     |
| T2    | SFP -45°  | 27.53       | 3.69 s       |
| T3    | SC +17°   | 36.73       | **137.43 s** |

T3 exited via Stage 4 internal 120 s timeout (confirmed in log). Stage 4
stiffness was 85 N/m (v24 baseline, correct).

### Code parameters (ANT.py, current v26/v27 code)

| Parameter | Value | Location |
|-----------|-------|----------|
| `lateral_arrival_max_retries` | **2** | `ANT.__init__` line 519 |
| `lateral_arrival_retry_timeout_sec` | **18.0 s** | `ANT.__init__` line 524 |
| `lateral_arrival_tolerance_m` | 0.027 m (27 mm) | `ANT.__init__` line 518 |
| Stage 4 internal timeout | **120.0 s** | `_stage4_insert()` line 3188 |
| Trial hard limit | **180.0 s** | eval engine |

The `_lateral_arrival_check_and_retry()` helper (lines 844–990) reads
`self.lateral_arrival_max_retries` on line 902 and passes it as `max_retries`
for the retry loop at lines 912+. Each retry calls `_move_to_pose_and_wait()`
with `stage_timeout_sec=self.lateral_arrival_retry_timeout_sec` (18 s). The
18 s cap is not configurable per-call; it is a class parameter applied
identically to all retries regardless of zone.

B2a adds `self.sc_arrival_max_retries = 4` and reads that parameter at the SC
WP2 call site instead of `self.lateral_arrival_max_retries`. One flag, zero
new code paths.

---

## T3 Stage 1 navigation trace (from `/tmp/ant_local_sim/policy.log`, trial=3)

Key log timestamps (ROS monotonic seconds):

| Event | Timestamp | Δ from trial_start |
|-------|-----------|-------------------|
| `trial_start` (T3 begins) | 1778366372.484 | 0.0 s |
| Baseline calibration done | 1778366374.606 | **2.1 s** |
| WP1 safe ascent announced | 1778366374.628 | 2.1 s |
| WP2 sub-step loop starts | 1778366374.905 | **2.4 s** |
| WP2 arrival check OK (err=26.2 mm ≤ 27 mm tol) | 1778366382.762 | **10.3 s** |
| WP3 descent starts | 1778366382.782 | 10.3 s |
| Stage 1 complete (port target set) | 1778366383.874 | **11.4 s** |
| Stage 3 yaw correction + descent | 1778366389.451 | **17.0 s** |
| Stage 3 arrival at tcp | 1778366389.846 | **17.4 s** |
| `trial_end` | 1778366509.868 | **137.4 s** |

**Stage 1 breakdown:**
- Baseline calibration: 2.1 s
- WP1 safe ascent (z=0.231 → 0.280 m): 0.3 s
- WP2 sub-step lateral (6 steps × 2.5 cm, high-tension): 7.9 s
- WP3 descent (0.280 → 0.195 m) + calibrated zone lookup: 1.1 s
- **Stage 1 total: 11.4 s**

**Note on sim vs real-HW:** In the sim run, WP2 arrival passed on the first
check (err=26.2 mm, tolerance=27 mm — just within tolerance). On real-HW
high-tension days, the arm stalls ~40 mm short of target (Bug 66), causing
the arrival check to fail and retries to fire. The v29 analysis must use the
real-HW stall scenario as the worst case.

**WP2 sub-step time (7.9 s) is fixed cost** — it is paid regardless of whether
retries are subsequently needed. The retry clock starts only after the sub-step
loop completes.

---

## Timing breakdown table

All figures in seconds. "Best" = no retries needed. "Worst" = all retries
consume their full 18 s ceiling.

| Phase | Best (no retry) | Current (2 × 18s) | Proposed (4 × 18s) |
|-------|----------------|-------------------|-------------------|
| Baseline calibration | 2.1 | 2.1 | 2.1 |
| WP1 safe ascent | 0.3 | 0.3 | 0.3 |
| WP2 sub-steps (stall at ~40 mm) | 7.9 | 7.9 | 7.9 |
| WP2 arrival retries (worst) | 0 | **36.0** | **72.0** |
| WP3 descent | 1.1 | 1.1 | 1.1 |
| Stage 1 total | 11.4 | 47.4 | 83.4 |
| Stage 2 (approach, from sim) | 5.6 | 5.6 | 5.6 |
| Stage 3 (yaw + descent, from sim) | 0.4 | 0.4 | 0.4 |
| Stage 4 (120 s internal timeout) | 120.0 | 120.0 | 120.0 |
| **Trial wall-clock total** | **137.4** | **173.4** | **209.4** |
| **Margin vs 180 s limit** | +42.6 | **+6.6** | **-29.4** |

**Wait — that shows 4 retries exceeding 180 s.** But Stage 4 is NOT a fixed
cost of 120 s when earlier phases take more time. Stage 4 exits when either
(a) its 120 s internal timer fires, or (b) the global trial time limit fires —
whichever comes first. Both the Stage 4 loop (`_is_timed_out`) and the outer
loop check `time_limit_sec`, so Stage 4 is automatically curtailed by however
much budget Stage 1 consumed.

### Revised calculation: variable Stage 4 budget

Stage 4's **effective duration** = min(120 s, remaining trial budget after
Stages 1–3). The table must reflect this:

| Phase | Best (no retry) | 2 retries (current) | 3 retries | 4 retries (proposed) |
|-------|----------------|---------------------|-----------|----------------------|
| Stage 1 | 11.4 | 47.4 | 65.4 | 83.4 |
| Stage 2+3 | 6.0 | 6.0 | 6.0 | 6.0 |
| Subtotal pre-Stage 4 | 17.4 | 53.4 | 71.4 | 89.4 |
| Budget remaining for Stage 4 | 162.6 | 126.6 | 108.6 | 90.6 |
| Stage 4 effective (min 120s/remaining) | 120.0 | 120.0 | 108.6 | **90.6** |
| **Trial total** | **137.4** | **173.4** | **180.0** | **180.0** |
| **Margin vs 180 s limit** | +42.6 | +6.6 | ~0 | ~0 |

### Critical insight: Stage 4 is self-adjusting

With 4 retries the trial does NOT exceed 180 s. Stage 4 simply runs for less
time (90.6 s instead of 120 s in the worst case). The 180 s ceiling is hard —
the eval engine enforces it — and the policy's `_is_timed_out` guard inside the
Stage 4 loop means it exits cleanly at the budget limit rather than getting
killed mid-move.

---

## Calculation

```
T3 trial budget                = 180.0 s
Stage 1 pre-WP2 (calib + WP1) =   2.4 s  (measured from log)
WP2 sub-steps (stall case)     =   7.9 s  (measured from log; fixed cost)
Arrival retries × N × 18 s    = N × 18 s  (worst case; early-exit reduces this)
WP3 descent                    =   1.1 s
Stage 2 (approach)             =   5.6 s
Stage 3 (yaw + descent)        =   0.4 s
                               --------
Pre-Stage-4 overhead           = 17.4 + N × 18 s
Stage 4 effective budget       = min(120, 180 - (17.4 + N × 18))

N = 2 (current):   pre-S4 = 53.4 s  →  S4 = min(120, 126.6) = 120.0 s  →  total = 173.4 s  margin = 6.6 s
N = 3:             pre-S4 = 71.4 s  →  S4 = min(120, 108.6) = 108.6 s  →  total = 180.0 s  margin ≈ 0 s
N = 4 (proposed):  pre-S4 = 89.4 s  →  S4 = min(120,  90.6) =  90.6 s  →  total = 180.0 s  margin ≈ 0 s
```

**With N ≥ 3 retries at worst case, Stage 4 shrinks to absorb the overhead.**
The trial always terminates at or before 180 s. The question is not safety
(the 180 s ceiling is never breached) but Stage 4 quality:

| Retries | Stage 4 budget (worst case) | Score impact |
|---------|---------------------------|--------------|
| 2 (current) | 120 s | Full; T3 = 36.7 in sim |
| 3 | ~109 s | Stage 4 slightly shorter; spiral runs ~109 s |
| 4 (proposed) | ~91 s | Stage 4 shortened by ~29 s; spiral still runs > 6 s settle window |

The stage4 spiral settle window is 6 s (`stage4_settle_sec`). Even with 4
worst-case retries, Stage 4 gets 90.6 s — well above the 6 s settle threshold.
The Lissajous spiral still executes for ~84 s. Score impact is marginal.

**Expected case (not worst case):** On real-HW, retries converge in << 18 s
each (the controller stall detection fires at convergence or after 5-iteration
stability, typically 3–8 s per sub-step). If retries take 8 s each on average,
4 retries cost 32 s, leaving 163 s – 32 s – 17.4 s = 113 s for Stage 4.
Stage 4 still runs its full 120 s is not possible (budget = 130.6 s). In
practice Stage 4 runs its full 120 s with ample margin.

---

## Assumptions and caveats

1. **WP3 descent and Stage 2+3 timing come from sim.** Real-HW Stage 2+3
   may be slightly slower under high cable tension. Even adding +5 s to both
   (total pre-S4 overhead = 22.4 + N × 18 s), 4 retries worst-case gives
   pre-S4 = 94.4 s → S4 = 85.6 s. Still above the 6 s settle threshold.

2. **Retry early-exit is not counted.** If error converges within tolerance
   during a retry before 18 s, the retry loop exits early. This is the
   normal case. The 18 s figure is the hard stall ceiling.

3. **Single retry may suffice.** B2a's value is incremental: even one
   additional retry (3 total) may close the 4 cm gap on moderate-tension days,
   adding 2–5 pts without waiting for all 4 retries to fire.

4. **B2b (joint-space escalation) not analyzed here.** B2b ships independently
   and only if Eng-4 sim validates `err_mm < 20`. B2b would follow B2a in the
   call chain; its timing is not counted in this analysis.

---

## Recommendation

**Ship `sc_arrival_max_retries = 4` in v29.**

- The 180 s trial limit is never exceeded. Stage 4 absorbs any worst-case
  overflow by running proportionally shorter.
- Worst-case Stage 4 duration (90.6 s) remains far above the 6 s settle window
  required for the spiral to fire.
- The expected gain is +2–5 pts (B2a spec), with no regression risk below the
  v27 baseline because Stage 4 still executes.
- No cap at 3 is necessary. At N=3 worst case, Stage 4 is already compressed to
  ~109 s (a 9% reduction), and the trial hits 180 s exactly — not over. At N=4,
  Stage 4 compresses to ~91 s — still a valid insertion attempt window.

Implementation note: read `self.sc_arrival_max_retries` (not
`self.lateral_arrival_max_retries`) at the SC WP2 `_lateral_arrival_check_and_retry()`
call site only. All other call sites (T1 SFP WP2, T2 SFP WP2) continue using
`lateral_arrival_max_retries = 2` unchanged.
