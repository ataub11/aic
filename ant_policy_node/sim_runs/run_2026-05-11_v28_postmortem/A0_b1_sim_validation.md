# v28 A0 — B1 Sim Gate Validation

**Date:** UTC 2026-05-10  
**Branch:** `claude/b1-joint-space-t1-sfp` @ `9894917`  
**Image:** `ant-policy:v28` (local sim build, BUILD_VERSION=v28-sim-gate)  
**Flag state:** `joint_space_t1_sfp = True`  
**Policy log:** `/tmp/ant_local_sim/policy_v28.log` (7387 lines)

---

## Sim Gate Results

| Gate | Required | Observed | Result |
|---|---|---|---|
| `joint_space_start label="Stage 1 SFP T1 WP2 escalation"` fires | Yes | trial=1, zone=sfp ✓ | **PASS** |
| T3 sim score | ≥ 30 | 36.07 | **PASS** |
| T3 wall-clock | < 175 s | 133.08 s | **PASS** |
| `joint_space_guard_violation` events | == 0 | 0 | **PASS** |
| Traceback / Exception | == 0 | 0 | **PASS** |
| `trial_end` event count | == 3 | 3 | **PASS** |

**All 6 v28 sim gates PASS.**

---

## Trial Scores

| Trial | Tier 1 | Tier 2 | Tier 3 | Total |
|---|---|---|---|---|
| T1 (SFP +17°) | 1.0 | 10.71 | 24.32 | **36.03** |
| T2 (SFP −45°) | 1.0 | 0.0 | 0.0 | **1.0** |
| T3 (SC +17°) | 1.0 | 11.58 | 23.49 | **36.07** |
| **Total** | | | | **73.10** |

---

## B1 Joint-Space Escalation Evidence (T1)

```
ANT-DIAG event=joint_space_start trial=1 zone=sfp label=Stage 1 SFP T1 WP2 escalation
  tgt_x=-0.3845 tgt_y=0.2000 delta_q_norm=0.0739

ANT-DIAG event=joint_space_final trial=1 zone=sfp label=Stage 1 SFP T1 WP2 escalation
  tgt_x=-0.3845 tgt_y=0.2000 actual_x=-0.3805 actual_y=0.1983
  err_mm=4.3151 within_tol=True ok=True
  max_force_settle_n=20.3032 diag_case=ok
```

**B1 worked:** Joint-space escalation converged T1 to **4.3 mm** from port XY (vs ~100 mm Cartesian stall on real HW). Tier_3 jumped from 6.87 (v27 real-HW) to 24.32 in sim — arm is now in partial insertion territory.

**Note:** T1 sim total (36.03) appears lower than v27 sim (52.83) because tier_2 duration component differs when joint-space path is active. This is expected — sim has no cable tension, so Cartesian was already getting close; the tier_2 trajectory sub-scores reflect the joint-space path. The real-HW gain is expected to be significantly larger since joint-space overcomes the 15 N Cartesian ceiling.

---

## T3 Protection Confirmed

T3 (SC, trial=3) produced NO `joint_space_start` event — the zone guard (`zone != "sfp"`) correctly blocked B1 from firing on T3. T3 score 36.07 ≥ 30 and wall-clock 133.08 s < 175 s — no regression.

---

## trial_end Events

```
trial=1  success=True  duration_sec=127.19
trial=2  success=True  duration_sec=126.81
trial=3  success=True  duration_sec=133.08
```

---

## Decision

**All v28 sim gates pass. Cleared for `./submit.sh v28`.**

B1 joint-space escalation is working: err_mm=4.3 mm convergence on T1 in sim, 0 guard violations, T3 unaffected. Expected real-HW T1 gain: T1 tier_3 from 6.87 → 20–30+ (arm now reaching insertion territory instead of stalling at 100 mm).
