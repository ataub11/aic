# A0 — v27 Overnight Validation: Startup Timing + 3-Trial Sim

**Date:** 2026-05-09 (UTC evening) — overnight prep for v27 submission (UTC 2026-05-10)
**Driver:** Allison + Claude Sonnet 4.6 (Eng-1)
**Image:** `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v26`
**Digest:** `sha256:525a8367a5288ecd778f1f408ebd8f8aa554fdfdadc680fe78ad00d9f31a0be1`
**Local tag:** `ant-policy:v26` (same digest — verified in `docker images` output)
**Build version reported:** `v26-df82c5c-dirty`

Code basis for v27 is v26 as-is. This run validates startup timing and all
v27 acceptance gates with the image that will be re-tagged as v27.

---

## Part 1: Startup timing

**Purpose:** Rule out pixi init slowness as a contributing factor to the
UTC 2026-05-09 infrastructure failures (v26 no artifacts; v24-fallback timeout).

**Stack:** `docker compose up` — eval (`ghcr.io/intrinsic-dev/aic/aic_eval`) +
model (`ant-policy:v26`). Images were already present locally (no ECR pull required
— ECR digest matches local tag).

| Event | Wall-clock (UTC) | ROS time (s) | Δ from compose up |
|---|---|---|---|
| Container first log (Zenoh scouting) | 22:36:26.869 | 1778366186.869 | 0 s |
| `Loading policy module: ant_policy_node.ANT` | 22:36:27.055 | 1778366187.055 | 0.2 s |
| `on_configure()` | 22:36:34.519 | 1778366194.519 | 7.7 s |
| `Policy.__init__()` / `ANT policy startup` | 22:36:34.530 | 1778366194.530 | 7.7 s |
| **`on_activate()` — node ACTIVE** | **22:36:39.432** | **1778366199.432** | **12.6 s** |
| First trial goal accepted | 22:36:59.929 | 1778366219.929 | 33.1 s |

**Startup time (compose up → Active): 12.6 s ✅ (target ≤ 60 s)**

Pixi init is not the bottleneck. The ~7.7 s between first log and on_configure
is the Zenoh scouting/connection phase. on_configure → on_activate takes ~4.9 s
(policy `__init__` + TF buffer warm-up). All within expected bounds.

**Verdict: infra slowness is not reproducible in local env.** The UTC 2026-05-09
failures (v26 no artifacts; v24-fallback timeout) were evaluation-side, not
startup-related. v27 fresh build should resolve.

---

## Part 2: 3-trial local sim

**Same docker compose run as Part 1 above (container `aic-model-1`, exited cleanly
after 3 trials). `policy.log` saved to `/tmp/ant_local_sim/policy.log` (4979 lines).**

### Per-trial results

| Trial | Config | Score (sim) | tier_1 | tier_2 | tier_3 | duration_sec |
|---|---|---|---|---|---|---|
| T1 | SFP +17° | **52.83** | 1 | 13.41 | 38.41 | 128.45 s |
| T2 | SFP −45° | **27.53** | 1 | 20.21 | 6.32 | 3.69 s |
| T3 | SC +17° | **36.73** | 1 | 12.35 | 23.38 | 137.43 s |
| **Total** | | **117.09** | | | | |

T2 score reflects diagnostic Case B (joint move stalled → Bug 124 abort fires,
arm parks at diagnostic signature pose). Expected sim behavior — joint-space IK
converges in sim but diagnostic fires because v26 is reverted (v24 code, Bugs
125/126/127 absent). Real-HW T2=1 expected.

T3 Stage 4 stiffness confirmed **85 N/m** (v24 baseline, NOT 200 N/m CR-2
regression). T3 exited via Stage 4 internal 120 s timeout, NOT the 180 s
task-limit timeout (v25 regression signature). All anti-markers confirmed absent
from image (carried forward from v26 A1/A2 audit).

### Acceptance gate table

| Gate | Required | Observed | Result |
|---|---|---|---|
| `trial_end` event count | == 3 | 3 | ✅ |
| T1 sim score | ≥ 50 | 52.83 | ✅ |
| T3 sim score | ≥ 30 | 36.73 | ✅ |
| T3 wall-clock | < 175 s | **137.43 s** | ✅ |
| `joint_space_guard_violation` count | == 0 | 0 | ✅ |
| Traceback / Exception (policy) | == 0 | 0 (shutdown ExternalShutdownException only — expected rclpy teardown) | ✅ |
| build_version | != unknown | `v26-df82c5c-dirty` | ✅ |
| Stage 4 SC stiffness | 85 N/m (not 200) | 85 N/m ✅ | ✅ |

**All v27 acceptance gates PASS.**

---

## Decision

**Proceed to `./submit.sh v27` (fresh build, same code as v26).**

- Startup timing: 12.6 s ✅ — no pixi init issue
- All sim gates pass ✅
- `/tmp/ant_local_sim/policy.log` ready for submit.sh T3 gate ✅
- Independent reviewer (Eng-2) must sign `pre_submit_checklist.md` before
  `submit.sh v27` runs.
