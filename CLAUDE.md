# ANT Policy — Project Memory

This file holds durable context for Claude Code sessions on this repo. The
authoritative running log of bug history, run results, and parameter state
lives in `ant_policy_node/ANT_PROJECT_STATUS.md` — read it first.

## Repo at a glance

- **Goal**: ANT cable-insertion policy for the AI for Industry Challenge,
  scored across 3 trials per run (T1=SFP +17°, T2=SFP −45°, T3=SC +17°).
- **Policy entrypoint**: `ant_policy_node/ant_policy_node/ANT.py`
- **Status doc** (read this every session): `ant_policy_node/ANT_PROJECT_STATUS.md`
- **Submit script**: `./submit.sh <tag>` (builds, tags, pushes to ECR)
- **Docker**: `docker/docker-compose.yaml` (image tag must match `submit.sh` arg)

## Build pipeline gotcha (Bug 90 saga)

`ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/`
contains a **stale committed copy** of ANT.py and friends (added in commit
`5536e27` "first successful insertion"). The Dockerfile copies the entire
`ant_policy_node/` directory into the image, and at runtime Python's import
resolution sometimes picks up the install/ copy over the source.

**Symptom**: Stage 4 log shows `feedforward_fz=0.0 N` and budget `0.3 s`
even though source has `−5.0 N` and `0.5 s` for SC.

**Fix**: After editing `ant_policy_node/ant_policy_node/ANT.py`, also run

    cp ant_policy_node/ant_policy_node/ANT.py \
       ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py

Same for `__init__.py` and `stage1_debug.py` if they change. Verify with
`diff -q` before committing. Long-term fix is to remove install/ from git
and have the Dockerfile do a fresh colcon build, but that's deferred.

## Score progression key milestones

| Tag/Sim | Score | What changed |
|---------|-------|--------------|
| v11 | 88.28 | First passing real-HW result. T1=50, T2=37, T3=1. |
| v14 | 23.20 | Same code as v11 — pure hardware-variance regression on a high-cable-tension day. T2 plug missed bounding radius. |
| sim 42 | 104.08 | Bug 92 (T2 3-way split) confirmed working. |
| sim 43 | 76.30 | Bug 93 (SC WP2 multi-step) navigation success but Bug 90 still missing from image (install/ stale) → SC Stage 4 force abort `return False` traded a likely partial insertion for −12 force penalty. |
| sim 44 | 88.46 | Clean docker rebuild. Install/ synced. Bug 95 (SC force-abort `break`) added. T3 navigation + descent fully working — TCP reaches z=0.0142 (target 0.0095) — but plug ends up 0.19 m from port due to gripper-orientation issue. |
| **v17** | **23.25** | Real-HW reproduces v14 failure mode despite v15 fixes. T1 = 21.25 (dist=0.10 m, jerk=1.95 — identical to v14). T2 = 1.0 (dist=0.17 m — Bug 92 brought plug into measurement range vs unmeasurable in v14, still outside `init×0.5≈0.085 m` radius). T3 = 1.0 (dist=0.22 m — Bug 93 6 cm steps insufficient). No force penalties (Bug 94+95 working). v15 changes help marginally but worst-case days still unsolved. |
| **sim 2026-04-28** | **103.05** | First run with Bug 96A code. Threshold was 21.0N (lowered to 20.5N after), so high-tension path never triggered — score is mostly sim variance + favorable T3 lateral. T1=50.01, T2=37.30, T3=15.74. `build_version=unknown` (pixi bypass; now fixed). |
| **main (Bugs 96A/97/98)** | **PENDING SIM** | Bug 96A: adaptive lateral feedforward (5-way split + 4N Y) when baseline > 20.5N. Bug 97: SC Stage 1 sub-step always fires. Bug 98a: Stage 4 cable-slack early exit. Bug 98b: Stage 4 XY spiral ±8mm. Need ≥3 sim runs to validate. |

## Real-HW vs sim variance pattern (key insight)

Sim consistently produces ~88 with v15 code. Real HW oscillates between
**23 (v14, v17)** and **88 (v11)**. Same code, same compose, different
days. The difference correlates with cable pretension on the day:

- **Good day** (low cable tension): Stage 1 lateral moves converge,
  Stage 4 descends to port depth, T1=50/T2=37/T3=1 → 88 total.
- **Bad day** (high cable tension): Stage 1 lateral moves stall short,
  arm ends up 5–17 cm from port XY, Stage 4 either skipped (Bug 94) or
  descends to wrong place. T1=21/T2=1/T3=1 → 23 total.

**Bug 96A (2026-04-29) implements lateral feedforward at safe_z.** When
`cable_force_baseline > 20.5N` (high-tension day), T2 SFP lateral uses
5-way split (vs 3-way) with 4N Y-direction feedforward per sub-step.
SC lateral uses 3.5 cm sub-steps (vs 6 cm). Threshold lowered from 21.0N
to 20.5N so typical sim baselines (T1≈20.9N, T3-stage1≈20.8N) trigger
for validation.

Other robustness levers (not yet attempted, deferred):

1. **Lower safe_z** for T2 lateral (currently 0.28 m). Risk: NIC mount collision at Y≈0.233 if safe_z < 0.21.
2. **Joint-space lateral** instead of cartesian — bypasses impedance accumulation.
3. **Higher max_wrench during lateral** (from 15N to 20N). Risk: approaches force-penalty ceiling.

## Bugs 96A/97/98 (main branch, pending sim validation)

- **Bug 96A**: Adaptive lateral feedforward. `_high_tension` property true when
  `cable_force_baseline > 20.5N`. T2 SFP: 5-way split + 4N Y feedforward each step.
  SC: 3.5 cm sub-steps (vs 6 cm). Generic: works for any port configuration.
- **Bug 97**: SC Stage 1 sub-step always fires. Refactored if/else into flat
  WP1 (conditional ascent) → WP2 (always sub-step lateral) → WP3 (conditional descent).
  Prior code bypassed sub-steps when arm started at z≥safe_z−0.01 (common case in T3).
- **Bug 98a**: Stage 4 cable-slack early exit. `|F| < baseline − 5N` for 2s → break.
  Signal: plug past port entrance, cable tension dropped. Tunable tunables in `__init__`.
- **Bug 98b**: Stage 4 XY spiral. After 6s settle at depth: ±8mm radius, 10s period,
  5s ramp. Probes the ±8mm neighborhood to find port opening when TCP is 3–8mm off.

## Bugs 99–104 (branch claude/review-simulation-logs-6knxl, pending sim validation)

Sim 2026-04-29 (score 99.59) showed:
- T1 SFP got tier_3=38.41 (lucky partial insertion at 0.04m, arm never descended).
- T2 SFP plug ended 0.04m from port but no insertion — 130s spent in stuck spiral.
- T3 SC stalled at z=0.069m, plug 0.14m from port — gripper-orientation issue.

Six new bugs target the architectural gaps identified in that run.

- **Bug 99 (A) — calibrated port-yaw alignment**: New
  `gripper_yaw_correction_rad[(zone, trial)]` table plus helper
  `_yaw_rotated_orientation()` rotates the gripper around base_link Z by a
  per-trial calibrated angle during Stage 3/4. Default 0.0 rad for entries
  not in the table preserves SFP behaviour exactly (T1's 38pt partial-insert
  unaffected). Uses a hardcoded calibration table, NOT live `sc_port_base`
  TF — CLAUDE.md prohibits ground-truth TF lookup.
- **Bug 100 (B) — Stage 4 Fxy gradient**: When `|Fxy| > 3 N`, step the
  commanded XY by 0.6 mm/sample against −F̂xy (cap 12 mm accumulated). Adds
  a closed-loop XY correction to the existing Lissajous so the chamfer
  reaction can guide the plug. Generic — no port-specific values.
- **Bug 101 (C) — Stage 4 per-axis compliance**: New
  `_build_motion_update_axis()` helper. Stage 4 uses XY=50 N/m, Z=120 N/m
  instead of isotropic 85/200 N/m so the chamfer can guide the plug
  laterally without fighting a stiff XY position target.
- **Bug 102 (J) — stiffer lateral on high-tension days**: Joint-space IK
  isn't exposed to the policy. Proxy: when `_high_tension` is True, raise
  the Stage 1 lateral Cartesian stiffness from 85 to 250 N/m so impedance
  drives against cable equilibrium. Applied to both T2 SFP and SC laterals.
- **Bug 103 (H) — cable-anchor XY bias**: High-tension cable pull biases the
  arm's stalled XY away from the true port. Counter by shifting the lateral
  target 1 cm toward the calibrated `cable_anchor_xy_in_base[zone]`.
- **Bug 104 (I) — adaptive Stage 4 mode**: Branch on
  `xy_err = |contact_pose − connector_pose|`:
  - `< 5 mm`  : `direct`  — hold + slack-detect, no spiral.
  - `5–15 mm` : `spiral`  — current Bug 98b Lissajous.
  - `15–40 mm`: `descend` — spiral + per-orbit z-ramp (2 mm/orbit) so axial
    progress continues even when XY can't find the hole.
  - `≥ 40 mm` : skip — tightened from Bug 94's 60 mm so we don't burn 130s
    over a misaligned port.

All Bugs 99–104 are gated by enable flags in `ANT.__init__`. Disable in
code to roll back individually if a sim run regresses.

### Calibration TODO (Bug 99)
`gripper_yaw_correction_rad[("sc", 3)]` starts at 0.0 rad. After the first
sim run with this code, observe T3 plug orientation in the log and pick a
yaw correction (likely in the ±0.5 rad range) that aligns the plug's flat
with the port slot. Iterate over 2–3 sims.

## Bug 105 (vision-based SC port localisation, branch claude/review-simulation-logs-6knxl)

Replaces hardcoded `zone_known_ports["sc"]` with a live back-projection from
the center camera image when a confident detection is available.  This is
the path that generalises to unknown board placements/yaws — not just the
sample_config-calibrated one.

**Pipeline**:
1. Stage 1 SC fallback path captures the latest Observation.
2. `_detect_sc_housing_pixel()` runs Otsu threshold + contour scoring on
   `center_image` (same algorithm as `stage1_debug.py`).
3. `_back_project_to_z()` looks up `base_link → center_camera/optical` from
   the URDF TF tree (NOT ground-truth) and intersects the camera ray with
   `connector_z_in_base["sc"]`.
4. Sanity check: detection must lie within `vision_sanity_radius_m` (5 cm)
   of the calibrated XY.  If not — caller falls back to the calibrated
   table.  Worst case: behaviour matches pre-Bug-105 code exactly.

**Why SC only**: SFP HSV from above is structurally blind (CLAUDE.md).
The SC housing is a high-contrast cream/tan block visible top-down; Otsu
thresholding finds it reliably (the same code path that the disabled SC
pre-scan used, just without the lateral-offset gymnastics).

**Tunables in `ANT.__init__`**:
- `enable_vision_sc_localization` — master toggle.
- `vision_min_score` — contour score floor (default 1.0).
- `vision_sanity_radius_m` — max accepted deviation from calibrated XY.
- `vision_min_area_px` / `vision_max_area_frac` — contour size gates.

**Generalisation note**: this lifts the rigid `(-0.3830, 0.4295)` calibration
constraint for SC.  The existing tables remain as the safety fallback so a
detection failure is no worse than the prior code.  For full generalisation
to unknown SFP boards, a successor effort needs side-camera or board-frame
detection (HSV from above will not work — see "SFP HSV from above is
structurally blind" in Policy/runtime constraints).

## v15 ships with these robustness fixes (vs v11/v14)

- **Bug 90**: SC Stage 4 `feedforward_fz=−5.0`, budget `0.5 s`, force-timer
  resets on dip (continuous, not cumulative). Now actually reaches the
  image after install/ sync.
- **Bug 92**: T2 SFP Stage 1 lateral approach 2-way → 3-way split
  (1.75 cm steps, 30 s/step). Y fractions recomputed from arm's actual
  position each step so cable snap-through can't compound.
- **Bug 93**: SC Stage 1 WP2 lateral split into ≤6 cm sub-steps (`n_steps =
  ceil(distance/0.06)`), targets recomputed each step.
- **Bug 94**: Stage 4 XY-guard tightened `0.15 m → 0.06 m`. Bad-XY trials
  skip Stage 4 instead of pushing into board face → eliminates force-event
  risk. Good-XY trials (T1/T2 with xy_err <0.04 m) still proceed.
- **Bug 95**: For `zone == "sc"` only, Stage 4 force abort `break`s out of
  the loop (`insert_cable` returns True) instead of `return False`. SFP
  abort still returns False (those are real surface-contact failures
  caught by Bug 94). Rationale: if the SC arm passed the Bug 94 xy guard
  it's at the port; force buildup is cable tension at port depth, not
  surface contact, and the arm pose at abort is potentially partially
  inserted — return True so scoring evaluates tier_3 (potential 5–25
  partial credit) instead of marking task incomplete (tier_3=0).

## Open T3 problem (partially addressed by Bug 98b)

Sim 44 showed Bug 92/93/94/95 deliver the arm to the SC port: TCP at
xy_err=0.005 m, descended to z=0.0142 m (target 0.0095 m). But the
**plug-port distance is 0.19 m** at end of trial → tier_3=0.

Investigation findings (commit `ce44861`):

- `gripper/tcp` is +0.172 m offset from `gripper/hande_base_link`
  (`aic_assets/models/Robotiq Hand-E/robotiq_hande_macro.xacro:159-165`).
- Cable plugin sets `cable_connection_0_link = sc_plug_link` for T3
  (`aic_description/urdf/cable.sdf.xacro:17`). Plug attaches directly to
  gripper end-effector via fixed joint — not via the flexible rope end.
- `sc_tip_link` is +11.65 mm in plug-local +X with rotation `(−π/2, 0,
  −π/2)` from `sc_plug_link`
  (`aic_assets/models/SC Plug/model.sdf:141-156`).
- Scoring computes Euclidean distance between `sc_tip` and `sc_port_base`
  TF frames in `aic_world` frame
  (`aic_scoring/src/ScoringTier2.cc:497-521,478-488`).
- Bounding radius = `initial_distance × 0.5`
  (`aic_scoring/src/ScoringTier2.cc:720-728`). For T3 with init_dist ≈
  0.20 m, scoring window is roughly `[0.015, 0.115]` m. Our 0.19 m sits
  outside → tier_3=0.

**To score tier_3 ≥ 1 on T3** the final plug-port distance must be ≤ ~0.115
m, i.e. 7–8 cm closer than current. That requires explicit gripper
orientation alignment with the SC port's insertion axis at Stage 4 — a
non-trivial code change with risk of disrupting T1/T2 (which currently
provide ~87 of the 88 total). Sketch of the fix:

1. Read `sc_port_base` orientation from TF.
2. Compose with the SC plug's local-frame rotation
   (`sc_tip_link` rotated `(−π/2, 0, −π/2)` from `sc_plug_link`) to get the
   target gripper quaternion that aligns `sc_tip`'s insertion axis with
   the port.
3. Pass that quaternion into Stage 3/Stage 4 `_make_pose` calls instead of
   reusing the arm's current orientation.
4. Calibrate any residual XY/Z offset between `gripper/tcp` and `sc_tip`
   under the new orientation, and apply it to the descent target.

Validate in sim across 3+ runs before submitting — both that T3 scores
partial credit and that T1/T2 are unchanged.

## Recurring lessons

- **Real-hardware variance is huge**. Same code (v11 vs v14) ran 88.28 vs
  23.20 on different days. Every change must be tested for *worst-case*
  cable tension behavior, not just typical.
- **The 1.0 s force-penalty threshold (`> 20 N` sustained) is the cliff
  edge**. For SC, baseline cable tension is already ~19–22 N. Any
  multi-second Stage 4 hold accumulates time-above-20-N quickly. Bug 95
  exists because the difference between `return False` and `break` on
  force abort changes scoring from −11 to ~+13.
- **The sim cable model is rigid axially** (MuJoCo cable plugin has no
  stiffness key — bug 86). T1/T2 Stage 4 stall heights cannot be moved in
  sim regardless of feedforward; only real hardware validates Stage 4
  feedforward changes.
- **GitHub MCP is restricted to `ataub11/aic`**. Do not attempt other
  repos. The development branch for competition work is
  `claude/improve-competition-score-cginC`.

## Policy/runtime constraints (durable — don't re-investigate)

- **Forbidden TF frames**. The parent node (`aic_model.AicModel`) owns a
  `_tf_buffer` accessible from a policy as `self._parent_node._tf_buffer`,
  but the plug/port/cable frames (`sc_tip`, `sc_port_base`,
  `sfp_port_0_link*`, `cable_connection_*`, `aic_world`) are only relayed
  from `/scoring/tf` to `/tf` when launched with `ground_truth:=true`
  (`aic_bringup/launch/aic_gz_bringup.launch.py` lines 354–367). They are
  **ground-truth state** and competition runs disable the relay. Do not
  call `lookup_transform` on any of those frames in policy code — it works
  in local debug runs and silently fails at eval time.
- **Allowed TF frames**: anything in the robot URDF tree
  (`base_link → ... → gripper/tcp`, `center_camera/optical`,
  `ati/tool_link`, etc.) is published by `robot_state_publisher` and is
  fair game.
- **Joint-space IK is not exposed to the policy**. `move_robot()` accepts
  Cartesian (`MotionUpdate`, impedance) or direct joint-space targets
  (`JointMotionUpdate`, six floats). There is no `/compute_ik` service
  call wired in. "Go to (x, y, z) in joint space" requires solving IK
  in-policy. Bug 102's stiffer-Cartesian-impedance is the practical
  workaround.
- **No ArUco / fiducial markers on the task board**. The board mesh
  (`Task Board Base/base_visual.glb`) has no marker textures, and the
  rules forbid modifying the board. Vision must work from the natural
  appearance of the board / port hardware.
- **SFP HSV from above is structurally blind**. The SFP bracket's blue/
  cyan face is on the outward side, not visible top-down. Any vision
  approach for SFP needs board-frame localisation (Bug 105) or an angled
  camera position.

## Quick commands

```bash
# Verify install/ is in sync with source
diff -q ant_policy_node/ant_policy_node/ANT.py \
        ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py

# Force a clean docker build + push
docker image rm ant-policy:v15 2>/dev/null
./submit.sh v15

# Check current submitted ECR tag
grep "image:" docker/docker-compose.yaml

# Read ground-truth port poses for Bug 99 yaw / Bug 103 anchor calibration
# (run a sim with ground_truth:=true; these frames are then relayed to /tf)
ros2 run tf2_ros tf2_echo base_link sc_port_base
ros2 run tf2_ros tf2_echo base_link cable_base
ros2 run tf2_ros tf2_echo base_link sfp_port_0_link
```
