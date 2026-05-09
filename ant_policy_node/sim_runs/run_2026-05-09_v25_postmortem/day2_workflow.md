# Day-2 Morning Workflow (UTC 2026-05-10 = v27 slot)

**Window:** PST 2026-05-09 17:00 → 2026-05-10 16:59 (UTC 2026-05-10 00:00 → 23:59).
**Hard portal deadline:** PST 2026-05-10 16:59 = UTC 2026-05-10 23:59.
**Self-imposed soft deadline:** PST 2026-05-10 14:00 (3-hour safety margin).

This doc resolves the team-review action item: *"Day-2 morning workflow:
read v26 score → fill v26 postmortem → build v27. Gate v27 build on v26
postmortem being TODO-free."* Read top-to-bottom in order.

---

## 0. Prerequisites (verify before starting)

- [ ] v26 portal submission registered last night (PST 2026-05-09 evening,
      UTC 2026-05-09 window).  Confirm in AIC portal.
- [ ] v26 score artifacts pulled and committed to
      `ant_policy_node/sim_runs/run_2026-05-09_v26_submission_<N>/`:
      score JSON, summary TXT, image-URI JSON, and any returned
      `policy.log` / `ant_diagnostics.jsonl` / `hw_variance.jsonl`.
- [ ] Independent reviewer assigned for the day (NOT the same person who
      drove yesterday's build).  See pairing rule in CLAUDE.md §plan.

If any prerequisite is unmet, halt and surface to Lead A/Lead B before
touching any code.

---

## 1. Read v26 result (≤ 30 min)

1. Open `score.json` and record T1/T2/T3 sub-tier breakdown.
2. Compare against expected band: floor 23, baseline 64, realistic 86.
3. If a `policy.log` or `ant_diagnostics.jsonl` was returned, grep for:
   - `event=trial_end ... duration_sec=` — confirms timing per trial.
   - `event=trial_end ... c1_code=` — C1 failure-code per trial.
   - `event=joint_space_guard_violation` — should be ZERO if zone guard
     is doing its job.
   - `event=baseline ... high_tension=true|false` — HW-day type.
4. If `hw_variance.jsonl` is present, append a row to
   `~/aic_results/hw_variance_history.csv` (or equivalent) so cross-day
   drift is graphable.

## 2. Fill v26 postmortem (≤ 30 min)

Edit `ant_policy_node/sim_runs/run_2026-05-09_v26_postmortem/postmortem_v26.md`
(template was auto-created last night by `submit.sh`).

Replace **every** `TODO` with concrete content:
- **Score:** actual T1/T2/T3 + total.
- **vs prior:** delta vs v22-v24 baseline; delta vs v25 (regression
  recovery is the v26 KPI).
- **Regression:** if any trial regressed vs v22/v23/v24, name the trial
  and the magnitude.  If no regression, write "none".
- **What happened:** narrative tying score to expected outcome (revert
  hypothesis confirmed / falsified).
- **Process improvement:** one sentence on what the v26 retrospective
  tells us we should do differently for v27.

`submit.sh` for v27 will SOFT-WARN if `postmortem_v26.md` still contains
"TODO".  Don't ship until it's clean.

## 3. Decide v27 candidate (≤ 15 min, by 09:00 PST)

| v26 outcome | v27 candidate | Rationale |
|---|---|---|
| ≥ 64 (baseline holds) | v27 = v26 + B1 + B2a + B2b (all flags default-off) | Plan §"May 10 — v27" |
| 60-63 (marginal) | v27 = v26 + B2a only (T3 yaw recal) | Conservative — single-lever change |
| < 60 (regression) | v27 = `ant:v24-fallback` re-tag, calibration shot | Plan D-track |
| ≥ 95 (huge upside) | v27 = re-submit `ant:v26` for variance check | Plan §"three readings beat one explanation" |

Decision must be written into the day-2 standup notes before 09:00 PST.

## 4. Run local sim of v27 candidate (~ 90 min)

```bash
# Wherever the local sim launcher lives (scripts/local_sim.sh or pixi run …):
./scripts/local_sim.sh                   # writes /tmp/ant_local_sim/policy.log
```

Acceptance gates (must all pass before submit):
- 3× `event=trial_end` events.
- T3 wall-clock < 110 s.
- T1 sim score ≥ 50.
- T3 sim score ≥ 30 (no T3 regression vs v26).
- Zero `joint_space_guard_violation` events.
- No `Traceback` / `Exception`.

If any gate fails, FALL BACK to `ant:v26-fallback` (re-tag of v26 image)
and submit v26 again as a variance datapoint.  Don't push a regression to
the portal — Day-2 budget for finding regressions is sim, not eval.

## 5. Pre-submit (≤ 30 min)

1. Update `pre_submit_checklist.md`:
   - Tag `v27`, UTC slot `2026-05-10`, PST window populated.
   - Driver = today's engineer; Reviewer = the *other* engineer (must
     differ — pairing rule).
   - Mark each gate `[x]` based on §4 sim results.  Any gate that
     genuinely cannot be satisfied gets `[w]` with a substantive reason.
   - Cannot waive both A0 and T3 in the same submission (`submit.sh`
     enforces this — v25 anti-pattern).
2. Reviewer signs the checklist by adding their name to the "Reviewer
   (independent)" line and committing.
3. Re-tag the previous best image as `ant:v26-fallback` and push to ECR
   so Day-3 has a fallback ready.

## 6. Build & push (≤ 30 min)

```bash
./submit.sh v27
```

`submit.sh` will:
1. Parse `pre_submit_checklist.md`; abort on unchecked or unreasoned-waiver.
2. Reject the build if both A0 and T3 are waived (anti-pattern).
3. Run unit tests (host-side); abort on failure.
4. Read T3 wall-clock from `${T3_LOG_PATH:-/tmp/ant_local_sim/policy.log}`;
   abort if > 110 s OR if log is missing AND not waived.
5. Build with `BUILD_VERSION=v27-<sha>`.  If `-dirty`, capture
   `working_tree.diff` to `submit_evidence_v27/`; abort if dirty bits
   include policy code (`ant_policy_node/.../*.py`).
6. Run install/ sync (Bug 90); abort on diff.
7. Run image-content marker check (12+ markers, 13+ anti-markers, with
   per-marker counts).  Abort on missing or anti-marker present.
8. Push to ECR.

Time budget: builds typically 8-12 minutes.  If anything fails, debug
locally, do NOT pass `--waive-checklist-lead-signed` without explicit
sign-off from Lead A or Lead B.

## 7. Portal registration (≤ 5 min)

1. Open AIC portal.
2. Paste image URI from
   `ant_policy_node/sim_runs/submit_evidence_v27/verify_output.txt`.
3. Submit.  Confirm submission ID is recorded.
4. Update postmortem template's Score: line to "Pending — submitted at
   <PST timestamp> = UTC <date>".

## 8. End-of-day handoff (≤ 15 min)

1. Push all v27 commits, evidence dir, checklist updates.
2. Open the v26 postmortem and fill it in with the v26 score (already
   done in step 2; this is just a re-read).
3. Write a one-paragraph standup note for tomorrow:
   - v27 expected score band.
   - Anything that surprised the team in v26 result.
   - Whether any flag was flipped in v27 vs the planned default-off
     ladder.

---

## Hard rules (non-negotiable)

- **No build after PST 16:00** unless Lead-signed.  We need 1 hour
  buffer for portal flakiness before the UTC 23:59 window closes.
- **No solo build.** Driver and reviewer must be different humans.
  Engineer + Claude is one human; that does not satisfy the rule.
- **No `--waive-checklist-lead-signed` without a Slack/email trail** to
  Lead A or Lead B confirming the waiver.  The flag is logged into
  `submit_evidence_<TAG>/checklist_state.txt` so this is auditable.
- **Postmortem must be filled before next-slot build.**  `submit.sh`
  warns; do not ignore the warning.
