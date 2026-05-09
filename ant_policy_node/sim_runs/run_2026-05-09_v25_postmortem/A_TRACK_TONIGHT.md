# A-Track — Tonight (UTC 2026-05-09 evening)

**Decision:** Ship `ant:v26` as built (Option α — strict revert + diagnostics).
**Authority:** Joint sign-off Lead A + Lead B 2026-05-09.
**Reason:** Compounded-risk principle. v26 is the controlled baseline that lets
v27 onward be interpretable. Sim-validate it tonight; click submit tomorrow.

The portal slot is held until clicked. Take the time. **Do not click submit
tonight.** Tomorrow at PST 14:00 is the target.

---

## Owners + ETA

| Step | Owner | Wall-clock | Window |
|---|---|---|---|
| 1. Local sim of `ant:v26` (3-trial, full pipeline) | Build-host driver | 90–120 min | starts now |
| 2. Independent image audit by Eng-2 | Eng-2 (must differ from driver) | 30 min | parallel with step 1 |
| 3. Re-tag `ant:v25` → `ant:v25-fallback` in ECR | Build-host driver | 5 min | during step 1 |
| 4. Update `pre_submit_checklist.md` v26 entry from `[w]` to `[x]` | Build-host driver | 15 min | after step 1 |
| 5. Commit + push everything | Build-host driver | 5 min | end of night |
| **STOP** | — | — | bed by PST 23:00 |

Tomorrow 09:00 PST: Lead A sends stakeholder comms. Tomorrow 14:00 PST: portal click.

---

## Step 1 — Local sim of `ant:v26`

```bash
# 1a. Pull the v26 image we built
docker pull 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v26
docker tag 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v26 ant-policy:v26

# 1b. Run a clean 3-trial sim.  Whatever the local-sim wrapper is on this
#     host (scripts/local_sim.sh was reverted in commit 6949809; check
#     with the build-host engineer what they use today).
#
# Required: the sim must write to /tmp/ant_local_sim/policy.log so the
# T3 wall-clock gate in submit.sh can read it (env T3_LOG_PATH overrides).

# 1c. Verify the run produced the diagnostic events we expect.
LOG=/tmp/ant_local_sim/policy.log
grep -c "ANT-DIAG event=trial_end"        $LOG   # expect 3
grep    "ANT-DIAG event=trial_end trial=3" $LOG  # check duration_sec
grep    "ANT-DIAG event=baseline"         $LOG   # cable_force_baseline + high_tension
grep    "ANT-DIAG event=joint_space_guard_violation" $LOG  # MUST be 0 lines
grep -E "Traceback|Exception"             $LOG | head  # MUST be empty
```

### Acceptance gates (all must pass)

| Gate | Required | Why |
|---|---|---|
| `trial_end` event count | == 3 | Confirms 3 trials ran to completion |
| T3 `duration_sec` | < 110 | The new mechanical T3 gate; v25 anti-pattern detector |
| T1 sim score | ≥ 50 | Baseline holds — no T1 regression from additive C1/C5/guards |
| T3 sim score | ≥ 30 | Bug 122 yaw alignment still present (no Bug-90 silent regression) |
| `joint_space_guard_violation` count | == 0 | Defense-in-depth guard isn't being triggered (= no caller drift) |
| Traceback / Exception count | == 0 | No new code path crashing the policy |

### Output

Create `ant_policy_node/sim_runs/run_2026-05-09_v26_postmortem/A0_local_repro.md`
with:
- Section "Sim configuration" — image SHA, sim env, cable-tension setting.
- Section "Per-trial result" — T1, T2, T3 each with: duration_sec, score
  (sim's), final TCP pose, C1 code if any failure path fired.
- Section "Acceptance gate table" — copy the above table, fill in
  observed values.
- Section "Decision" — one line. Either "Proceed to portal submit
  tomorrow" or "HALT — see below for failure mode."

---

## Step 1 stop-rules (decision tree)

| Sim outcome | Action | Notes |
|---|---|---|
| All 6 gates green | Proceed. v26 will ship per plan tomorrow. | Update checklist Step 4. |
| T3 `duration_sec` > 110 | **HALT.** v26 has the same regression as v25. | Investigate which v25 anti-marker we missed in submit.sh. Most likely cause: an additive change interacted with Stage 4 SC. |
| T1 sim < 50 | **HALT.** v26 broke T1. | Most likely cause: an additive C1/C5/guard interacted with Bug 106 lateral retry. Bisect the v26 additive layer. |
| `guard_violation` events present | **Investigate but probably ship.** | Means a non-T2 trial reached `_lateral_move_joint_space`. Guard fail-closed correctly so behavior is OK, but the call-site is a latent bug for v27 to fix. Document in postmortem. |
| Traceback / Exception present | **HALT.** | Investigate the trace. Likely a v26 additive code path with an edge case sim hits but tests didn't. |

If we HALT, we have all night to rebuild v26' (re-spin v26 with the fix)
or fall back to `ant:v24` digest re-tag as v26 calibration shot. **Do
not skip the slot.** UTC May 9 23:59 hard deadline.

---

## Step 2 — Independent image audit (Eng-2)

Pairing rule: the v26 build was self-driven. Eng-2 (NOT the build-host
driver) does an independent ECR pull + grep before submission.

```bash
# 2a. Fresh pull from ECR (do not reuse build-host's local image)
docker pull 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/ant:v26 2>&1 \
  | tee /tmp/v26_audit_pull.log

# 2b. md5 hashes — must match submit_evidence_v26/py_hashes.txt
docker run --rm --entrypoint /bin/bash ant:v26 -c '
  for f in $(find /ws_aic -name ANT.py -not -path "*/proc/*"); do
    md5sum "$f"
  done' | tee /tmp/v26_audit_hashes.txt

diff /tmp/v26_audit_hashes.txt \
     <(awk "/ANT.py/ {print}" ant_policy_node/sim_runs/submit_evidence_v26/py_hashes.txt | sort) \
  && echo "✓ md5 matches build evidence" \
  || echo "✗ MISMATCH — image in ECR is not what we built"

# 2c. Marker counts (Eng-2 reads them with their own eyes)
docker run --rm --entrypoint /bin/bash ant:v26 -c '
  for f in $(find /ws_aic -name ANT.py); do
    echo "=== $f ==="
    for m in "-1.7133" "joint_space_guard_violation" \
             "_remaining_trial_budget_sec" "enable_c1_diagnostic_signature"; do
      n=$(grep -cF -- "$m" "$f")
      [[ "$n" -ge 1 ]] && echo "  ✓ $m (n=$n)" || echo "  ✗ MISSING $m"
    done
    for am in "enable_cable_overdrive" "BUG125 rel-to-current" \
              "# CR-2" "# CR-3" "# HR-1"; do
      n=$(grep -cF -- "$am" "$f")
      [[ "$n" -eq 0 ]] && echo "  ✓ absent $am" || echo "  ✗ ANTI-MARKER PRESENT $am (n=$n)"
    done
  done'
```

### Output

Append a new section to `A1_image_audit.md`:

```markdown
## Independent verification by Eng-2 (2026-05-09 evening)

- Pulled fresh from ECR: ant:v26 digest sha256:525a83…
- md5 match vs build evidence: PASS / FAIL
- Marker counts (4 markers × 3 ANT.py copies): all PRESENT / N missing
- Anti-marker counts (5 anti-markers × 3 ANT.py copies): all 0 / N present
- Signed: <Eng-2 name>
```

Then mark the "Independent reviewer signed" line in
`pre_submit_checklist.md` v26 entry as `[x]` with the engineer's name.

### Step 2 stop-rule

If md5 differs OR any anti-marker count > 0 OR any required marker
missing: **HALT.** The image in ECR is not what we built. Investigate
ECR vs build-host before any portal action.

---

## Step 3 — Re-tag `ant:v25` → `ant:v25-fallback`

Day-1 checklist had this as `[ ]` not done. Doing it tonight removes a
Day-2 time sink and gives Day-3 (v27) a known-good fallback.

```bash
aws ecr batch-get-image \
  --repository-name aic-team/ant \
  --image-ids imageTag=v25 \
  --query 'images[].imageManifest' --output text \
| aws ecr put-image \
  --repository-name aic-team/ant \
  --image-tag v25-fallback \
  --image-manifest file:///dev/stdin

# Verify
aws ecr describe-images \
  --repository-name aic-team/ant \
  --image-ids imageTag=v25 imageTag=v25-fallback \
  --query 'imageDetails[].{Tag:imageTags,Digest:imageDigest}'
# Both lines must show the SAME imageDigest.
```

Mark `pre_submit_checklist.md` "Fallback image re-tagged" as `[x]`.

---

## Step 4 — Update v26 checklist with real evidence

Open `pre_submit_checklist.md`. In the v26 retrospective section, change:

```diff
- [w] A0 — Local sim repro.
-     reason: eval-container network state and 14:00 PST self-imposed deadline ...
+ [x] A0 — Local sim repro.
+     evidence: sim_runs/run_2026-05-09_v26_postmortem/A0_local_repro.md
+     T1=<obs> T3_dur=<obs>s T3_score=<obs> guard_violations=0
```

Same for T3 wall-clock, T1 sim score, no-guard-violations, no-Traceback.
Driver and reviewer lines: fill in actual names.

If Step 1 surfaced any HALT: do NOT update the checklist; instead add a
new section "v26' rebuild" with the rebuild plan.

---

## Step 5 — Commit + push

```bash
git add ant_policy_node/sim_runs/run_2026-05-09_v26_postmortem/A0_local_repro.md
git add ant_policy_node/sim_runs/run_2026-05-09_v25_postmortem/A1_image_audit.md  # Eng-2 sign-off
git add pre_submit_checklist.md
git commit -m "v26 sim validation tonight: A0 + Eng-2 image audit + v25-fallback"
git push origin claude/analyze-v25-regression-25OyM
```

Then go home. Tomorrow morning resumes with:
1. Lead A sends stakeholder comms (09:00 PST).
2. Build-host engineer reads B-track sim results from Eng-3/Eng-4 branches.
3. 14:00 PST: portal click on the v26 image URI from
   `ant_policy_node/sim_runs/submit_evidence_v26/verify_output.txt`.
4. After portal confirmation, decide v27 candidate per Day-2 workflow.

---

## What you do NOT do tonight

- **Do not click submit on the AIC portal tonight.** A0 first.
- **Do not modify ANT.py or submit.sh.** v26 is locked.
- **Do not pass `--waive-checklist-lead-signed`** for any reason.
- **Do not start B-track work** — Eng-3 and Eng-4 own that on their branches.
- **Do not stay past PST 23:00.** Tomorrow's window is 8 hours wide. Use it.

If you are blocked or any HALT fires: page Lead A or Lead B before
making any decision that goes outside this doc.
