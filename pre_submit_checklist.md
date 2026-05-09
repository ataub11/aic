# Pre-Submit Checklist

**Authoritative location:** `pre_submit_checklist.md` at repo root.
**Parsed by:** `submit.sh` — push is refused if any required line is not
checked OR explicitly waived with a documented reason.

This file is a Day-1-action item from the team review of `b078aee` /
`df82c5c` and the v26 retrospective. It exists because two principles
were quietly relaxed on Day 1 (A0 skipped, T3 wall-clock gate skipped)
and we need mechanism instead of discipline.

---

## How this works

For every submission `vN`, the on-call engineer:

1. Performs each gate listed below.
2. Marks the line `[x]` (passed) or `[w] reason: <free text>` (waived).
3. Commits the updated checklist as part of the submission's evidence.
4. Runs `./submit.sh vN`. The script reads this file, refuses the push
   if any required line is `[ ]` (unchecked) or `[w]` without a reason.

Lines marked `[w]` MUST include a non-empty reason after `reason:`.
The reason is captured in `submit_evidence_<TAG>/checklist_state.txt`
so future audits can see what was waived and why.

Either of two engineers may sign a waiver, but a single engineer **cannot
unilaterally waive both A0 and the T3 gate** in the same submission —
that's the v25 anti-pattern and is mechanically enforced by `submit.sh`.

---

## Submission slot info

- Tag:                       `<filled by engineer>`
- UTC slot date:             `<UTC YYYY-MM-DD>`
- PST build window:          `<PST start — PST end>`
- Engineer (driver):         `<name>`
- Reviewer (independent):    `<name>` (must differ from driver)

---

## Required gates

### Diagnosis & forensics

- [ ] **A0 — Local sim repro** (single discriminator for the slot's
      diagnosis). For revert/calibration submissions the discriminator is
      "does the prior failure mode reproduce locally?"; for forward
      progress submissions it is "does the new code achieve its sim-level
      target?".
      Acceptance: documented in `sim_runs/run_<UTC-date>_<TAG>_postmortem/A0_*.md`.
      Waiver requires sign-off from Lead A or Lead B with reason.

- [ ] **A1 — Image audit of the prior shipped image** (only for slots
      following an unexplained score regression).
      Acceptance: documented in `sim_runs/run_<UTC-date>_*_postmortem/A1_*.md`.

- [ ] **A2 — Static call-graph audit** of any new helper, only when the
      submission introduces a new code path that crosses trial boundaries.

### Code & build

- [ ] **Unit tests pass** — `python3 -m unittest discover -s ant_policy_node/tests`
      green host-side. (Auto-checked by `submit.sh`; this line documents
      the engineer ran it intentionally.)

- [ ] **install/ tree in sync** — `diff -q` clean for ANT.py and
      `ur5e_kinematics.py`. (Auto-synced by `submit.sh`; this line
      documents the engineer reviewed the sync output.)

- [ ] **build_version != "unknown"** — git SHA injected.

- [ ] **build_version is NOT -dirty**, OR a `working_tree.diff` artifact
      is captured in `submit_evidence_<TAG>/`. (`submit.sh` writes the
      diff automatically when build is dirty; engineer must confirm the
      diff is policy-irrelevant, e.g. only touches submit.sh.)

### Behavioural validation

- [ ] **T3 wall-clock < 110 s** in local sim.
      Acceptance: `policy.log` referenced via `T3_LOG_PATH` env var or
      default `/tmp/ant_local_sim/policy.log`.
      Waiver requires sign-off from Lead A or Lead B with reason.

- [ ] **T1 sim score ≥ 50** in local sim.

- [ ] **No `joint_space_guard_violation` events** in the local sim
      `policy.log` (defense-in-depth — this gate confirms the v26 zone
      guard is wired correctly and isn't being silently triggered).

- [ ] **No Traceback / Exception** in local sim `policy.log`.

### Process

- [ ] **Markers + anti-markers** verified by `submit.sh` post-build run
      (this line confirms the engineer reviewed the output and is not
      ignoring a `MISSING:` or `ANTI-MARKER present:` warning).

- [ ] **Fallback image re-tagged** — the previous best `vM-fallback`
      image is pushed to ECR before the new submission, so a Day-N+1
      bake-off has a known-good fallback. (See plan §"Bake-off discipline".)

- [ ] **Prior postmortem filled** — `postmortem_v<N-1>.md` no longer
      contains "TODO" stubs. (`submit.sh` warns; this line is the
      engineer's confirmation.)

- [ ] **This submission's postmortem template exists** — `submit.sh`
      auto-creates it; engineer confirms creation.

- [ ] **Independent reviewer** has signed the image audit and the
      pre-submit checklist. The reviewer must differ from the driver
      (pairing rule). For first build of the day this MAY be waived if
      the driver explicitly notes "solo-build" with a reason.

---

## Waiver-line syntax (parsed by `submit.sh`)

```
- [w] T3 wall-clock < 110 s in local sim.
      reason: gz simulator unavailable on build host (ticket OPS-471);
      will run on B-host before v27 ships.
```

The script extracts the substring after `reason:` and refuses if empty.
Multi-line reasons are fine; only the first line is captured for the
evidence file.

---

## v26 (UTC 2026-05-09) — retrospective entry

This section is filled IN ARREARS for the Day-1 submission so the
checklist is established as of v26. Real enforcement begins with v27.

- Tag:                       `v26`
- UTC slot date:             `2026-05-09`
- PST build window:          `2026-05-08 19:00 — 2026-05-09 16:59`
- Engineer (driver):         Allison + Claude Sonnet 4.6
- Reviewer (independent):    NOT ASSIGNED (waived for Day-1 only)

- [x] Unit tests pass
- [x] install/ tree in sync
- [x] build_version != "unknown" (v26-df82c5c-dirty)
- [x] -dirty captured? **PARTIAL** — submit.sh edits were the dirty
      bits; py_hashes.txt confirms ANT.py matches df82c5c byte-for-byte;
      working_tree.diff was NOT captured (process gap; v27 onward will
      capture automatically).
- [x] Markers + anti-markers verified (12 markers + 13 anti-markers ×
      3 copies — all green per `submit_evidence_v26/verify_output.txt`)
- [x] This submission's postmortem template exists
      (`run_2026-05-09_v26_postmortem/postmortem_v26.md`)
- [x] Prior postmortem filled (`postmortem_v25.md` complete)

- [w] A0 — Local sim repro.
      reason: eval-container network state and 14:00 PST self-imposed
      deadline precluded a full sim run; A1 image audit + A2 static
      callgraph provide consistent (not equivalent) evidence. Waived
      ONCE for Day-1 with Lead A and Lead B post-hoc sign-off; v27
      must run A0 or take an explicit Lead-signed waiver.

- [w] T3 wall-clock < 110 s.
      reason: same — local sim not run on build host this slot. Soft
      gate (no policy.log present) skipped per submit.sh's design.
      v27 onward this gate is HARD.

- [w] T1 sim score ≥ 50.        reason: see T3.
- [w] No guard_violation events. reason: see T3.
- [w] No Traceback / Exception.  reason: see T3.

- [ ] Independent reviewer signed. **NOT SIGNED.** Driver and reviewer
      were the same person (Allison + Claude Sonnet 4.6). v27 onward
      requires a distinct human reviewer.

- [ ] Fallback image re-tagged. **NOT DONE.** v25 image was not
      re-tagged as `v25-fallback` before v26 push. For v27 we will
      re-tag v26 → v26-fallback before pushing v27.

**Net Day-1 disposition:** 5 unwaived gaps captured for retrospective
visibility. None are blocking for v26 (image is in ECR, anti-markers
green, postmortem written). All MUST be resolved before v27.
