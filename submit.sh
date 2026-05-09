#!/usr/bin/env bash
# submit.sh — Build, verify, tag, and push the ANT policy image to the AIC ECR registry.
# Follows the steps in docs/submission.md exactly.
#
# Usage:
#   ./submit.sh <tag> [--skip-verify]
#
#   <tag>            Required. Version tag, e.g. v5, v6, or a git SHA.
#   --skip-verify    Skip local docker compose up verification step.
#
# Prerequisites:
#   - AWS CLI installed
#   - AWS profile 'ant_aic' configured (aws configure --profile ant_aic)
#   - Docker running
#
# Bug 90 / Bug 123 Safeguard (stale or missing install/ files):
#   This script includes two protective steps:
#   1. Pre-build: Syncs ant_policy_node/install/ from source for ANT.py,
#      __init__.py, stage1_debug.py, and ur5e_kinematics.py to ensure the
#      Dockerfile copies fresh code (not stale committed copies).  If a
#      source file exists but its install/ counterpart does not, the script
#      aborts (a new package member must have its install/ copy committed).
#   2. Post-build: Runtime verification extracts ANT.py and ur5e_kinematics.py
#      from the image and asserts that all expected code markers are present
#      (yaw correction constant from Bug 122, joint-space IK call sites and
#      module definitions from Bug 123).  Missing markers ABORT the push —
#      the image is unshippable.
#   Without these steps, Python's import resolution can pick the old install/
#   copy at runtime, shipping old or incomplete code despite new commits being
#   pushed (observed: v23 shipped without verifiable IK code presence).

set -euo pipefail

# ── Configuration ──────────────────────────────────────────────────────────────
TEAM_NAME="ant"
ECR_REGISTRY="973918476471.dkr.ecr.us-east-1.amazonaws.com"
AWS_PROFILE="ant_aic"
AWS_REGION="us-east-1"
LOCAL_IMAGE="ant-policy"
DOCKER_COMPOSE_FILE="docker/docker-compose.yaml"

# ── Argument parsing ────────────────────────────────────────────────────────────
TAG="${1:-}"
SKIP_VERIFY=false
for arg in "$@"; do
  [[ "$arg" == "--skip-verify" ]] && SKIP_VERIFY=true
done

if [[ -z "$TAG" ]]; then
  echo "Error: version tag required. Usage: ./submit.sh <tag> [--skip-verify]"
  exit 1
fi

LOCAL_TAGGED="${LOCAL_IMAGE}:${TAG}"
REMOTE_IMAGE="${ECR_REGISTRY}/aic-team/${TEAM_NAME}:${TAG}"

# ── v26 / A4: pre-build unit tests (Workstream A acceptance gate) ────────────
# Tests run on the build host (no ROS imports needed for the green path).
# A failure here means a v26 helper regressed; the build is aborted before
# Docker even starts, saving 5+ minutes vs catching it post-build.
echo ""
echo "=== Pre-build: v26 Workstream A unit tests ==="
if python3 -m unittest discover -s ant_policy_node/tests -v; then
  echo "✓ v26 unit tests pass"
else
  echo "✗ FATAL: v26 unit tests failed.  Aborting before build."
  exit 1
fi

# ── v26 / Workstream C2: T3 wall-clock gate from prior local sim ─────────────
# If a recent local sim's policy.log is available (from `scripts/local_sim.sh`
# or equivalent), grep the trial=3 trial_end event for `duration_sec=` and
# refuse the push if T3 timed out.  The gate is SOFT: when no log is found
# (e.g. first build of the day), it warns but does not block.  This way the
# gate adds value when a sim has been run, but doesn't block fresh hosts.
T3_LOG_PATH="${T3_LOG_PATH:-/tmp/ant_local_sim/policy.log}"
echo ""
echo "=== Pre-build: T3 wall-clock gate (Workstream C) ==="
if [[ -f "$T3_LOG_PATH" ]]; then
  T3_DUR=$(grep "ANT-DIAG event=trial_end trial=3" "$T3_LOG_PATH" 2>/dev/null \
           | grep -oE 'duration_sec=[0-9.]+' | head -1 \
           | awk -F= '{print $2}')
  if [[ -z "$T3_DUR" ]]; then
    echo "⚠ T3 wall-clock gate: no trial=3 trial_end found in $T3_LOG_PATH"
    echo "  (run a local sim before submitting to enable this gate)"
  else
    # bash-only float compare (no bc dependency)
    if awk "BEGIN{exit !($T3_DUR > 110)}"; then
      echo "✗ FATAL: T3 wall-clock $T3_DUR s > 110 s threshold."
      echo "  v25-class regression detected in local sim.  Aborting push."
      exit 1
    fi
    echo "✓ T3 wall-clock $T3_DUR s ≤ 110 s"
  fi
else
  echo "⚠ T3 wall-clock gate: $T3_LOG_PATH not found — gate skipped."
  echo "  Set T3_LOG_PATH to your local sim's policy.log to enable."
fi

# ── Step 1: Build the image ────────────────────────────────────────────────────
echo ""
echo "=== Step 1: Building image ${LOCAL_TAGGED} ==="
# Keep docker-compose image tag in sync with the requested tag.
sed -i "s|image: ant-policy:.*|image: ${LOCAL_TAGGED}|" "$DOCKER_COMPOSE_FILE"
# Inject the current git SHA (with a -dirty suffix when the tree has
# uncommitted changes) as BUILD_VERSION; ANT.py logs it on startup so every
# eval log identifies the exact code that shipped.
GIT_SHA="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [[ "$GIT_SHA" != "unknown" ]] && ! git diff --quiet HEAD 2>/dev/null; then
  GIT_SHA="${GIT_SHA}-dirty"
fi
if [[ "$GIT_SHA" == "unknown" ]]; then
  echo "Error: git rev-parse failed — refusing to ship a build_version=unknown image."
  echo "Run submit.sh from inside the aic git checkout."
  exit 1
fi
export BUILD_VERSION="${TAG}-${GIT_SHA}"
echo "BUILD_VERSION=${BUILD_VERSION}"

# ── Pre-build: Sync stale install/ directory (Bug 90 gotcha) ─────────────────
echo ""
echo "=== Pre-build: Syncing install/ directory from source (Bug 90) ==="
# The install/ directory contains a stale committed copy of ANT.py and friends.
# At runtime, Python's import resolution sometimes picks the install/ copy over
# source, causing deployed images to have old code. This step ensures they're
# in sync before building.
INSTALL_ANT_PY="ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/ANT.py"
if [[ -f "$INSTALL_ANT_PY" ]]; then
  cp ant_policy_node/ant_policy_node/ANT.py "$INSTALL_ANT_PY"
  if diff -q ant_policy_node/ant_policy_node/ANT.py "$INSTALL_ANT_PY" >/dev/null 2>&1; then
    echo "✓ install/ANT.py synced successfully"
  else
    echo "✗ FATAL: install/ANT.py sync verification failed (diff still shows differences)"
    exit 1
  fi
fi
# Also sync __init__.py, stage1_debug.py, and ur5e_kinematics.py if they exist
# and have been modified.  ur5e_kinematics.py was added in Bug 123 (v23) — it
# is a NEW package member that the install/ tree must mirror so that
# `from ant_policy_node import ur5e_kinematics` resolves at runtime regardless
# of which copy Python imports.
for file in __init__.py stage1_debug.py ur5e_kinematics.py; do
  SRC="ant_policy_node/ant_policy_node/${file}"
  DEST="ant_policy_node/install/ant_policy_node/lib/python3.12/site-packages/ant_policy_node/${file}"
  if [[ -f "$SRC" && -f "$DEST" ]]; then
    cp "$SRC" "$DEST"
    if diff -q "$SRC" "$DEST" >/dev/null 2>&1; then
      echo "✓ install/${file} synced"
    fi
  elif [[ -f "$SRC" && ! -f "$DEST" ]]; then
    echo "✗ FATAL: ${SRC} exists but ${DEST} is missing."
    echo "  ${file} would not be importable from the install/ tree at runtime."
    echo "  Add the install/ copy or remove the source file before submitting."
    exit 1
  fi
done

# Drop any prior image with this tag and bust BuildKit cache so the
# ARG BUILD_VERSION layer (and the source COPY layers) are rebuilt fresh.
# Without this, Docker happily reuses a cached early layer that baked in
# BUILD_VERSION=unknown, and the running container reports the wrong SHA
# even though the source COPY layers were rebuilt.
docker image rm "${LOCAL_TAGGED}" >/dev/null 2>&1 || true
docker compose -f "$DOCKER_COMPOSE_FILE" build --no-cache --pull model

# Verify BUILD_VERSION actually made it into the image's environment.
# The Dockerfile sets ENV ANT_BUILD_VERSION=${BUILD_VERSION}; if that ended
# up as "unknown" the image is unshippable per CLAUDE.md.
BAKED_VERSION="$(docker image inspect "${LOCAL_TAGGED}" \
  --format '{{range .Config.Env}}{{println .}}{{end}}' \
  | awk -F= '/^ANT_BUILD_VERSION=/ {print $2}')"
echo "Baked ANT_BUILD_VERSION=${BAKED_VERSION}"
if [[ "$BAKED_VERSION" != "$BUILD_VERSION" ]]; then
  echo "Error: image baked with ANT_BUILD_VERSION='${BAKED_VERSION}', expected '${BUILD_VERSION}'."
  echo "Aborting before push so we don't ship a misidentified image."
  exit 1
fi

# Runtime verification: run a bash command inside the image to find and grep
# every copy of the policy files.  Uses --entrypoint /bin/bash to bypass the
# router-addr check in the real entrypoint.  This avoids hardcoding the install
# path (pixi env copy vs install/ copy) and checks ALL copies present in the
# image — if any copy is missing a marker the build is considered stale.
#
# The old approach used `docker cp /ws_aic/install/aic_model/...` which was a
# path that never existed; the `|| true` silently masked 100% of checks.
# This version is fail-closed: any missing marker or docker run failure aborts.
echo ""
echo "=== Step 1b: Runtime verification (image content check) ==="

# Markers are embedded directly in the script string to avoid quoting issues
# when passing arrays with spaces through env vars.
# Add a new line per bug whose presence in the deployed image must be confirmed.
# Format: ANT_MARKERS entries are grepped against every ANT.py found in the
# image; IK_MARKERS entries against every ur5e_kinematics.py.
VERIFY_SCRIPT=$(cat <<'SCRIPT_END'
set -e
FAIL=0

# ── Marker definitions (add new bugs here) ────────────────────────────────────
ANT_MARKERS=(
  "-1.7133"                                # Bug 122 yaw correction
  "solve_ik_dls"                           # Bug 123 IK call site in ANT.py
  "def _lateral_move_joint_space"          # Bug 123 joint-space helper
  "enable_joint_space_lateral"             # Bug 123 master toggle
  "class JointSpaceDiagnosticAbort"        # Bug 124 A/B/C discriminator class
  "enable_joint_space_diag_signatures"     # Bug 124 master toggle
  "BUG124 diag signature"                  # Bug 124 runtime log line
  # ── v26 (Workstream A) sentinels ──────────────────────────────────────
  # Reverted from v25: Bug 125/126/127 markers removed.  Their absence
  # in the built image is the strict-revert acceptance gate.
  "joint_space_guard_violation"            # v26 zone/trial guard event name
  "enable_c1_diagnostic_signature"         # v26 C1 master toggle
  "_c1_failure_code_pending"               # v26 C1 state
  "enable_hw_variance_log"                 # v26 C5 master toggle
  "_remaining_trial_budget_sec"            # v26 budget accountant
)
# v26 strict-revert anti-markers: any of these strings appearing in the
# built image's ANT.py copies indicates the v25 changes did NOT fully
# revert.  Push is aborted if any anti-marker is present.
#
# All strings below were verified absent from the v24 baseline ANT.py at
# `git checkout e52c930` time (see review of commit b078aee).  v24 had a
# benign `lateral_arrival_ff_scale_per_retry` (Bug 106) which would
# false-positive on a bare `_ff_scale` substring — so CR-1/HR-1 are
# detected via the `# CR-1` / `# HR-1` v25-commit-tag comments instead,
# which are the unique substring v25 introduced.
ANT_ANTI_MARKERS=(
  # Bug 125/126/127 — T2 joint-space additions
  "enable_cable_overdrive"                 # Bug 127 — must be absent in v26
  "enable_two_stage_joint_move"            # Bug 127 — must be absent in v26
  "enable_multi_seed_ik"                   # Bug 126 — must be absent in v26
  "diag_signatures_relative_to_current_pose"  # Bug 125 — must be absent
  "BUG125 rel-to-current"                  # Bug 125 log marker — must be absent
  "BUG126 IK"                              # Bug 126 log marker — must be absent
  "BUG127 cable overdrive"                 # Bug 127 log marker — must be absent
  # v25 Stage 4 SC changes (the actual T3-regression cause per A2 audit)
  "# CR-1"                                 # CR-1 baseline-adaptive feedforward
  "# CR-2"                                 # CR-2 SC Stage 4 stiffness 85→200
  "# CR-3"                                 # CR-3 force guard disable
  "# HR-1"                                 # HR-1 ff cap
  "# HR-2"                                 # HR-2 force-checked phase settle
  "# MR-4"                                 # MR-4 Z-floor guard
)
IK_MARKERS=(
  "def solve_ik_dls"
  "def geometric_jacobian"
  "def joint_state_to_ordered_array"
)

# ── File discovery ────────────────────────────────────────────────────────────
ANT_FILES=$(find /ws_aic -name "ANT.py" -path "*/ant_policy_node/*" 2>/dev/null | sort)
IK_FILES=$(find /ws_aic  -name "ur5e_kinematics.py"                  2>/dev/null | sort)

if [[ -z "$ANT_FILES" ]]; then
  echo "✗ MISSING: ANT.py not found anywhere under /ws_aic"
  FAIL=1
fi
if [[ -z "$IK_FILES" ]]; then
  echo "✗ MISSING: ur5e_kinematics.py not found anywhere under /ws_aic (Bug 123 module absent — joint-space IK will ImportError at runtime)"
  FAIL=1
fi

# ── Marker checks ─────────────────────────────────────────────────────────────
# Note: `-e "$m"` is REQUIRED — markers can start with `-` (e.g. -1.7133),
# and without `-e`/`--` grep parses leading-dash markers as flags
# (`-1.7133` → `-1` context-lines + pattern `.7133`), silently checking
# the wrong thing.  This caused every -1.7133 check since Bug 122 to lie.
for f in $ANT_FILES; do
  for m in "${ANT_MARKERS[@]}"; do
    if grep -qF -e "$m" "$f" 2>/dev/null; then
      echo "✓ ANT.py [$f]: $m"
    else
      echo "✗ MISSING: ANT.py [$f] lacks marker: $m"
      FAIL=1
    fi
  done
  # v26 strict-revert gate: any anti-marker present means the revert was
  # incomplete (something from v25 leaked into the deployed image).  This
  # is the second half of "every change is flag-gated, default-off, sim-
  # validated" — a v26 image that still contains v25 code paths is not v26.
  for am in "${ANT_ANTI_MARKERS[@]}"; do
    if grep -qF -e "$am" "$f" 2>/dev/null; then
      echo "✗ ANTI-MARKER present in ANT.py [$f]: $am  (v25 code leaked through revert)"
      FAIL=1
    else
      echo "✓ anti-marker absent ANT.py [$f]: $am"
    fi
  done
done

for f in $IK_FILES; do
  for m in "${IK_MARKERS[@]}"; do
    if grep -qF -e "$m" "$f" 2>/dev/null; then
      echo "✓ ur5e_kinematics.py [$f]: $m"
    else
      echo "✗ MISSING: ur5e_kinematics.py [$f] lacks marker: $m"
      FAIL=1
    fi
  done
done

exit $FAIL
SCRIPT_END
)

VERIFY_OUTPUT=$(docker run --rm --entrypoint /bin/bash \
  "${LOCAL_TAGGED}" -c "$VERIFY_SCRIPT" 2>&1) && VERIFY_EXIT=0 || VERIFY_EXIT=$?

echo "$VERIFY_OUTPUT"

if [[ "$VERIFY_EXIT" -ne 0 ]]; then
  echo ""
  echo "✗ FATAL: image content verification failed."
  echo "  One or more expected code markers are missing from the built image."
  echo "  Likely causes: (a) install/ tree out of sync with source (Bug 90 saga),"
  echo "  (b) docker layer cache served a stale COPY layer, or (c) a new package"
  echo "  member was not added to the install/ tree."
  echo "  Aborting push — DO NOT submit this image."
  exit 1
fi
echo "✓ All image content markers verified — safe to push."

# Save verification evidence alongside sim_runs so every submission has a
# permanent local record of exactly what was in the image at push time.
# This means "what shipped" can be answered from the repo without pulling
# the ECR image or asking the organizers for logs.
EVIDENCE_DIR="ant_policy_node/sim_runs/submit_evidence_${TAG}"
mkdir -p "$EVIDENCE_DIR"
{
  echo "tag=${TAG}"
  echo "build_version=${BUILD_VERSION}"
  echo "built_at=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "---"
  echo "$VERIFY_OUTPUT"
} > "${EVIDENCE_DIR}/verify_output.txt"
# Also snapshot the file-level hashes of every .py that went into the image.
docker run --rm --entrypoint /bin/bash "${LOCAL_TAGGED}" -c \
  'find /ws_aic -name "*.py" -path "*/ant_policy_node/*" | sort | xargs md5sum 2>/dev/null' \
  > "${EVIDENCE_DIR}/py_hashes.txt" 2>/dev/null || true
echo "Evidence saved to ${EVIDENCE_DIR}/"

# ── Step 2: Verify locally ─────────────────────────────────────────────────────
if [[ "$SKIP_VERIFY" == "false" ]]; then
  echo ""
  echo "=== Step 2: Verify locally (docker compose up — Ctrl+C to stop) ==="
  echo "Check that the ANT policy node starts and connects to the eval service."
  echo "Press Ctrl+C when satisfied."
  docker compose -f "$DOCKER_COMPOSE_FILE" up || true
  echo ""
  read -rp "Did the policy node start successfully? [y/N] " confirm
  if [[ "${confirm,,}" != "y" ]]; then
    echo "Aborting. Fix the issue and re-run."
    exit 1
  fi
else
  echo ""
  echo "=== Step 2: Local verification SKIPPED (--skip-verify) ==="
fi

# ── Step 3: Authenticate with the registry ─────────────────────────────────────
echo ""
echo "=== Step 3: Authenticating with ECR ==="
export AWS_PROFILE
aws ecr get-login-password --region "$AWS_REGION" \
  | docker login --username AWS --password-stdin "$ECR_REGISTRY"

# ── Step 4: Tag ────────────────────────────────────────────────────────────────
echo ""
echo "=== Step 4: Tagging ${LOCAL_TAGGED} → ${REMOTE_IMAGE} ==="
docker tag "${LOCAL_TAGGED}" "${REMOTE_IMAGE}"

# ── Step 5: Push ───────────────────────────────────────────────────────────────
echo ""
echo "=== Step 5: Pushing to ECR ==="
docker push "${REMOTE_IMAGE}"

# ── Done ───────────────────────────────────────────────────────────────────────
echo ""
echo "======================================================================"
echo "  Image pushed successfully:"
echo "  ${REMOTE_IMAGE}"
echo ""
echo "  Next steps (Step 3 of submission.md — Register Your Submission):"
echo "  1. Log in to the submission portal."
echo "  2. Go to AI for Industry Challenge → Submit → Qualification phase."
echo "  3. Paste the Image URI above into the 'OCI Image' field and click Submit."
echo "  4. Monitor status at My Submissions (takes 5–15 min)."
echo "======================================================================"
