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

# Runtime verification: extract policy files from the image and check known
# markers to ensure the source code (not stale install/ copy) and all expected
# package members are in the image.  This is the final safeguard against the
# Bug 90 gotcha and the Bug 123 "new file may not have shipped" gotcha.
# Failures here ABORT the push — an image that fails verification is unshippable.
echo ""
echo "=== Step 1b: Runtime verification (image content check) ==="
TEMP_EXTRACT="/tmp/ant_policy_check_$$"
mkdir -p "$TEMP_EXTRACT"
trap 'rm -rf "$TEMP_EXTRACT"' EXIT

CONTAINER_ID=$(docker create "${LOCAL_TAGGED}" 2>/dev/null) || {
  echo "✗ FATAL: could not create container from ${LOCAL_TAGGED} for verification."
  exit 1
}

IMAGE_PKG_DIR="/ws_aic/install/aic_model/lib/python3.12/site-packages/ant_policy_node"

# Extract every file we will verify.  Missing files are a hard failure — we
# do not push images with unknown content.
docker cp "${CONTAINER_ID}:${IMAGE_PKG_DIR}/ANT.py" "$TEMP_EXTRACT/ANT.py" 2>/dev/null \
  || { echo "✗ FATAL: ${IMAGE_PKG_DIR}/ANT.py not present in image."; docker rm "$CONTAINER_ID" >/dev/null 2>&1; exit 1; }
docker cp "${CONTAINER_ID}:${IMAGE_PKG_DIR}/ur5e_kinematics.py" "$TEMP_EXTRACT/ur5e_kinematics.py" 2>/dev/null \
  || { echo "✗ FATAL: ${IMAGE_PKG_DIR}/ur5e_kinematics.py not present in image (Bug 123 module missing — joint-space IK would fail to import at runtime)."; docker rm "$CONTAINER_ID" >/dev/null 2>&1; exit 1; }
docker rm "$CONTAINER_ID" >/dev/null 2>&1

# Each marker is `description|file|grep-extended-regex`.  All markers must
# match; any miss aborts the push.  Add a new line per bug whose presence in
# the deployed image must be confirmed.
MARKERS=(
  "Bug 122 yaw correction (-1.7133)|ANT.py|-1\.7133"
  "Bug 123 IK call site (solve_ik_dls)|ANT.py|solve_ik_dls"
  "Bug 123 joint-space helper (_lateral_move_joint_space)|ANT.py|def _lateral_move_joint_space"
  "Bug 123 joint-space master toggle (enable_joint_space_lateral)|ANT.py|enable_joint_space_lateral"
  "Bug 123 IK module (solve_ik_dls definition)|ur5e_kinematics.py|def solve_ik_dls"
  "Bug 123 IK module (geometric_jacobian)|ur5e_kinematics.py|def geometric_jacobian"
  "Bug 123 IK module (joint_state_to_ordered_array)|ur5e_kinematics.py|def joint_state_to_ordered_array"
)

VERIFY_FAILED=0
for marker in "${MARKERS[@]}"; do
  IFS='|' read -r desc file pattern <<<"$marker"
  if grep -Eq "$pattern" "$TEMP_EXTRACT/$file" 2>/dev/null; then
    echo "✓ ${desc}"
  else
    echo "✗ MISSING: ${desc} (expected pattern '${pattern}' in ${file})"
    VERIFY_FAILED=1
  fi
done

if [[ "$VERIFY_FAILED" -ne 0 ]]; then
  echo ""
  echo "✗ FATAL: image content verification failed."
  echo "  The image at ${LOCAL_TAGGED} is missing expected code markers."
  echo "  Likely causes: (a) install/ tree out of sync with source (Bug 90 saga),"
  echo "  (b) docker layer cache served a stale COPY layer, or (c) a new package"
  echo "  member was not added to the install/ tree."
  echo "  Aborting push — DO NOT submit this image."
  exit 1
fi
echo "✓ All image content markers verified — safe to push."

trap - EXIT
rm -rf "$TEMP_EXTRACT"

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
