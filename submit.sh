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
