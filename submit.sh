#!/usr/bin/env bash
# submit.sh — Build, verify, tag, and push the ANT policy image to the AIC ECR registry.
#
# Usage:
#   ./submit.sh <tag> [--skip-verify]
#
#   <tag>            Required. Version tag, e.g. v1, v2, or a git SHA.
#   --skip-verify    Skip local docker compose up verification step.
#
# Prerequisites:
#   - AWS CLI installed and configured (see submission.md §2)
#   - TEAM_NAME env var set (or edit the defaults below)
#   - ECR_REGISTRY env var set (or edit the default below)
#
# Example:
#   TEAM_NAME=teamabc ECR_REGISTRY=973918476471.dkr.ecr.us-east-1.amazonaws.com \
#     ./submit.sh v3

set -euo pipefail

# ── Configuration ──────────────────────────────────────────────────────────────
TEAM_NAME="${TEAM_NAME:-}"
ECR_REGISTRY="${ECR_REGISTRY:-973918476471.dkr.ecr.us-east-1.amazonaws.com}"
LOCAL_IMAGE="ant-policy"
DOCKER_COMPOSE_FILE="docker/docker-compose.yaml"
AWS_REGION="us-east-1"

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

if [[ -z "$TEAM_NAME" ]]; then
  echo "Error: TEAM_NAME env var must be set (e.g. export TEAM_NAME=teamabc)"
  exit 1
fi

LOCAL_TAGGED="${LOCAL_IMAGE}:${TAG}"
REMOTE_IMAGE="${ECR_REGISTRY}/aic-team/${TEAM_NAME}:${TAG}"

# ── Step 1: Build ──────────────────────────────────────────────────────────────
echo ""
echo "=== Step 1: Building image ${LOCAL_TAGGED} ==="
# Update docker-compose image tag to match requested tag
sed -i "s|image: ant-policy:.*|image: ${LOCAL_TAGGED}|" "$DOCKER_COMPOSE_FILE"
docker compose -f "$DOCKER_COMPOSE_FILE" build model

# ── Step 2: Verify locally ─────────────────────────────────────────────────────
if [[ "$SKIP_VERIFY" == "false" ]]; then
  echo ""
  echo "=== Step 2: Local verification (docker compose up — Ctrl+C to stop) ==="
  echo "Check that the ANT policy node starts and connects to the eval service."
  echo "Press Ctrl+C when satisfied, or let it run to completion."
  docker compose -f "$DOCKER_COMPOSE_FILE" up || true
  echo ""
  read -rp "Did the policy node start successfully? [y/N] " confirm
  if [[ "${confirm,,}" != "y" ]]; then
    echo "Aborting submission. Fix the issue and re-run."
    exit 1
  fi
else
  echo ""
  echo "=== Step 2: Local verification SKIPPED (--skip-verify) ==="
fi

# ── Step 3: Authenticate with ECR ─────────────────────────────────────────────
echo ""
echo "=== Step 3: Authenticating with ECR ==="
export AWS_PROFILE="${TEAM_NAME}"
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
echo "  Next steps:"
echo "  1. Log in to the submission portal."
echo "  2. Go to AI for Industry Challenge → Submit → Qualification phase."
echo "  3. Paste the Image URI above into the 'OCI Image' field and click Submit."
echo "  4. Monitor status at My Submissions (takes 5–15 min)."
echo "======================================================================"
