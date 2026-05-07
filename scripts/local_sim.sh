#!/usr/bin/env bash
# scripts/local_sim.sh — One-shot local sim run for ANT policy validation.
#
# Usage:
#   ./scripts/local_sim.sh [<run_label>]
#
#   <run_label>  Optional. Suffix for the run directory.
#                Defaults to "v$(grep image: docker/docker-compose.yaml | head -1 |
#                                 sed 's/.*ant-policy://')_local".
#
# What it does:
#   1. Starts the eval container in the background (compose service "eval").
#   2. Waits for Gazebo + AIC engine to be ready.
#   3. Runs the ANT policy under pixi from the host (faster than rebuilding
#      the model image — appropriate for crash-prevention sim, NOT for
#      submission verification, which is gated by submit.sh).
#   4. Tears down the eval container after the policy exits.
#   5. Runs triage on the captured logs and prints PASS/FAIL.
#
# Pass criteria (matches the "crash + T1/T3 regression" goal — sim does NOT
# reproduce the real-HW cable-tension problem so T2 sim scores are
# uninformative):
#   - No Traceback / Exception in policy stdout
#   - 3× ANT-DIAG event=trial_end in policy stdout
#   - At least one BUG126 or BUG127 line on T2 (joint-space code path fired)
#   - T1 sim score ≥ 50, T3 sim score ≥ 30
#
# Artifacts saved to ant_policy_node/sim_runs/run_<date>_<label>/:
#   - eval.log         (eval container stdout — scoring output ends up here)
#   - policy.log       (ANT.py stdout — BUG12x markers, ANT-DIAG events)
#   - ant_diagnostics.jsonl  (Bug 108 sidecar, copied from ~/aic_results/)
#   - triage.txt       (pass/fail summary)

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ── Resolve run directory ──────────────────────────────────────────────────────
DEFAULT_TAG="$(grep 'image: ant-policy:' docker/docker-compose.yaml | \
                head -1 | sed 's|.*ant-policy:||;s|[^A-Za-z0-9._-]||g')"
LABEL="${1:-${DEFAULT_TAG}_local}"
DATE_STR="$(date -u +%Y-%m-%d)"
RUN_DIR="ant_policy_node/sim_runs/run_${DATE_STR}_${LABEL}"
mkdir -p "$RUN_DIR"
EVAL_LOG="${RUN_DIR}/eval.log"
POLICY_LOG="${RUN_DIR}/policy.log"
TRIAGE="${RUN_DIR}/triage.txt"

echo "=== Local sim: ${LABEL} → ${RUN_DIR} ==="

# ── Sanity: docker daemon + compose ────────────────────────────────────────────
if ! docker info >/dev/null 2>&1; then
  echo "✗ FATAL: docker daemon not running."
  exit 1
fi

# ── Cleanup any leftover containers from a previous run ────────────────────────
docker compose -f docker/docker-compose.yaml down --remove-orphans \
  >/dev/null 2>&1 || true

# ── Step 1: Start eval container in background ─────────────────────────────────
echo "Starting eval container (background)…"
docker compose -f docker/docker-compose.yaml up eval > "$EVAL_LOG" 2>&1 &
COMPOSE_PID=$!

cleanup() {
  echo ""
  echo "Cleaning up containers…"
  docker compose -f docker/docker-compose.yaml down --remove-orphans \
    >/dev/null 2>&1 || true
  if kill -0 "$COMPOSE_PID" 2>/dev/null; then
    kill "$COMPOSE_PID" 2>/dev/null || true
    wait "$COMPOSE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ── Step 2: Wait for AIC engine to be ready ────────────────────────────────────
# Engine prints "No node with name 'aic_model' found. Retrying..." once it's up.
# Bound the wait so a stuck eval doesn't hang the script forever.
echo "Waiting for AIC engine to be ready (up to 120 s)…"
DEADLINE=$(( $(date +%s) + 120 ))
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
  if grep -q "No node with name 'aic_model' found" "$EVAL_LOG" 2>/dev/null; then
    echo "✓ AIC engine up"
    break
  fi
  if ! kill -0 "$COMPOSE_PID" 2>/dev/null; then
    echo "✗ FATAL: eval container exited before becoming ready."
    echo "  See $EVAL_LOG for details."
    exit 1
  fi
  sleep 2
done
if ! grep -q "No node with name 'aic_model' found" "$EVAL_LOG" 2>/dev/null; then
  echo "✗ FATAL: AIC engine readiness timeout (120 s)."
  exit 1
fi

# ── Step 3: Run policy under pixi ──────────────────────────────────────────────
echo "Running policy under pixi…"
# shellcheck disable=SC1091
source ./pixi_env_setup.sh

# Truncate the diagnostics sidecar before run so we capture only this trial set.
DIAG_PATH="${HOME}/aic_results/ant_diagnostics.jsonl"
mkdir -p "${HOME}/aic_results" 2>/dev/null || true
: > "$DIAG_PATH" 2>/dev/null || true

pixi run ros2 run aic_model aic_model \
  --ros-args \
  -p use_sim_time:=true \
  -p policy:=ant_policy_node.ANT.ANT \
  > "$POLICY_LOG" 2>&1
POLICY_EXIT=$?

# Snapshot the diagnostics sidecar so we have a permanent record.
if [ -f "$DIAG_PATH" ]; then
  cp "$DIAG_PATH" "${RUN_DIR}/ant_diagnostics.jsonl" 2>/dev/null || true
fi

# ── Step 4: Triage ─────────────────────────────────────────────────────────────
echo ""
echo "=== Triage ==="
{
  echo "label: ${LABEL}"
  echo "run_dir: ${RUN_DIR}"
  echo "policy_exit: ${POLICY_EXIT}"
  echo "---"
} > "$TRIAGE"

PASS=1

# Crash check
CRASH_LINES=$(grep -E "Traceback|^Error:|Exception" "$POLICY_LOG" | head -20)
if [ -n "$CRASH_LINES" ]; then
  echo "✗ CRASH: policy stdout contains exception(s)" | tee -a "$TRIAGE"
  echo "$CRASH_LINES" | sed 's/^/    /' | tee -a "$TRIAGE"
  PASS=0
else
  echo "✓ no exceptions in policy stdout" | tee -a "$TRIAGE"
fi

# Trial completion
TRIAL_END_COUNT=$(grep -c "ANT-DIAG event=trial_end" "$POLICY_LOG" 2>/dev/null || echo 0)
if [ "$TRIAL_END_COUNT" -ge 3 ]; then
  echo "✓ all 3 trials completed (trial_end count=${TRIAL_END_COUNT})" | tee -a "$TRIAGE"
else
  echo "✗ only ${TRIAL_END_COUNT}/3 trials completed" | tee -a "$TRIAGE"
  PASS=0
fi

# v25 code-path firing
BUG125_HITS=$(grep -c "BUG125" "$POLICY_LOG" 2>/dev/null || echo 0)
BUG126_HITS=$(grep -c "BUG126" "$POLICY_LOG" 2>/dev/null || echo 0)
BUG127_HITS=$(grep -c "BUG127" "$POLICY_LOG" 2>/dev/null || echo 0)
echo "  BUG125 hits=${BUG125_HITS} BUG126 hits=${BUG126_HITS} BUG127 hits=${BUG127_HITS}" | tee -a "$TRIAGE"
if [ "$((BUG126_HITS + BUG127_HITS))" -lt 1 ]; then
  echo "  (warning) no BUG126/BUG127 lines — joint-space code path may not have fired on T2" | tee -a "$TRIAGE"
fi

# Score parsing — eval container prints final scores to stdout
SCORE_BLOCK=$(grep -E "T[123].*[Ss]core|Total|Tier" "$EVAL_LOG" | tail -30)
if [ -n "$SCORE_BLOCK" ]; then
  echo "--- eval scoring excerpt ---" | tee -a "$TRIAGE"
  echo "$SCORE_BLOCK" | tee -a "$TRIAGE"
fi

echo "---" | tee -a "$TRIAGE"
if [ "$PASS" -eq 1 ]; then
  echo "PASS: safe to run ./submit.sh ${DEFAULT_TAG}" | tee -a "$TRIAGE"
  exit 0
else
  echo "FAIL: do NOT submit — fix issues above first" | tee -a "$TRIAGE"
  exit 1
fi
