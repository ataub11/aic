#!/usr/bin/env bash
# b2a_yaw_sweep.sh — Sweep T3 SC gripper-yaw correction across candidate
# values, capturing each sim run's policy.log so Eng-4 can pick the
# best-performing value for v27 bake-off.
#
# Default sweep: -1.7133 ± 0.05 rad in 0.01 rad steps = 11 values.
# Per-sim runtime: ~3-5 minutes.  Total sweep budget: ~45-60 minutes.
#
# Usage:
#   ./scripts/b2a_yaw_sweep.sh [SIM_LAUNCHER]
#
# SIM_LAUNCHER defaults to "./scripts/local_sim.sh".  If your local sim
# wrapper is elsewhere, pass it explicitly:
#
#   ./scripts/b2a_yaw_sweep.sh ./tools/run_local_sim
#
# Each sim writes /tmp/ant_local_sim/policy.log; this script copies it
# into ant_policy_node/sim_runs/run_2026-05-09_b2a_sweep/log_<value>.log
# after each run so the sweep produces an auditable artifact set.

set -euo pipefail

SIM_LAUNCHER="${1:-./scripts/local_sim.sh}"
BASELINE="-1.7133"
SWEEP_DIR="ant_policy_node/sim_runs/run_2026-05-09_b2a_sweep"
mkdir -p "$SWEEP_DIR"

# Sweep values: baseline ± 0.05 rad in 0.01 steps (11 values total).
# bash floating-point arithmetic via awk.
VALUES=$(awk 'BEGIN{
  for (i=-5; i<=5; i++) printf "%.4f\n", -1.7133 + i*0.01
}')

echo "B2A yaw sweep starting at $(TZ=America/Los_Angeles date)"
echo "Sweep values: $(echo "$VALUES" | tr "\n" " ")"
echo "Sim launcher: $SIM_LAUNCHER"
echo "Output dir:   $SWEEP_DIR"
echo "---"

if [[ ! -x "$SIM_LAUNCHER" ]]; then
  echo "✗ Sim launcher not executable: $SIM_LAUNCHER"
  echo "  Pass the correct path as the first argument, or:"
  echo "  - Restore scripts/local_sim.sh (was reverted in commit 6949809)"
  echo "  - Or use whatever wrapper the build host uses"
  exit 1
fi

SUMMARY="$SWEEP_DIR/sweep_summary.tsv"
echo -e "yaw_rad\ttrial_count\tt1_score\tt2_score\tt3_score\tt3_dur_sec\tguard_violations\tnotes" > "$SUMMARY"

for v in $VALUES; do
  echo ""
  echo "=== Sim with ANT_T3_YAW_OVERRIDE_RAD=$v ==="
  ANT_T3_YAW_OVERRIDE_RAD="$v" "$SIM_LAUNCHER" || {
    echo "  Sim returned non-zero; continuing to next value"
  }

  LOG=/tmp/ant_local_sim/policy.log
  if [[ ! -f "$LOG" ]]; then
    echo -e "${v}\t0\t-\t-\t-\t-\t-\tno log produced" >> "$SUMMARY"
    continue
  fi

  cp "$LOG" "$SWEEP_DIR/log_${v}.log"

  # Extract metrics.  These greps are best-effort; if the local sim
  # reports scores differently, edit here.
  TRIAL_COUNT=$(grep -c "ANT-DIAG event=trial_end" "$LOG" || echo 0)
  T3_DUR=$(grep "ANT-DIAG event=trial_end trial=3" "$LOG" \
           | grep -oE "duration_sec=[0-9.]+" | head -1 \
           | awk -F= '{print $2}')
  T3_DUR="${T3_DUR:--}"
  GUARD=$(grep -c "joint_space_guard_violation" "$LOG" || echo 0)
  # Sim score extraction depends on local-sim implementation.  Leave
  # blank for Eng-4 to fill in manually from the log.
  echo -e "${v}\t${TRIAL_COUNT}\t-\t-\t-\t${T3_DUR}\t${GUARD}\t" >> "$SUMMARY"
  echo "  trial_end=$TRIAL_COUNT  T3_dur=$T3_DUR  guard_violations=$GUARD"
done

echo ""
echo "=== Sweep complete ==="
echo "Summary: $SUMMARY"
echo "Per-value logs: $SWEEP_DIR/log_*.log"
echo ""
echo "Next step: Eng-4 reads each log, fills T1/T2/T3 score columns in"
echo "$SUMMARY, picks best value, writes RESULTS.md with recommendation."
