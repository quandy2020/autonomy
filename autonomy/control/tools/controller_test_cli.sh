#!/usr/bin/env bash
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Command-line integration tests for autonomy_controller_test.
#
# Usage (from build tree after ninja autonomy_controller_test):
#   ./autonomy/control/tools/controller_test_cli.sh
#
# Environment:
#   AUTONOMY_BUILD_DIR   - build directory (default: <repo>/build)
#   AUTONOMY_CONFIG_DIR  - config root (default: <repo>/config)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTONOMY_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY_ROOT}/build}"
CONFIG_DIR="${AUTONOMY_CONFIG_DIR:-${AUTONOMY_ROOT}/config}"
BIN="${BUILD_DIR}/bin/autonomy_controller_test"

PASS=0
FAIL=0
TMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/autonomy_controller_test_cli.XXXXXX")"
trap 'rm -rf "${TMP_ROOT}"' EXIT

log() { echo "[controller_test_cli] $*"; }
pass() { PASS=$((PASS + 1)); log "PASS: $*"; }
fail() { FAIL=$((FAIL + 1)); log "FAIL: $*"; }

require_file() {
  if [[ ! -f "$1" ]]; then
    fail "missing file: $1"
    return 1
  fi
  return 0
}

run_case() {
  local name="$1"
  shift
  log "=== ${name} ==="
  if "$@"; then
    pass "${name}"
  else
    fail "${name}"
  fi
}

if [[ ! -x "${BIN}" ]]; then
  echo "Executable not found: ${BIN}" >&2
  echo "Build first: cd build && ninja autonomy_controller_test" >&2
  exit 1
fi

require_file "${CONFIG_DIR}/control/controller.lua" || exit 1
require_file "${CONFIG_DIR}/planner/planner.lua" || exit 1

# --- 1. --help ---
run_case "help" bash -c "
  '${BIN}' --help 2>&1 | grep -q 'output_format' || exit 1
  exit 0
"

# --- 2. Synthetic map + graceful_controller ---
OUT_SYN="${TMP_ROOT}/synthetic"
run_case "synthetic_graceful" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --output_dir='${OUT_SYN}' \
    --use_synthetic_map=true \
    --add_demo_obstacles=false \
    --start_x=0.5 --start_y=0.5 \
    --goal_x=2.0 --goal_y=2.0 \
    --controllers=graceful_controller \
    --max_sim_steps=300 \
    --hold_frames=2 \
    --per_controller_video=true \
    --fps=5 \
    --skip_mppi=true
  test -f '${OUT_SYN}/graceful_controller_tracking.png'
  test -f '${OUT_SYN}/graceful_controller.mp4'
  test -f '${OUT_SYN}/run_summary.txt'
"

# --- 3. Auto-detect controllers (skip mppi for CI speed) ---
OUT_AUTO="${TMP_ROOT}/auto_detect"
run_case "auto_detect_controllers" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --output_dir='${OUT_AUTO}' \
    --use_synthetic_map=true \
    --add_demo_obstacles=false \
    --start_x=0.5 --start_y=0.5 \
    --goal_x=2.0 --goal_y=2.0 \
    --max_sim_steps=200 \
    --hold_frames=1 \
    --per_controller_video=false \
    --skip_mppi=true \
    --write_run_summary=true
  test -f '${OUT_AUTO}/reference_path.png'
  test -f '${OUT_AUTO}/run_summary.txt'
  grep -q 'controller=graceful_controller' '${OUT_AUTO}/run_summary.txt'
"

log "Results: ${PASS} passed, ${FAIL} failed"
if [[ "${FAIL}" -gt 0 ]]; then
  exit 1
fi
exit 0
