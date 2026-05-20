#!/usr/bin/env bash
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Command-line integration tests for autonomy_planning_test.
#
# Usage (from build tree after ninja autonomy_planning_test):
#   ./autonomy/planning/tools/planning_test_cli.sh
#
# Environment:
#   AUTONOMY_BUILD_DIR   - build directory (default: <repo>/build)
#   AUTONOMY_CONFIG_DIR  - config root (default: <repo>/config)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTONOMY_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY_ROOT}/build}"
CONFIG_DIR="${AUTONOMY_CONFIG_DIR:-${AUTONOMY_ROOT}/config}"
BIN="${BUILD_DIR}/bin/autonomy_planning_test"
MAP_YAML="${CONFIG_DIR}/data/map.yaml"

PASS=0
FAIL=0
TMP_ROOT="$(mktemp -d "${TMPDIR:-/tmp}/autonomy_planning_test_cli.XXXXXX")"
trap 'rm -rf "${TMP_ROOT}"' EXIT

log() { echo "[planning_test_cli] $*"; }
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
  echo "Build first: cd build && ninja autonomy_planning_test" >&2
  exit 1
fi

require_file "${CONFIG_DIR}/planner/planner.lua" || exit 1
require_file "${MAP_YAML}" || exit 1
require_file "${CONFIG_DIR}/data/map.pgm" || exit 1

# --- 1. --help ---
run_case "help" \
  "${BIN}" --help 2>&1 | grep -q "map_yaml"

# --- 2. Default map (config/data/map.pgm via map.yaml) ---
OUT_MAP="${TMP_ROOT}/map_pgm"
run_case "default_map_pgm" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --output_dir='${OUT_MAP}' \
    --start_x=30 --start_y=30 \
    --goal_x=370 --goal_y=370 \
    --hold_frames=2 \
    --per_planner_video=false \
    --fps=5
  test -f '${OUT_MAP}/navfn_planner.png'
  test -f '${OUT_MAP}/planning_all_planners.mp4'
"

# --- 3. Explicit map_yaml ---
OUT_EXPLICIT="${TMP_ROOT}/explicit_yaml"
run_case "explicit_map_yaml" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --map_yaml='${MAP_YAML}' \
    --output_dir='${OUT_EXPLICIT}' \
    --start_x=30 --start_y=30 \
    --goal_x=370 --goal_y=370 \
    --hold_frames=2 \
    --per_planner_video=false
  test -f '${OUT_EXPLICIT}/dijkstra_planner.png'
  test -f '${OUT_EXPLICIT}/planning_all_planners.mp4'
"

# --- 4. Synthetic costmap (planner.lua 20x20m grid) ---
OUT_SYN="${TMP_ROOT}/synthetic"
run_case "synthetic_map" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --use_synthetic_map=true \
    --add_demo_obstacles=false \
    --output_dir='${OUT_SYN}' \
    --start_x=0.5 --start_y=0.5 \
    --goal_x=19.0 --goal_y=19.0 \
    --hold_frames=2 \
    --per_planner_video=false
  test -f '${OUT_SYN}/theta_star_planner.png'
  test -f '${OUT_SYN}/planning_all_planners.mp4'
"

# --- 5. Single planner video output enabled ---
OUT_VIDEO="${TMP_ROOT}/per_planner_mp4"
run_case "per_planner_video" bash -c "
  '${BIN}' \
    --configuration_directory='${CONFIG_DIR}' \
    --use_synthetic_map=true \
    --add_demo_obstacles=false \
    --output_dir='${OUT_VIDEO}' \
    --start_x=1.0 --start_y=1.0 \
    --goal_x=18.0 --goal_y=18.0 \
    --hold_frames=2 \
    --per_planner_video=true
  test -f '${OUT_VIDEO}/navfn_planner.mp4'
"

log "Results: ${PASS} passed, ${FAIL} failed (tmp: ${TMP_ROOT})"
if [[ "${FAIL}" -gt 0 ]]; then
  exit 1
fi
exit 0
