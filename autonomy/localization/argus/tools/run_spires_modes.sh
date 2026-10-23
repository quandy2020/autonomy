#!/usr/bin/env bash
# Run LIVO / LIO / LO on Oxford Spires and plot all trajectories on one figure.
#
# Usage:
#   ./scripts/run_spires_modes.sh [max_frames]
#
# Env:
#   ARGUS_MODES         default "livo lio lo"
#   ARGUS_FORCE_RERUN=1 redo even if estimate.tum exists
#   ARGUS_SPIRES_* / ARGUS_LIVO_CONFIG / ARGUS_SPIRES_BIN
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

DATA="${SPIRES_DEST}"
SEQ_NAME="${SPIRES_SEQ}"
SEQ="${ARGUS_SPIRES_ROOT:-$DATA/sequences/$SEQ_NAME}"
CALIB="${SPIRES_CALIB}"
CONFIG="${LIVO_CONFIG}"
CAM_YAML="${ARGUS_CAMERA_YAML:-$SEQ/camera_equidistant.yaml}"
EVAL_ROOT="${ARGUS_EVAL_OUT:-$SEQ/argus_eval}"
MODES_OUT="$EVAL_ROOT/modes"
CMAKE_BIN="$ARGUS_ROOT/build_spires/offline_spires_livo"
BIN="${ARGUS_SPIRES_BIN:-}"

MAX_FRAMES="${1:-0}"
MODES="${ARGUS_MODES:-livo lio lo}"

mkdir -p "$MODES_OUT"

argus_need "$SEQ/raw/imu.csv"
argus_need "$CALIB/cam0.yaml"
argus_need "$CONFIG"

if [[ ! -f "$CAM_YAML" ]]; then
  CAM_YAML="$(argus_ensure_camera_yaml "$SEQ")"
fi

bin_ok() {
  local b="$1"
  [[ -x "$b" ]] || return 1
  local out
  out=$("$b" 2>&1 | head -3 || true)
  echo "$out" | grep -q 'Library not loaded' && return 1
  echo "$out" | grep -q 'need --spires\|Usage' && return 0
  return 0
}

if [[ -z "$BIN" ]]; then
  if bin_ok "$CMAKE_BIN"; then
    BIN="$CMAKE_BIN"
  else
    echo "missing runnable offline_spires_livo (set ARGUS_SPIRES_BIN=...)"
    echo "hint: cmake build under $ARGUS_ROOT/build_spires"
    exit 1
  fi
fi
argus_need "$BIN"
echo "BIN=$BIN"

run_mode() {
  local mode="$1"
  local out="$MODES_OUT/$mode"
  mkdir -p "$out"
  local est="$out/estimate.tum"
  if [[ -f "$est" ]] && [[ "${ARGUS_FORCE_RERUN:-0}" != "1" ]]; then
    local n
    n=$(wc -l < "$est" | tr -d ' ')
    if [[ "$n" -gt 10 ]]; then
      echo "==> skip $mode (existing $est lines=$n); set ARGUS_FORCE_RERUN=1 to redo"
      return 0
    fi
  fi
  echo "==> run mode=$mode max_frames=$MAX_FRAMES → $out"
  "$BIN" \
    --spires "$SEQ" \
    --calib "$CALIB" \
    --config "$CONFIG" \
    --camera-yaml "$CAM_YAML" \
    --out-dir "$out" \
    --mode "$mode" \
    --max-frames "$MAX_FRAMES" \
    --camera cam0 \
    >"$out/run.log" 2>&1
  echo "    done $(grep -E 'done processed' "$out/run.log" | tail -1)"
}

for m in $MODES; do
  run_mode "$m"
done

GT="$SEQ/processed/vilens-slam/gt.tum"
PNG="$MODES_OUT/modes_compare.png"
ATE_TXT="$MODES_OUT/ate_modes.txt"
argus_need "$GT"

EST_ARGS=()
for m in $MODES; do
  argus_need "$MODES_OUT/$m/estimate.tum"
  EST_ARGS+=(--est "$m=$MODES_OUT/$m/estimate.tum")
done

echo "==> plot compare"
python3 "${SCRIPT_DIR}/plot_modes_compare.py" \
  --gt "$GT" \
  "${EST_ARGS[@]}" \
  --out "$PNG" \
  --max-dt 1.0 \
  --title "Argus LIVO / LIO / LO vs VILENS (${SEQ_NAME})" \
  | tee "$ATE_TXT"

echo "PNG=$PNG"
echo "ATE=$ATE_TXT"
ls -lah "$PNG" "$ATE_TXT"
for m in $MODES; do
  ls -lah "$MODES_OUT/$m/estimate.tum"
done
