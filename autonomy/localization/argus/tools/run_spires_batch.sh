#!/usr/bin/env bash
# Run LIVO/LIO/LO + modes compare for each Oxford Spires sequence (download separate).
#
# Usage:
#   ./scripts/run_spires_batch.sh [max_frames] SEQ1 [SEQ2 ...]
#   max_frames: optional int (0=all). If first arg is not a number, all args are seqs.
#
# Env:
#   ARGUS_MODES / ARGUS_FORCE_RERUN / ARGUS_SPIRES_* / ARGUS_LIVO_CONFIG
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

DATA="${SPIRES_DEST}"
BIN="${SPIRES_BIN}"
CONFIG="${LIVO_CONFIG}"
CALIB="${SPIRES_CALIB}"
MODES="${ARGUS_MODES:-livo lio lo}"

MAX_FRAMES=0
if [[ $# -ge 1 && "$1" =~ ^[0-9]+$ ]]; then
  MAX_FRAMES="$1"
  shift
fi
if [[ $# -lt 1 ]]; then
  echo "usage: $0 [max_frames] SEQ1 [SEQ2 ...]"
  exit 1
fi

argus_need "$BIN"
argus_need "$CONFIG"
argus_need "$CALIB/cam0.yaml"

SUMMARY="$DATA/argus_batch_summary_$(date +%Y%m%d_%H%M%S).txt"
echo "BIN=$BIN max_frames=$MAX_FRAMES" | tee "$SUMMARY"
echo "modes=$MODES" | tee -a "$SUMMARY"

run_mode() {
  local seq_dir="$1" mode="$2" out="$3"
  mkdir -p "$out"
  if [[ -f "$out/estimate.tum" && "${ARGUS_FORCE_RERUN:-0}" != "1" ]]; then
    local n
    n=$(wc -l < "$out/estimate.tum" | tr -d ' ')
    if [[ "$n" -gt 10 ]]; then
      echo "  skip $mode (estimate.tum lines=$n)"
      return 0
    fi
  fi
  echo "  run $mode → $out"
  "$BIN" \
    --spires "$seq_dir" \
    --calib "$CALIB" \
    --config "$CONFIG" \
    --camera-yaml "${ARGUS_CAMERA_YAML:-$seq_dir/camera_equidistant.yaml}" \
    --out-dir "$out" \
    --mode "$mode" \
    --max-frames "$MAX_FRAMES" \
    --camera cam0 \
    >"$out/run.log" 2>&1
  grep -E 'done processed' "$out/run.log" | tail -1 || true
}

for SEQ in "$@"; do
  SEQ_DIR="$DATA/sequences/$SEQ"
  echo
  echo "======== $SEQ ========"
  argus_need "$SEQ_DIR/raw/imu.csv"
  python3 "${SCRIPT_DIR}/prepare_spires_gt.py" "$SEQ_DIR"
  argus_ensure_camera_yaml "$SEQ_DIR" >/dev/null

  MODES_OUT="$SEQ_DIR/argus_eval/modes"
  mkdir -p "$MODES_OUT"
  for m in $MODES; do
    run_mode "$SEQ_DIR" "$m" "$MODES_OUT/$m"
  done

  GT="$SEQ_DIR/processed/vilens-slam/gt.tum"
  PNG="$MODES_OUT/modes_compare.png"
  EST_ARGS=()
  for m in $MODES; do
    argus_need "$MODES_OUT/$m/estimate.tum"
    EST_ARGS+=(--est "$m=$MODES_OUT/$m/estimate.tum")
  done
  python3 "${SCRIPT_DIR}/plot_modes_compare.py" \
    --gt "$GT" \
    "${EST_ARGS[@]}" \
    --out "$PNG" \
    --max-dt 1.0 \
    --title "Argus LIVO / LIO / LO vs VILENS (${SEQ})" \
    | tee "$MODES_OUT/ate_modes.txt"
  cp -f "$PNG" "$SEQ_DIR/argus_eval/modes_compare.png"
  {
    echo "---- $SEQ ----"
    cat "$MODES_OUT/ate_modes.txt"
    echo "PNG=$PNG"
  } | tee -a "$SUMMARY"
done

echo
echo "SUMMARY=$SUMMARY"
cat "$SUMMARY"
