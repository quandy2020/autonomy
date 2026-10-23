#!/usr/bin/env bash
# End-to-end: Oxford Spires sequence → offline odometry → Open3D viz (map + traj).
#
# Usage:
#   ./scripts/run_spires_open3d.sh [SEQ_NAME] [modes]
#   ./scripts/run_spires_open3d.sh 2024-07-09-new-college-01 livo,lio,lo
#   SKIP_RUN=1 ./scripts/run_spires_open3d.sh
#   SCREENSHOT=/tmp/o3d.png ./scripts/run_spires_open3d.sh
#
# Pipeline:
#   1) check raw + calib + GT
#   2) offline_spires_livo for each mode (unless SKIP_RUN=1)
#   3) matplotlib modes_compare.png
#   4) Open3D: map.xyz + Umeyama-aligned trajectories
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

DATA="${SPIRES_DEST}"
SEQ_NAME="${1:-$SPIRES_SEQ}"
MODES="${2:-${ARGUS_MODES:-livo,lio,lo}}"
SEQ_DIR="${ARGUS_SPIRES_ROOT:-$DATA/sequences/$SEQ_NAME}"
CALIB="${SPIRES_CALIB}"
CONFIG="${LIVO_CONFIG}"
BIN="${SPIRES_BIN}"
PY="$(argus_python)"
MAP_MODE="${ARGUS_MAP_MODE:-lio}"
VOXEL="${ARGUS_MAP_VOXEL:-0.25}"
SKIP_RUN="${SKIP_RUN:-0}"
MAX_FRAMES="${MAX_FRAMES:-0}"

echo "======== Argus Open3D pipeline ========"
echo "seq=$SEQ_NAME"
echo "seq_dir=$SEQ_DIR"
echo "modes=$MODES  map_mode=$MAP_MODE  skip_run=$SKIP_RUN"
echo "python=$PY"

echo
echo "[1/4] check dataset"
argus_need "$SEQ_DIR/raw/imu.csv"
argus_need "$SEQ_DIR/raw/lidar-clouds"
argus_need "$CALIB/cam0.yaml"
argus_need "$CONFIG"
"$PY" "${SCRIPT_DIR}/prepare_spires_gt.py" "$SEQ_DIR"
argus_ensure_camera_yaml "$SEQ_DIR" >/dev/null
argus_need "$SEQ_DIR/processed/vilens-slam/gt.tum"
echo "  OK raw + GT + camera yaml"

echo
echo "[2/4] offline odometry → estimate.tum + map/"
MODES_OUT="$SEQ_DIR/argus_eval/modes"
IFS=',' read -r -a MODE_ARR <<< "$MODES"
if [[ "$SKIP_RUN" != "1" ]]; then
  argus_need "$BIN"
  for mode in "${MODE_ARR[@]}"; do
    mode=$(echo "$mode" | tr -d ' ')
    out="$MODES_OUT/$mode"
    mkdir -p "$out"
    if [[ -f "$out/estimate.tum" && "${ARGUS_FORCE_RERUN:-0}" != "1" ]]; then
      n=$(wc -l < "$out/estimate.tum" | tr -d ' ')
      if [[ "$n" -gt 10 ]]; then
        echo "  skip $mode (estimate.tum lines=$n)"
        continue
      fi
    fi
    echo "  run mode=$mode"
    set +e
    "$BIN" \
      --spires "$SEQ_DIR" --calib "$CALIB" --config "$CONFIG" \
      --camera-yaml "$SEQ_DIR/camera_equidistant.yaml" \
      --out-dir "$out" --mode "$mode" --max-frames "$MAX_FRAMES" --camera cam0 \
      >"$out/run.log" 2>&1
    rc=$?
    set -e
    if [[ $rc -ne 0 ]]; then
      echo "  WARN: mode=$mode exited rc=$rc (see $out/run.log) — continue"
    fi
    grep -E 'done processed' "$out/run.log" | tail -1 || true
  done
else
  echo "  SKIP_RUN=1 — reuse existing estimates"
fi

HAVE_EST=0
for mode in "${MODE_ARR[@]}"; do
  mode=$(echo "$mode" | tr -d ' ')
  if [[ -f "$MODES_OUT/$mode/estimate.tum" ]]; then
    n=$(wc -l < "$MODES_OUT/$mode/estimate.tum" | tr -d ' ')
    if [[ "$n" -gt 10 ]]; then
      HAVE_EST=1
    fi
  else
    echo "  WARN: missing/short estimate for mode=$mode"
  fi
done
[[ "$HAVE_EST" -eq 1 ]] || {
  echo "no usable estimate.tum"
  exit 1
}

echo
echo "[3/4] matplotlib modes_compare.png"
EST_ARGS=()
for mode in "${MODE_ARR[@]}"; do
  mode=$(echo "$mode" | tr -d ' ')
  EST_ARGS+=(--est "$mode=$MODES_OUT/$mode/estimate.tum")
done
"$PY" "${SCRIPT_DIR}/plot_modes_compare.py" \
  --gt "$SEQ_DIR/processed/vilens-slam/gt.tum" \
  "${EST_ARGS[@]}" \
  --out "$MODES_OUT/modes_compare.png" \
  --max-dt 1.0 \
  --title "Argus vs VILENS (${SEQ_NAME})" \
  | tee "$MODES_OUT/ate_modes.txt"
echo "  PNG=$MODES_OUT/modes_compare.png"

echo
echo "[4/4] Open3D map + trajectories"
MAP_XYZ="$MODES_OUT/$MAP_MODE/map/map.xyz"
VIZ_ARGS=(
  --seq-dir "$SEQ_DIR"
  --modes "$MODES"
  --max-rmse "${ARGUS_VIZ_MAX_RMSE:-50}"
  --voxel "$VOXEL"
  --align-map-to-gt
)
if [[ -f "$MAP_XYZ" ]]; then
  VIZ_ARGS+=(--map "$MAP_XYZ")
  echo "  map=$MAP_XYZ"
else
  echo "  warn: no $MAP_XYZ — traj only (run mode=$MAP_MODE with mapping on)"
fi

if [[ -n "${SCREENSHOT:-}" ]]; then
  VIZ_ARGS+=(--screenshot "$SCREENSHOT")
  echo "  screenshot → $SCREENSHOT"
  "$PY" "${SCRIPT_DIR}/viz_open3d_traj.py" "${VIZ_ARGS[@]}"
else
  echo "  launching interactive Open3D (Q to quit)…"
  echo "  colors: GT=black  LIVO=red  LIO=blue  LO=purple  map=height"
  "$PY" "${SCRIPT_DIR}/viz_open3d_traj.py" "${VIZ_ARGS[@]}"
fi

echo
echo "======== done ========"
echo "ATE:  $MODES_OUT/ate_modes.txt"
echo "PNG:  $MODES_OUT/modes_compare.png"
echo "map:  $MAP_XYZ"
[[ -n "${SCREENSHOT:-}" ]] && echo "O3D:  $SCREENSHOT"
