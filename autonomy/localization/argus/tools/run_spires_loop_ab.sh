#!/usr/bin/env bash
# A/B: baseline (--no-loop) vs loop+FPR+LTA on Oxford Spires.
#
# Usage:
#   ./scripts/run_spires_loop_ab.sh [mode] [max_frames]
#   ./scripts/run_spires_loop_ab.sh lio 400
#
# Env: ARGUS_SPIRES_* / ARGUS_EVAL_BUILD / ARGUS_FORCE_RERUN=1
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

MODE="${1:-lio}"
MAX_FRAMES="${2:-400}"
DATA="${SPIRES_DEST}"
SEQ_NAME="${SPIRES_SEQ}"
SEQ="${ARGUS_SPIRES_ROOT:-$DATA/sequences/$SEQ_NAME}"
CALIB="${SPIRES_CALIB}"
CAM_YAML="${ARGUS_CAMERA_YAML:-$SEQ/camera_equidistant.yaml}"
EVAL_ROOT="${ARGUS_EVAL_OUT:-$SEQ/argus_eval}/loop_ab"
BASE_CFG="${ARGUS_ROOT}/config/livo_esikf.yaml"
LOOP_CFG="${ARGUS_ROOT}/config/livo_esikf_loop.yaml"
OUT_DIR="${ARGUS_EVAL_BUILD:-/tmp/argus_spires_eval}"
BIN="${ARGUS_SPIRES_BIN:-$OUT_DIR/offline_spires_livo}"
GT="$SEQ/processed/vilens-slam/gt.tum"

mkdir -p "$EVAL_ROOT" "$OUT_DIR"

argus_need "$SEQ/raw/imu.csv"
argus_need "$CALIB/cam0.yaml"
argus_need "$BASE_CFG"
argus_need "$LOOP_CFG"
argus_need "$GT"

if [[ ! -f "$CAM_YAML" ]]; then
  CAM_YAML="$(argus_ensure_camera_yaml "$SEQ")"
fi

build_with_ceres() {
  local stub="$ARGUS_REPO/tools/argus/stub_include"
  local eigen_inc="${EIGEN_INC:-/opt/homebrew/include/eigen3}"
  local ocv_inc
  ocv_inc=$(ls -d /opt/homebrew/Cellar/opencv/*/include/opencv5 2>/dev/null | head -1)
  [[ -n "$ocv_inc" ]] || ocv_inc=/opt/homebrew/opt/opencv/include/opencv5
  local ocv_lib=/opt/homebrew/opt/opencv/lib
  local ceres_inc=/opt/homebrew/opt/ceres-solver/include
  local ceres_lib=/opt/homebrew/opt/ceres-solver/lib
  local glog_inc=/opt/homebrew/opt/glog/include
  local glog_lib=/opt/homebrew/opt/glog/lib
  local ocv_calib=opencv_calib
  [[ -e "$ocv_lib/libopencv_calib.dylib" ]] || ocv_calib=opencv_calib3d
  local ocv_extra=()
  [[ -e "$ocv_lib/libopencv_geometry.dylib" ]] && ocv_extra+=(-lopencv_geometry)

  echo "==> compile offline_spires_livo (+Ceres) → $BIN"
  # shellcheck disable=SC2046
  c++ -std=c++17 -O2 -DNDEBUG \
    -DARGUS_HAS_CERES=1 -DGLOG_USE_GLOG_EXPORT \
    -I"$ARGUS_ROOT" -I"$stub" -I"$eigen_inc" -I"$ocv_inc" \
    -I"$ceres_inc" -I"$glog_inc" \
    -pthread \
    -Wno-unused-variable -Wno-unused-but-set-variable -Wno-deprecated-declarations \
    $(find "$ARGUS_ROOT/argus" -name '*.cpp' | sort) \
    "$ARGUS_ROOT/apps/offline_spires_livo.cpp" \
    "$ARGUS_ROOT/apps/datasets/pcd_io.cpp" \
    "$ARGUS_ROOT/apps/datasets/oxford_spires_options.cpp" \
    "$ARGUS_ROOT/apps/datasets/oxford_spires_dataset.cpp" \
    -L"$ocv_lib" -Wl,-rpath,"$ocv_lib" \
    -L"$ceres_lib" -Wl,-rpath,"$ceres_lib" \
    -L"$glog_lib" -Wl,-rpath,"$glog_lib" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -l"$ocv_calib" \
    "${ocv_extra[@]}" \
    -lceres -lglog -pthread \
    -o "$BIN"
}

need_rebuild=0
if [[ ! -x "$BIN" ]]; then
  need_rebuild=1
elif ! "$BIN" --help 2>&1 | grep -q '\-\-loop'; then
  echo "==> binary missing --loop; rebuild"
  need_rebuild=1
fi
if [[ "$need_rebuild" == "1" || "${ARGUS_FORCE_REBUILD:-0}" == "1" ]]; then
  build_with_ceres
fi
argus_need "$BIN"

run_one() {
  local tag="$1"
  local cfg="$2"
  shift 2
  local out="$EVAL_ROOT/$tag"
  mkdir -p "$out"
  local est="$out/estimate.tum"
  if [[ -f "$est" && "${ARGUS_FORCE_RERUN:-0}" != "1" ]]; then
    local n
    n=$(wc -l < "$est" | tr -d ' ')
    if [[ "$n" -gt 10 ]]; then
      echo "==> skip $tag (existing est lines=$n)"
      return 0
    fi
  fi
  echo "==> run $tag mode=$MODE max_frames=$MAX_FRAMES"
  # shellcheck disable=SC2068
  "$BIN" \
    --spires "$SEQ" \
    --calib "$CALIB" \
    --config "$cfg" \
    --camera-yaml "$CAM_YAML" \
    --out-dir "$out" \
    --mode "$MODE" \
    --max-frames "$MAX_FRAMES" \
    --camera cam0 \
    "$@" \
    2>&1 | tee "$out/run.log"
  argus_need "$est"
  local png="$out/gt_vs_estimate.png"
  python3 "${SCRIPT_DIR}/plot_traj_ate.py" \
    --gt "$GT" --est "$est" --out "$png" \
    --max-dt 1.0 \
    --title "Argus ${MODE} ${tag} (Spires ${SEQ_NAME}, N=${MAX_FRAMES})" \
    | tee "$out/ate.txt"
}

run_one baseline "$BASE_CFG" --no-loop
run_one loop_fpr "$LOOP_CFG" --loop

echo ""
echo "==== A/B summary ===="
for tag in baseline loop_fpr; do
  echo "-- $tag --"
  if [[ -f "$EVAL_ROOT/$tag/ate.txt" ]]; then
    cat "$EVAL_ROOT/$tag/ate.txt"
  fi
  if [[ -f "$EVAL_ROOT/$tag/run.log" ]]; then
    rg -n "loop stats|loop_closure.enable|synthetic_drift|done processed" \
      "$EVAL_ROOT/$tag/run.log" || true
  fi
done
echo "EVAL_ROOT=$EVAL_ROOT"
