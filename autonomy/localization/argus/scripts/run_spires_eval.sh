#!/usr/bin/env bash
# Build (optional) + run offline_spires_livo + plot ATE vs VILENS GT.
#
# Usage:
#   ./scripts/run_spires_eval.sh [mode] [max_frames]
#   ./scripts/run_spires_eval.sh livo 0
#
# Env:
#   ARGUS_SPIRES_DEST / ARGUS_SPIRES_SEQ / ARGUS_SPIRES_ROOT / ARGUS_SPIRES_BIN
#   ARGUS_LIVO_CONFIG / ARGUS_CAMERA_YAML / ARGUS_EVAL_OUT / ARGUS_EVAL_BUILD
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
EVAL_OUT="${ARGUS_EVAL_OUT:-$SEQ/argus_eval}"
BUILD_DIR="${ARGUS_EVAL_BUILD:-$ARGUS_ROOT/build_spires}"
BIN="${SPIRES_BIN}"

MODE="${1:-livo}"
MAX_FRAMES="${2:-0}"

mkdir -p "$EVAL_OUT"

argus_need "$SEQ/raw/imu.csv"
argus_need "$CALIB/cam0.yaml"
argus_need "$CONFIG"

if [[ ! -f "$CAM_YAML" ]]; then
  CAM_YAML="$(argus_ensure_camera_yaml "$SEQ")"
fi

build_ad_hoc() {
  local stub="$ARGUS_REPO/tools/argus/stub_include"
  local out_dir="${ARGUS_EVAL_BUILD:-/tmp/argus_spires_eval}"
  mkdir -p "$out_dir"
  BIN="$out_dir/offline_spires_livo"

  local eigen_inc="${EIGEN_INC:-/opt/homebrew/include/eigen3}"
  local ocv_inc
  ocv_inc=$(ls -d /opt/homebrew/opt/opencv/include/opencv5 2>/dev/null | head -1)
  local ocv_lib=/opt/homebrew/opt/opencv/lib
  local glog_inc=/opt/homebrew/include
  local glog_lib=/opt/homebrew/lib
  local ocv_calib=opencv_calib
  [[ -e "$ocv_lib/libopencv_calib.dylib" ]] || ocv_calib=opencv_calib3d
  local ocv_extra=()
  [[ -e "$ocv_lib/libopencv_geometry.dylib" ]] && ocv_extra+=(-lopencv_geometry)

  echo "==> ad-hoc compile → $BIN"
  # shellcheck disable=SC2046
  c++ -std=c++17 -O2 -DNDEBUG \
    -I"$ARGUS_ROOT" -I"$stub" -I"$eigen_inc" -I"$ocv_inc" -I"$glog_inc" \
    -pthread \
    -Wno-unused-variable -Wno-unused-but-set-variable -Wno-deprecated-declarations \
    $(find "$ARGUS_ROOT/argus" -name '*.cpp' | sort) \
    "$ARGUS_ROOT/apps/offline_spires_livo.cpp" \
    "$ARGUS_ROOT/apps/datasets/pcd_io.cpp" \
    "$ARGUS_ROOT/apps/datasets/oxford_spires_options.cpp" \
    "$ARGUS_ROOT/apps/datasets/oxford_spires_dataset.cpp" \
    -L"$ocv_lib" -Wl,-rpath,"$ocv_lib" \
    -L"$glog_lib" -Wl,-rpath,"$glog_lib" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -l"$ocv_calib" \
    "${ocv_extra[@]}" \
    -lglog -pthread \
    -o "$BIN"
}

if [[ ! -x "$BIN" ]]; then
  if [[ -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "==> cmake --build $BUILD_DIR --target offline_spires_livo"
    cmake --build "$BUILD_DIR" --target offline_spires_livo \
      -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
    BIN="$BUILD_DIR/offline_spires_livo"
  else
    build_ad_hoc
  fi
fi
argus_need "$BIN"

echo "==> run mode=$MODE max_frames=$MAX_FRAMES bin=$BIN"
"$BIN" \
  --spires "$SEQ" \
  --calib "$CALIB" \
  --config "$CONFIG" \
  --camera-yaml "$CAM_YAML" \
  --out-dir "$EVAL_OUT" \
  --mode "$MODE" \
  --max-frames "$MAX_FRAMES" \
  --camera cam0

GT="$SEQ/processed/vilens-slam/gt.tum"
EST="$EVAL_OUT/estimate.tum"
PNG="$EVAL_OUT/gt_vs_estimate.png"
argus_need "$GT"
argus_need "$EST"

TITLE_MODE=$(printf '%s' "$MODE" | tr '[:lower:]' '[:upper:]')
echo "==> plot"
python3 "${SCRIPT_DIR}/plot_traj_ate.py" \
  --gt "$GT" --est "$EST" --out "$PNG" \
  --max-dt 1.0 \
  --title "Argus ${TITLE_MODE} vs VILENS (Oxford Spires ${SEQ_NAME})"

echo "PNG=$PNG"
echo "EST=$EST"
echo "GT=$GT"
ls -lah "$PNG" "$EST" "$GT"
