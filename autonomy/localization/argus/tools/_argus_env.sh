#!/usr/bin/env bash
# Shared paths / helpers for Argus scripts (source me; do not execute).
#
# Naming convention (see README.md):
#   download_spires_*.sh | prepare_spires_*.py | convert_*.py
#   run_spires_*.sh      | plot_*.py           | viz_open3d_*.py

# shellcheck disable=SC2034
ARGUS_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARGUS_ROOT="$(cd "${ARGUS_SCRIPTS_DIR}/.." && pwd)"
ARGUS_REPO="$(cd "${ARGUS_ROOT}/../../.." && pwd)"

SPIRES_DEST="${ARGUS_SPIRES_DEST:-${HOME}/Downloads/oxford_spires_dataset}"
SPIRES_SEQ="${ARGUS_SPIRES_SEQ:-2024-07-09-new-college-01}"
SPIRES_CALIB="${ARGUS_SPIRES_CALIB:-${SPIRES_DEST}/calibration}"
SPIRES_BIN="${ARGUS_SPIRES_BIN:-${ARGUS_ROOT}/build_spires/offline_spires_livo}"
# Default to Spires DiffCal YAML so offline scripts match live ArgusNode.
LIVO_CONFIG="${ARGUS_LIVO_CONFIG:-${ARGUS_ROOT}/config/livo_esikf_spires.yaml}"
HF_BIN="${HF_BIN:-hf}"

argus_need() {
  [[ -e "$1" ]] || {
    echo "missing: $1" >&2
    exit 1
  }
}

argus_unzip_if_needed() {
  local zip="$1" dir="$2" marker="$3"
  if [[ -d "$dir/$marker" ]] && [[ -n "$(ls -A "$dir/$marker" 2>/dev/null || true)" ]]; then
    echo "[skip unzip] $dir/$marker"
    return 0
  fi
  if [[ ! -f "$zip" ]]; then
    echo "[WARN] missing $zip" >&2
    return 1
  fi
  echo "[unzip] $zip → $dir"
  mkdir -p "$dir"
  unzip -n -q "$zip" -d "$dir"
}

argus_link_spires_raw_layout() {
  local raw="$1"
  if [[ ! -d "$raw/cam0" && -d "$raw/images/cam0" ]]; then
    ln -sfn images/cam0 "$raw/cam0"
    ln -sfn images/cam1 "$raw/cam1"
    ln -sfn images/cam2 "$raw/cam2"
  fi
  if [[ ! -d "$raw/lidar-clouds" && -d "$raw/lidar_clouds" ]]; then
    ln -sfn lidar_clouds "$raw/lidar-clouds"
  fi
}

argus_ensure_camera_yaml() {
  local seq_dir="$1"
  local cam_yaml="${ARGUS_CAMERA_YAML:-${seq_dir}/camera_equidistant.yaml}"
  if [[ ! -f "$cam_yaml" ]]; then
    echo "==> generate $cam_yaml"
    python3 "${ARGUS_SCRIPTS_DIR}/convert_kalibr_camera.py" \
      --kalibr "${SPIRES_CALIB}/cam0.yaml" --out "$cam_yaml"
  fi
  echo "$cam_yaml"
}

# Prefer a python that has open3d when needed; else python3.
argus_python() {
  if [[ -n "${ARGUS_PYTHON:-}" ]]; then
    echo "${ARGUS_PYTHON}"
    return 0
  fi
  local py
  for py in python3 python; do
    command -v "$py" >/dev/null 2>&1 || continue
    if "$py" -c "import open3d" 2>/dev/null; then
      echo "$py"
      return 0
    fi
  done
  echo "python3"
}
