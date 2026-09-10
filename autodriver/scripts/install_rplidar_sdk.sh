#!/usr/bin/env bash
# Download, build, and install Slamtec rplidar_sdk for autodriver.
# Upstream: https://github.com/slamtec/rplidar_sdk
#
# Usage:
#   ./scripts/install_rplidar_sdk.sh
#   PREFIX=/usr/local ./scripts/install_rplidar_sdk.sh
#   ./scripts/install_rplidar_sdk.sh /path/to/existing/rplidar_sdk   # skip clone
#
# Env:
#   PREFIX           install prefix (default: /usr/local)
#   RPLIDAR_SDK_SRC  existing checkout (skip download)
#   RPLIDAR_SDK_REF  git ref to clone (default: master)
#   RPLIDAR_SDK_DIR  clone/cache directory (default: /tmp/rplidar_sdk-src)

set -euo pipefail

REPO_URL="${RPLIDAR_SDK_REPO:-https://github.com/slamtec/rplidar_sdk.git}"
REF="${RPLIDAR_SDK_REF:-master}"
PREFIX="${PREFIX:-/usr/local}"
CACHE_DIR="${RPLIDAR_SDK_DIR:-/tmp/rplidar_sdk-src}"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

download_sdk() {
  need_cmd git
  need_cmd make
  need_cmd g++

  if [[ -f "${CACHE_DIR}/sdk/include/sl_lidar.h" && -d "${CACHE_DIR}/.git" ]]; then
    echo "== Updating ${CACHE_DIR} (${REF}) =="
    git -C "${CACHE_DIR}" fetch --depth 1 origin "${REF}"
    git -C "${CACHE_DIR}" checkout -q FETCH_HEAD || \
      git -C "${CACHE_DIR}" checkout -q "${REF}"
  else
    echo "== Cloning ${REPO_URL} (${REF}) -> ${CACHE_DIR} =="
    rm -rf "${CACHE_DIR}"
    git clone --depth 1 --branch "${REF}" "${REPO_URL}" "${CACHE_DIR}"
  fi
  echo "${CACHE_DIR}"
}

resolve_src() {
  if [[ $# -ge 1 && -n "${1}" ]]; then
    echo "${1}"
    return
  fi
  if [[ -n "${RPLIDAR_SDK_SRC:-}" ]]; then
    echo "${RPLIDAR_SDK_SRC}"
    return
  fi
  download_sdk
}

SRC="$(resolve_src "${1:-}")"
if [[ ! -f "${SRC}/sdk/include/sl_lidar.h" ]]; then
  echo "rplidar_sdk source invalid: ${SRC}" >&2
  echo "Expected sdk/include/sl_lidar.h (see ${REPO_URL})" >&2
  exit 1
fi

echo "== Building rplidar_sdk in ${SRC} =="
make -C "${SRC}" -j"$(nproc)"

LIB=""
for cand in \
  "${SRC}/output/Linux/Release/libsl_lidar_sdk.a" \
  "${SRC}/output/Linux/Debug/libsl_lidar_sdk.a"; do
  if [[ -f "${cand}" ]]; then
    LIB="${cand}"
    break
  fi
done
if [[ -z "${LIB}" ]]; then
  echo "libsl_lidar_sdk.a not found under ${SRC}/output" >&2
  exit 1
fi

echo "== Installing to ${PREFIX} (may need sudo) =="
install_cmd() {
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

install_cmd mkdir -p "${PREFIX}/include" "${PREFIX}/lib"
install_cmd cp -a "${SRC}/sdk/include/." "${PREFIX}/include/"
install_cmd cp -f "${LIB}" "${PREFIX}/lib/libsl_lidar_sdk.a"

echo "Installed:"
echo "  ${PREFIX}/include/sl_lidar.h"
echo "  ${PREFIX}/lib/libsl_lidar_sdk.a"
echo "Rebuild autodriver with -DAUTODRIVER_WITH_RPLIDAR=ON"
