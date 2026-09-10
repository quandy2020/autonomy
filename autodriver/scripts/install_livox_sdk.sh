#!/usr/bin/env bash
# Download, build, and install Livox-SDK (v1) for autodriver.
# Upstream: https://github.com/Livox-SDK/Livox-SDK
#
# Usage:
#   ./scripts/install_livox_sdk.sh
#   PREFIX=/usr/local ./scripts/install_livox_sdk.sh

set -euo pipefail

REPO_URL="${LIVOX_SDK_REPO:-https://github.com/Livox-SDK/Livox-SDK.git}"
REF="${LIVOX_SDK_REF:-master}"
PREFIX="${PREFIX:-/usr/local}"
CACHE_DIR="${LIVOX_SDK_DIR:-/tmp/Livox-SDK-src}"

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 1
  }
}

download_sdk() {
  need_cmd git
  need_cmd cmake
  need_cmd make
  need_cmd g++

  if [[ -f "${CACHE_DIR}/sdk_core/include/livox_sdk.h" && -d "${CACHE_DIR}/.git" ]]; then
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
  if [[ -n "${LIVOX_SDK_SRC:-}" ]]; then
    echo "${LIVOX_SDK_SRC}"
    return
  fi
  download_sdk
}

SRC="$(resolve_src "${1:-}")"
if [[ ! -f "${SRC}/sdk_core/include/livox_sdk.h" ]]; then
  echo "Livox-SDK source invalid: ${SRC}" >&2
  exit 1
fi

BUILD_DIR="${SRC}/build"
echo "== Building Livox-SDK in ${BUILD_DIR} =="
# PIC required to link static .a into shared libautodriver.so.
cmake -S "${SRC}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "== Installing to ${PREFIX} (may need sudo) =="
install_cmd() {
  if [[ -w "${PREFIX}" ]] || [[ "$(id -u)" -eq 0 ]]; then
    "$@"
  else
    sudo "$@"
  fi
}

install_cmd cmake --install "${BUILD_DIR}" --prefix "${PREFIX}"

echo "Installed Livox-SDK under ${PREFIX}"
echo "Rebuild autodriver with -DAUTODRIVER_WITH_LIVOX=ON"
