#!/usr/bin/env bash
# Copyright 2026 The Autonomy Authors
#
# Build and pack an install-tree tarball for ansible artifact deployment.
#
# Usage:
#   ./scripts/package_autonomy_artifact.sh
#   ./scripts/package_autonomy_artifact.sh --output /tmp/autonomy.tar.gz
#
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/opt/autonomy}"
OUTPUT="${OUTPUT:-${ROOT_DIR}/dist/autonomy-$(git -C "${ROOT_DIR}" describe --tags --always 2>/dev/null || echo dev).tar.gz}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
BUILD_GRPC="${BUILD_GRPC:-ON}"
BUILD_TEST="${BUILD_TEST:-OFF}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output) OUTPUT="$2"; shift 2 ;;
    --prefix) INSTALL_PREFIX="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,12p' "$0"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "$(dirname "${OUTPUT}")"
mkdir -p "${BUILD_DIR}"

STAGE="$(mktemp -d)"
cleanup() { rm -rf "${STAGE}"; }
trap cleanup EXIT

echo "[package] configure: ${BUILD_DIR}"
cmake -G Ninja -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_GRPC="${BUILD_GRPC}" \
  -DBUILD_TEST="${BUILD_TEST}"

echo "[package] build + stage install under ${STAGE}"
DESTDIR="${STAGE}" ninja -C "${BUILD_DIR}" -j"${JOBS}" install

TAR_NAME="$(basename "${INSTALL_PREFIX}")"
TAR_PARENT="$(dirname "${INSTALL_PREFIX}")"
tar -C "${STAGE}${TAR_PARENT}" -czf "${OUTPUT}" "${TAR_NAME}"

echo "[package] wrote ${OUTPUT}"
