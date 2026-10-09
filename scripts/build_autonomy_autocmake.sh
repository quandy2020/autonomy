#!/usr/bin/env bash
# Build autonomy via autocmake: full / distributed / incremental.
#
# The repo root has package.xml (name=autonomy). Sibling packages live in
# subdirectories; pass each as --base-path so discovery does not stop at root.
#
# Usage:
#   scripts/build_autonomy_autocmake.sh full
#   scripts/build_autonomy_autocmake.sh select automsgs autolink autonomy
#   scripts/build_autonomy_autocmake.sh up-to autonomy
#   scripts/build_autonomy_autocmake.sh autonomy
#
# Extra cmake args after -- :
#   scripts/build_autonomy_autocmake.sh full -- -DBUILD_AUTOVIZ=OFF

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOCMAKE="${ROOT}/autocmake/scripts/autocmake"
BUILD_BASE="${AUTONOMY_BUILD_BASE:-${ROOT}/build}"
INSTALL_BASE="${AUTONOMY_INSTALL_BASE:-${ROOT}/install}"

if [[ ! -f "${AUTOCMAKE}" ]]; then
  echo "missing ${AUTOCMAKE}" >&2
  exit 1
fi

# Peers that ship their own package.xml (order does not matter; topo sort does).
_BASE_PATHS=(--base-path "${ROOT}")
for _peer in automsgs autolink autodriver autoviz; do
  if [[ -f "${ROOT}/${_peer}/package.xml" ]]; then
    _BASE_PATHS+=(--base-path "${ROOT}/${_peer}")
  fi
done

mode="${1:-full}"
shift || true

cmake_args=(-DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}")
packages=()

case "${mode}" in
  full)
    packages=(--packages-up-to autonomy)
    ;;
  select)
    packages=(--packages-select)
    while [[ $# -gt 0 && "$1" != "--" ]]; do
      packages+=("$1")
      shift
    done
    ;;
  up-to)
    packages=(--packages-up-to)
    while [[ $# -gt 0 && "$1" != "--" ]]; do
      packages+=("$1")
      shift
    done
    ;;
  autonomy|automsgs|autolink|autoviz|autodriver)
    packages=(--packages-select "${mode}")
    ;;
  list)
    exec python3 "${AUTOCMAKE}" list "${_BASE_PATHS[@]}"
    ;;
  -h|--help)
    sed -n '1,25p' "$0"
    exit 0
    ;;
  *)
    echo "unknown mode: ${mode}" >&2
    exit 1
    ;;
esac

if [[ $# -gt 0 && "$1" == "--" ]]; then
  shift
  cmake_args+=("$@")
elif [[ $# -gt 0 ]]; then
  cmake_args+=("$@")
fi

exec python3 "${AUTOCMAKE}" build \
  "${_BASE_PATHS[@]}" \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  "${packages[@]}" \
  --cmake-args "${cmake_args[@]}"
