#!/usr/bin/env bash
# Shared helpers for Autoviz Linux deploy scripts.
# shellcheck shell=bash

set -euo pipefail

_linux_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOVIZ_ROOT="$(cd "${_linux_script_dir}/../.." && pwd)"

autoviz_version() {
  local v
  v="$(sed -nE 's/^project\(autoviz VERSION ([0-9.]+).*/\1/p' \
    "${AUTOVIZ_ROOT}/CMakeLists.txt" | head -1)"
  printf '%s\n' "${v:-0.1.0}"
}

default_build_dir() {
  printf '%s\n' "${AUTOVIZ_BUILD_DIR:-${AUTOVIZ_ROOT}/build}"
}

default_dist_dir() {
  printf '%s\n' "${AUTOVIZ_DIST_DIR:-${AUTOVIZ_ROOT}/dist/linux}"
}

nproc_jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    printf '%s\n' 4
  fi
}

require_linux() {
  [[ "$(uname -s)" == "Linux" ]] || die "Linux only"
}

log() { printf '==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

apt_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    printf '%s\n' apt-get
  else
    command -v sudo >/dev/null 2>&1 || die "need root or sudo"
    printf '%s\n' "sudo apt-get"
  fi
}
