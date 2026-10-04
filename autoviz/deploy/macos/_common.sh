#!/usr/bin/env bash
# Shared helpers for Autoviz macOS deploy scripts.
# shellcheck shell=bash

set -euo pipefail

_macos_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOVIZ_ROOT="$(cd "${_macos_script_dir}/../.." && pwd)"

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
  printf '%s\n' "${AUTOVIZ_DIST_DIR:-${AUTOVIZ_ROOT}/dist/macos}"
}

find_qt_prefix() {
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    local part
    local IFS=';'
    for part in ${CMAKE_PREFIX_PATH}; do
      if [[ -x "${part}/bin/macdeployqt" ]]; then
        printf '%s\n' "${part}"
        return 0
      fi
    done
  fi
  local candidate
  for candidate in \
    /opt/homebrew/opt/qt@6 \
    /usr/local/opt/qt@6 \
    /opt/homebrew/opt/qt \
    /usr/local/opt/qt; do
    if [[ -x "${candidate}/bin/macdeployqt" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  if command -v macdeployqt >/dev/null 2>&1; then
    local bin
    bin="$(command -v macdeployqt)"
    printf '%s\n' "$(cd "$(dirname "${bin}")/.." && pwd)"
    return 0
  fi
  # Last resort — can be slow if Homebrew is locked.
  if command -v brew >/dev/null 2>&1; then
    local prefix
    prefix="$(brew --prefix qt@6 2>/dev/null || true)"
    if [[ -n "${prefix}" && -x "${prefix}/bin/macdeployqt" ]]; then
      printf '%s\n' "${prefix}"
      return 0
    fi
  fi
  return 1
}

log() { printf '==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }
