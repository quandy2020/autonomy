#!/usr/bin/env bash
# Build Atla2 targets inside the autonomy workspace.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)"
BUILD="${BUILD_DIR:-${ROOT}/build}"

TARGETS=(
  autonomy.localization.atla2_offline
  autonomy.localization.atla2_node
  autonomy.localization.atla2_benchmark
)

echo "build dir: ${BUILD}"
if [[ ! -d "${BUILD}" ]]; then
  echo "missing build dir; configure the autonomy workspace first" >&2
  exit 1
fi

ninja -C "${BUILD}" "${TARGETS[@]}"
echo "ok: ${TARGETS[*]}"
