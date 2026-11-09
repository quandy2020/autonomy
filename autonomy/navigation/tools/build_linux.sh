#!/usr/bin/env bash
# Build the autonomy umbrella and run its navigation tests in a colcon workspace.
# Usage: tools/build_linux.sh [workspace-dir] [jobs]
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${1:-$(cd -- "${SCRIPT_DIR}/../../../../.." && pwd)}"
BUILD_JOBS="${2:-8}"

cd "${WORKSPACE_DIR}"
colcon build --packages-up-to autonomy \
  --cmake-args -DNAVIGATION_BUILD_TESTS=ON
ctest --test-dir build/autonomy/autonomy/navigation \
  --output-on-failure --no-tests=error -j "${BUILD_JOBS}"

echo "Navigation build and tests complete: ${WORKSPACE_DIR}"
