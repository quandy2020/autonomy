#!/usr/bin/env bash
# Run the Autoviz core / headless-UI regression tests with an isolated result
# directory. Stale ctest XMLs from other packages never enter the summary.
#
# Usage:
#   src/autonomy/autoviz/scripts/run_autoviz_tests.sh [RESULT_DIR]
#
# Environment:
#   AUTOVIZ_BUILD_DIR  CMake build tree (default: <package>/build)
#   AUTOVIZ_JOBS       Parallel compile jobs (default: nproc)
#
# Exit status: 0 when every test passes, 1 on build / test failure.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
BUILD_DIR="${AUTOVIZ_BUILD_DIR:-${PKG_DIR}/build}"
RESULT_DIR="${1:-${BUILD_DIR}/autoviz-test-results}"
JOBS="${AUTOVIZ_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"

if [[ ! -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
  echo "No CMake build tree at ${BUILD_DIR}." >&2
  echo "Configure it with -DAUTOVIZ_BUILD_CORE_TESTS=ON first." >&2
  exit 1
fi

echo "== Autoviz tests =="
echo "build:  ${BUILD_DIR}"
echo "result: ${RESULT_DIR}"

cmake --build "${BUILD_DIR}" \
  --target autoviz_interaction_test autoviz_mosaic_restore_test \
  -j "${JOBS}"

rm -rf "${RESULT_DIR}"
mkdir -p "${RESULT_DIR}"

ctest --test-dir "${BUILD_DIR}" \
  -R '^autoviz_(interaction|mosaic_restore)_test$' \
  --output-on-failure \
  --output-junit "${RESULT_DIR}/junit.xml"

echo "JUnit: ${RESULT_DIR}/junit.xml"

