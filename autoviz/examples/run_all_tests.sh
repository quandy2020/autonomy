#!/usr/bin/env bash
# Run BICMap example catalog verification + headless display gtests.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-${ROOT}/build}"
TEST_BIN="${BUILD}/bin/autoviz_bicmap_examples_test"

python3 "${ROOT}/examples/verify_catalog.py"

if [[ ! -x "${TEST_BIN}" ]]; then
  echo "Missing ${TEST_BIN}" >&2
  echo "Configure and build with:" >&2
  echo "  python3 tools/configure.py --tests && cmake --build ${BUILD} --target autoviz_bicmap_examples_test" >&2
  exit 1
fi

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export DISPLAY="${DISPLAY:-:1}"
"${TEST_BIN}" "$@"
