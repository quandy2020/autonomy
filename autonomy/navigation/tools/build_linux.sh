#!/usr/bin/env bash
# Build navigation on Linux with autolink components and Engine adapters.
# Usage: tools/build_linux.sh <automsgs-build-dir> [build-dir]
set -euo pipefail

AUTOMSGS_BUILD_DIR="${1:?usage: tools/build_linux.sh <automsgs-build-dir> [build-dir]}"
BUILD_DIR="${2:-build-linux}"

cmake -S . -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DNAVIGATION_BUILD_TESTS=ON \
  -DNAVIGATION_WITH_AUTOLINK=ON \
  -DNAVIGATION_WITH_ENGINE=ON \
  -DNAVIGATION_AUTOMSGS_BUILD_DIR="${AUTOMSGS_BUILD_DIR}"

cmake --build "${BUILD_DIR}" -j"$(nproc)"
ctest --test-dir "${BUILD_DIR}" --output-on-failure

echo "Linux build complete: ${BUILD_DIR}"
echo "Register Engine plugins before creating the runner:"
echo "  navigation::planning::RegisterEnginePlugins();"
