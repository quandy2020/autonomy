#!/usr/bin/env bash
# Optional helper: build/install sherpa-onnx C API for autonomy audio ASR.
# Usage (inside SpaceHero / build container):
#   bash docker/install/install_sherpa_onnx.sh [/opt/sherpa-onnx]
set -euo pipefail

PREFIX="${1:-/usr/local}"
SRC_DIR="${SHERPA_ONNX_SRC:-/tmp/sherpa-onnx}"
REPO_URL="${SHERPA_ONNX_REPO:-https://github.com/k2-fsa/sherpa-onnx.git}"
TAG="${SHERPA_ONNX_TAG:-v1.10.46}"

echo "Installing sherpa-onnx ${TAG} → ${PREFIX}"
if [[ ! -d "${SRC_DIR}/.git" ]]; then
  git clone --depth 1 --branch "${TAG}" "${REPO_URL}" "${SRC_DIR}"
fi

cmake -S "${SRC_DIR}" -B "${SRC_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
  -DSHERPA_ONNX_ENABLE_C_API=ON \
  -DSHERPA_ONNX_ENABLE_PYTHON=OFF \
  -DSHERPA_ONNX_ENABLE_TESTS=OFF \
  -DSHERPA_ONNX_ENABLE_CHECK=OFF \
  -DSHERPA_ONNX_ENABLE_PORTAUDIO=OFF \
  -DSHERPA_ONNX_ENABLE_WEBSOCKET=OFF
cmake --build "${SRC_DIR}/build" -j"$(nproc)"
cmake --install "${SRC_DIR}/build"

echo "Done. Reconfigure autonomy with -DBUILD_SHERPA_ONNX=ON"
echo "  -DSherpaOnnx_ROOT=${PREFIX}   # if not in default paths"
