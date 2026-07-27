#!/bin/bash
# Autoviz Docker build entrypoint — configures and builds via autoviz/tools/.
set -euo pipefail

. /usr/local/lib/autoviz/build-type.sh

BUILD_TYPE="${1:-${BUILD_TYPE:-Release}}"
validate_build_type "${BUILD_TYPE}" || exit 1

AUTOVIZ_ROOT="/project/source/autoviz"
if [[ ! -f "${AUTOVIZ_ROOT}/tools/configure.py" ]]; then
    echo "Error: Autoviz tools not found at ${AUTOVIZ_ROOT}" >&2
    echo "Mount the Autonomy source tree (with autoviz/) at /project/source." >&2
    exit 1
fi

if [[ -n "${JOBS:-}" ]]; then
    BUILD_JOBS=(-j "${JOBS}")
else
    BUILD_JOBS=(--parallel)
fi

CONFIGURE_FLAGS=()
if [[ "${BUILD_TYPE}" == "Release" ]]; then
    CONFIGURE_FLAGS=(--release)
elif [[ "${BUILD_TYPE}" != "Debug" ]]; then
    CONFIGURE_FLAGS=(-DCMAKE_BUILD_TYPE="${BUILD_TYPE}")
fi

# AUTOVIZ_EXTRA_CONFIGURE: space-separated flags for configure.py (e.g. "--qml --ogre --tests")
if [[ -n "${AUTOVIZ_EXTRA_CONFIGURE:-}" ]]; then
    # shellcheck disable=SC2206
    extra=(${AUTOVIZ_EXTRA_CONFIGURE})
    CONFIGURE_FLAGS+=("${extra[@]}")
fi

if [[ "${CLEAN_BUILD:-0}" == "1" ]]; then
    echo "==> CLEAN_BUILD=1: removing ${AUTOVIZ_ROOT}/build"
    rm -rf "${AUTOVIZ_ROOT}/build"
fi

echo "==> Configure Autoviz (${BUILD_TYPE})"
python3 "${AUTOVIZ_ROOT}/tools/configure.py" "${CONFIGURE_FLAGS[@]}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

echo "==> Build autoviz"
if [[ -n "${JOBS:-}" ]]; then
    python3 "${AUTOVIZ_ROOT}/tools/build.py" --jobs "${JOBS}"
else
    python3 "${AUTOVIZ_ROOT}/tools/build.py" --jobs "$(nproc)"
fi

if [[ "${AUTOVIZ_INSTALL:-0}" == "1" ]]; then
    BUILD_DIR="${AUTOVIZ_ROOT}/build"
    PREFIX="${AUTOVIZ_INSTALL_PREFIX:-/project/install}"
    echo "==> Install to ${PREFIX}"
    cmake --install "${BUILD_DIR}" --prefix "${PREFIX}"
fi

echo "Build complete: ${AUTOVIZ_ROOT}/build"
