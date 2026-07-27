#!/usr/bin/env bash
# Build + run an Autoviz Docker builder image.
#
# Usage:
#   deploy/docker/run-docker.sh [variant] [build-type]
#     variant:    ubuntu (default) | ubuntu-2204 | ubuntu-2404
#     build-type: Release (default) | Debug | RelWithDebInfo | MinSizeRel
#
# Env:
#   IMAGE_NAME           Override image tag
#   AUTONOMY_BASE_IMAGE  Base for Dockerfile (pre-built Autonomy dev image)
#   AUTOVIZ_EXTRA_CONFIGURE Extra flags for tools/configure.py (e.g. "--qml --ogre")
#   CLEAN_BUILD          Set 1 to wipe autoviz/build before configure
#   JOBS                 Parallel build jobs
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Autonomy CMake root (parent of autoviz/)
SOURCE_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

VARIANT="${1:-ubuntu}"
BUILD_TYPE="${2:-Release}"

case "${VARIANT}" in
    ubuntu|ubuntu-2404)
        BASE_REF="${BASE_REF:-ubuntu:24.04}"
        IMAGE_NAME="${IMAGE_NAME:-autoviz-ubuntu-docker}"
        BUILD_ARGS=()
        ;;
    ubuntu-2204)
        BASE_REF="${BASE_REF:-ubuntu:22.04}"
        IMAGE_NAME="${IMAGE_NAME:-autoviz-ubuntu-2204-docker}"
        BUILD_ARGS=(--build-arg "APT_EXTRA=gcc-12 g++-12")
        ;;
    *)
        echo "Unknown variant: ${VARIANT}" >&2
        echo "Valid: ubuntu | ubuntu-2204 | ubuntu-2404" >&2
        exit 1
        ;;
esac

if [[ -n "${AUTONOMY_BASE_IMAGE:-}" ]]; then
    BUILD_ARGS+=(--build-arg "AUTONOMY_BASE_IMAGE=${AUTONOMY_BASE_IMAGE}")
fi

docker build \
    --file "${DOCKERFILE}" \
    --target dev \
    --build-arg "BASE_REF=${BASE_REF}" \
    ${BUILD_ARGS[@]+"${BUILD_ARGS[@]}"} \
    -t "${IMAGE_NAME}" \
    "${SOURCE_DIR}"

SOURCE_DIR="${SOURCE_DIR}" \
    "${SCRIPT_DIR}/_docker-exec.sh" "${IMAGE_NAME}" "${BUILD_TYPE}"

echo "Output: ${SOURCE_DIR}/autoviz/build"
