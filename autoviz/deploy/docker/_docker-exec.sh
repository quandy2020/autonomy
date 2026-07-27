#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <image-name> [build-type]" >&2
    echo "  Env: SOURCE_DIR, BUILD_DIR (unused; build stays in autoviz/build), JOBS, CLEAN_BUILD" >&2
    exit 1
}

IMAGE_NAME="${1:-}"
BUILD_TYPE="${2:-Release}"
SOURCE_DIR="${SOURCE_DIR:-$(pwd)}"

[[ -z "${IMAGE_NAME}" ]] && usage

docker run \
    --rm \
    --user "$(id -u):$(id -g)" \
    --env HOME=/tmp \
    --env CLEAN_BUILD="${CLEAN_BUILD:-0}" \
    --env JOBS="${JOBS:-}" \
    --env AUTOVIZ_EXTRA_CONFIGURE="${AUTOVIZ_EXTRA_CONFIGURE:-}" \
    --env AUTOVIZ_INSTALL="${AUTOVIZ_INSTALL:-0}" \
    --env AUTOVIZ_INSTALL_PREFIX="${AUTOVIZ_INSTALL_PREFIX:-/project/install}" \
    -v "${SOURCE_DIR}:/project/source" \
    "${IMAGE_NAME}" \
    "${BUILD_TYPE}"
