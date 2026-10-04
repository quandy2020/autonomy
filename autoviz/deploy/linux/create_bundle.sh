#!/usr/bin/env bash
# Stage a relocatable AppDir and pack it as a .tar.gz.
#
# The bundle keeps project libraries under lib/ and uses AppRun to set
# AUTOVIZ_* paths. Qt and other system libraries stay on the host
# (install the same Ubuntu release's runtime packages).
#
# Usage:
#   ./deploy/linux/create_bundle.sh
#   ./deploy/linux/create_bundle.sh --build-dir build --output dist/linux

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"

usage() {
  cat <<'EOF'
Usage: create_bundle.sh [--build-dir DIR] [--output DIR]
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output) OUT_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

require_linux
[[ -x "${BUILD_DIR}/bin/autoviz" ]] || die "missing ${BUILD_DIR}/bin/autoviz"

VERSION="$(autoviz_version)"
ARCH="$(uname -m)"
APPDIR="${OUT_DIR}/AppDir"
ARCHIVE="${OUT_DIR}/autoviz-${VERSION}-${ARCH}.tar.gz"

log "Install into ${APPDIR}"
rm -rf "${APPDIR}"
mkdir -p "${APPDIR}"
cmake --install "${BUILD_DIR}" --prefix "${APPDIR}"

install -m 0755 "${SCRIPT_DIR}/AppRun" "${APPDIR}/AppRun"

# AppImage-style root entries (also useful for a desktop launcher).
if [[ -f "${BUILD_DIR}/org.autonomy.autoviz.desktop" ]]; then
  install -m 0644 "${BUILD_DIR}/org.autonomy.autoviz.desktop" \
    "${APPDIR}/autoviz.desktop"
fi
ICON_SRC=""
for candidate in \
  "${AUTOVIZ_ROOT}/resources/icons/aviz.svg" \
  "${AUTOVIZ_ROOT}/resources/icons/aviz.png" \
  "${SCRIPT_DIR}/autoviz.svg"; do
  if [[ -f "${candidate}" ]]; then
    ICON_SRC="${candidate}"
    break
  fi
done
if [[ -n "${ICON_SRC}" ]]; then
  ext="${ICON_SRC##*.}"
  install -m 0644 "${ICON_SRC}" "${APPDIR}/autoviz.${ext}"
fi

mkdir -p "${OUT_DIR}"
log "Pack ${ARCHIVE}"
tar -C "${OUT_DIR}" -czf "${ARCHIVE}" AppDir

log "Done: ${ARCHIVE}"
log "Run: tar -xzf ${ARCHIVE} && ./AppDir/AppRun"
