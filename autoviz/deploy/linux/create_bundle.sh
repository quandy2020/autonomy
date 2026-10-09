#!/usr/bin/env bash
# Stage a relocatable AppDir and pack it as a .tar.gz.
#
# Project + vendored Ogre libraries live under AppDir/lib/autoviz.
# System Qt / protobuf / etc. remain on the host (same Ubuntu release).
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

log "Stage AppDir → ${APPDIR}"
rm -rf "${APPDIR}"
mkdir -p "${APPDIR}"
stage_autoviz_prefix "${APPDIR}" "${BUILD_DIR}" bundle

install -m 0755 "${SCRIPT_DIR}/AppRun" "${APPDIR}/AppRun"

# AppImage-style root desktop + icon (optional convenience).
if [[ -f "${APPDIR}/share/applications/org.autonomy.autoviz.desktop" ]]; then
  install -m 0644 "${APPDIR}/share/applications/org.autonomy.autoviz.desktop" \
    "${APPDIR}/autoviz.desktop"
fi
# Prefer squirrel PNG (same as macOS); scalable SVG only if present.
if [[ -f "${APPDIR}/share/icons/hicolor/256x256/apps/aviz.png" ]]; then
  install -m 0644 "${APPDIR}/share/icons/hicolor/256x256/apps/aviz.png" \
    "${APPDIR}/aviz.png"
elif [[ -f "${APPDIR}/share/icons/hicolor/scalable/apps/aviz.svg" ]]; then
  install -m 0644 "${APPDIR}/share/icons/hicolor/scalable/apps/aviz.svg" \
    "${APPDIR}/aviz.svg"
fi

mkdir -p "${OUT_DIR}"
log "Pack ${ARCHIVE}"
tar -C "${OUT_DIR}" -czf "${ARCHIVE}" AppDir

log "Done: ${ARCHIVE}"
log "Run: tar -xzf ${ARCHIVE} && ./AppDir/AppRun"
