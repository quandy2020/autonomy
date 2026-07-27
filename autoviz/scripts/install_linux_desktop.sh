#!/usr/bin/env bash
# Install Aviz .desktop + hicolor icons for the current user (Ubuntu dock / app menu).
set -euo pipefail

AUTOVIZ_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTONOMY_ROOT="$(cd "${AUTOVIZ_PKG}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${AUTONOMY_ROOT}/../.." && pwd)"
BUILD="${BUILD_DIR:-${WORKSPACE_ROOT}/build/autoviz}"
PREFIX="${INSTALL_PREFIX:-${HOME}/.local}"

DESKTOP_SRC="${BUILD}/org.autonomy.autoviz.desktop"
ICON_SVG="${AUTOVIZ_PKG}/resources/icons/aviz.svg"

if [[ ! -f "${DESKTOP_SRC}" ]]; then
  echo "Missing ${DESKTOP_SRC}; configure/build autoviz first:" >&2
  echo "  cmake -S ${AUTOVIZ_PKG} -B ${BUILD} -DBUILD_AUTOVIZ=ON" >&2
  echo "  cmake --build ${BUILD} --target autoviz" >&2
  exit 1
fi

mkdir -p "${PREFIX}/share/applications"
mkdir -p "${PREFIX}/share/icons/hicolor/scalable/apps"
install -m 644 "${DESKTOP_SRC}" "${PREFIX}/share/applications/"

if [[ -f "${ICON_SVG}" ]]; then
  install -m 644 "${ICON_SVG}" "${PREFIX}/share/icons/hicolor/scalable/apps/aviz.svg"
fi

if command -v rsvg-convert >/dev/null 2>&1; then
  for size in 48 64 128 256; do
    dir="${PREFIX}/share/icons/hicolor/${size}x${size}/apps"
    mkdir -p "${dir}"
    rsvg-convert -w "${size}" -h "${size}" "${ICON_SVG}" -o "${dir}/aviz.png"
  done
fi

if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${PREFIX}/share/applications" 2>/dev/null || true
fi
if command -v gtk-update-icon-cache >/dev/null 2>&1; then
  gtk-update-icon-cache -f -t "${PREFIX}/share/icons/hicolor" 2>/dev/null || true
fi

echo "Installed Aviz desktop entry to ${PREFIX}/share/applications/"
echo "Restart autoviz from the app menu, or log out/in if the dock icon is unchanged."
