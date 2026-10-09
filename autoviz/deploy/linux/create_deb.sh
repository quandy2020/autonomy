#!/usr/bin/env bash
# Build a .deb that installs Autoviz under /usr.
#
# Stages only Autoviz runtime files (binary, private libs, Ogre plugins,
# ogre_media, desktop/mime/icons) — not the whole Autonomy monorepo install.
#
# Usage:
#   ./deploy/linux/create_deb.sh
#   ./deploy/linux/create_deb.sh --build-dir /path/to/build/autonomy
#   sudo apt install ./dist/linux/autoviz_0.1.0_amd64.deb

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"
# Absolute paths so subshells (cd into package root) keep valid redirects.
BUILD_DIR="$(cd "${BUILD_DIR}" 2>/dev/null && pwd || printf '%s\n' "${BUILD_DIR}")"

usage() {
  cat <<'EOF'
Usage: create_deb.sh [options]

  --build-dir DIR   CMake build tree with bin/autoviz
                    (default: autoviz/build or <workspace>/build/autonomy)
  --output DIR      Output directory for .deb (default: dist/linux)
  -h, --help
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
command -v dpkg-deb >/dev/null 2>&1 || die "dpkg-deb not found (apt install dpkg-dev)"
[[ -x "${BUILD_DIR}/bin/autoviz" ]] || \
  die "missing ${BUILD_DIR}/bin/autoviz (build first, or pass --build-dir)"

VERSION="$(autoviz_version)"
ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"
PKG="autoviz_${VERSION}_${ARCH}"
mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd)"
ROOT="${OUT_DIR}/.deb-root/${PKG}"
DEBIAN="${ROOT}/DEBIAN"
USR="${ROOT}/usr"

log "Stage selective prefix → ${USR}"
rm -rf "${ROOT}"
mkdir -p "${DEBIAN}" "${USR}"
stage_autoviz_prefix "${USR}" "${BUILD_DIR}" deb

# Private libs live in /usr/lib/autoviz (wrapper + patchelf RPATH).
# Also register /etc/ld.so.conf.d so debuggers / ldd can resolve them.
mkdir -p "${ROOT}/etc/ld.so.conf.d"
printf '%s\n' "/usr/lib/autoviz" > "${ROOT}/etc/ld.so.conf.d/autoviz.conf"

# Portable Depends that work on both Ubuntu 22.04 (Jammy) and 24.04 (Noble).
# We do NOT emit raw dpkg-shlibdeps output: a .deb built in SpaceHero (22.04)
# must still configure on the host (24.04) via package-name alternatives.
# ABI-fragile libs (yaml/tinyxml/ffmpeg/glog/protobuf/assimp) are vendored.
RECOMMENDS="libqt6openglwidgets6 | libqt6openglwidgets6t64, libqt6svg6, libgl1, libegl1"
DEPENDS="libc6, \
libqt6core6t64 | libqt6core6, \
libqt6gui6t64 | libqt6gui6, \
libqt6widgets6t64 | libqt6widgets6, \
libqt6opengl6t64 | libqt6opengl6, \
libqt6openglwidgets6t64 | libqt6openglwidgets6, \
libqt6network6t64 | libqt6network6, \
libqt6svg6, \
libqt6xml6t64 | libqt6xml6, \
libgl1, \
libatomic1, \
libstdc++6, \
libgcc-s1 | libgcc1"
# Collapse whitespace from the heredoc-style list above.
DEPENDS="$(printf '%s' "${DEPENDS}" | tr -s '[:space:]' ' ' | sed 's/^ //; s/ $//')"
log "Using portable Qt/runtime Depends (22.04 + 24.04)"

INSTALLED_KB="$(du -sk "${USR}" | awk '{print $1}')"

cat > "${DEBIAN}/control" <<EOF
Package: autoviz
Version: ${VERSION}
Section: science
Priority: optional
Architecture: ${ARCH}
Maintainer: Autonomy <quandy2020@126.com>
Installed-Size: ${INSTALLED_KB}
Depends: ${DEPENDS}
Recommends: ${RECOMMENDS}
Homepage: https://github.com/autonomy/autonomy
Description: Autolink native 3D robot visualizer
 Desktop Qt application that renders Autolink channels (sensors, TF,
 robot models) without a ROS runtime. Sessions use .autoviz files.
 Vendored Ogre 1.12 plugins and media ship under /usr/lib/autoviz and
 /usr/share/autonomy/autoviz.
EOF

install -m 0755 "${SCRIPT_DIR}/debian/postinst" "${DEBIAN}/postinst"
install -m 0755 "${SCRIPT_DIR}/debian/postrm" "${DEBIAN}/postrm"
install -m 0644 "${SCRIPT_DIR}/debian/copyright" "${DEBIAN}/copyright"

{
  echo "autoviz (${VERSION}) unstable; urgency=medium"
  echo
  echo "  * Packaged Autoviz ${VERSION} for Ubuntu."
  echo
  echo " -- Autonomy <quandy2020@126.com>  $(date -R)"
} > "${DEBIAN}/changelog"

chmod 0755 "${DEBIAN}"
# Ensure maintainer scripts are executable even if umask was odd.
chmod 0755 "${DEBIAN}/postinst" "${DEBIAN}/postrm"

mkdir -p "${OUT_DIR}"
DEB="${OUT_DIR}/${PKG}.deb"
rm -f "${DEB}"
log "dpkg-deb ${DEB}"
dpkg-deb --root-owner-group -Zxz --build "${ROOT}" "${DEB}"

# Optional lintian (non-fatal).
if command -v lintian >/dev/null 2>&1; then
  log "lintian (advisory)"
  lintian --no-tag-display-limit "${DEB}" 2>&1 | head -40 || true
fi

log "Contents summary:"
dpkg-deb -c "${DEB}" | awk '{print $6}' \
  | grep -E 'bin/autoviz|lib/autoviz/[^/]+$|ogre_media|applications|mime|icons/hicolor' \
  | head -40 || true

log "Done: ${DEB}"
log "Install: sudo apt install ./${DEB##*/}   # from ${OUT_DIR}"
log "         or: sudo apt install ${DEB}"
