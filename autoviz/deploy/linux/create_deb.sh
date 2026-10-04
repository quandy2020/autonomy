#!/usr/bin/env bash
# Build a .deb that installs under /usr (desktop file Exec=autoviz).
#
# Shared libraries of the project ship inside the package. Qt, protobuf,
# yaml-cpp, glog and friends are declared via dpkg-shlibdeps.
#
# Usage:
#   ./deploy/linux/create_deb.sh
#   sudo apt install ./dist/linux/autoviz_0.1.0_amd64.deb

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"

usage() {
  cat <<'EOF'
Usage: create_deb.sh [--build-dir DIR] [--output DIR]
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
command -v dpkg-deb >/dev/null 2>&1 || die "dpkg-deb not found"
[[ -x "${BUILD_DIR}/bin/autoviz" ]] || die "missing ${BUILD_DIR}/bin/autoviz"

VERSION="$(autoviz_version)"
ARCH="$(dpkg --print-architecture 2>/dev/null || uname -m)"
PKG="autoviz_${VERSION}_${ARCH}"
ROOT="${OUT_DIR}/${PKG}"
DEBIAN="${ROOT}/DEBIAN"

log "Stage ${ROOT}/usr"
rm -rf "${ROOT}"
mkdir -p "${DEBIAN}"
cmake --install "${BUILD_DIR}" --prefix "${ROOT}/usr"

shopt -s nullglob globstar
BINS=()
while IFS= read -r -d '' f; do
  BINS+=("${f#${ROOT}/}")
done < <(find "${ROOT}/usr" -type f \( -name 'autoviz' -o -name 'libautoviz.so*' -o -name 'libautolink.so*' -o -name 'libautomsgs.so*' \) -print0)
shopt -u nullglob globstar
[[ ${#BINS[@]} -gt 0 ]] || die "no binaries staged under ${ROOT}/usr"

DEPENDS="libc6"
if command -v dpkg-shlibdeps >/dev/null 2>&1; then
  log "Resolve shared library dependencies"
  (
    cd "${ROOT}"
    # dpkg-shlibdeps wants paths relative to the package root.
    # shellcheck disable=SC2068
    deps="$(dpkg-shlibdeps -O ${BINS[@]} 2>/dev/null || true)"
    printf '%s\n' "${deps}" > "${DEBIAN}/shlibs.substvars"
  )
  if [[ -s "${DEBIAN}/shlibs.substvars" ]]; then
    parsed="$(sed -n 's/^shlibs:Depends=//p' "${DEBIAN}/shlibs.substvars" | head -1)"
    if [[ -n "${parsed}" ]]; then
      DEPENDS="${parsed}"
    fi
  fi
  rm -f "${DEBIAN}/shlibs.substvars"
else
  log "dpkg-shlibdeps missing; Depends falls back to libc6"
fi

INSTALLED_KB="$(du -sk "${ROOT}/usr" | awk '{print $1}')"

cat > "${DEBIAN}/control" <<EOF
Package: autoviz
Version: ${VERSION}
Section: science
Priority: optional
Architecture: ${ARCH}
Maintainer: Autonomy <quandy2020@126.com>
Installed-Size: ${INSTALLED_KB}
Depends: ${DEPENDS}
Homepage: https://github.com/autonomy/autonomy
Description: Autolink native 3D robot visualizer
 Desktop Qt application that renders Autolink channels (sensors, TF,
 robot models) without a ROS runtime. Sessions use .autoviz files.
EOF

chmod 0755 "${DEBIAN}"
mkdir -p "${OUT_DIR}"
DEB="${OUT_DIR}/${PKG}.deb"
rm -f "${DEB}"
log "dpkg-deb ${DEB}"
dpkg-deb --root-owner-group --build "${ROOT}" "${DEB}"

log "Done: ${DEB}"
log "Install: sudo apt install ${DEB}"
