#!/usr/bin/env bash
# Configure and build Autoviz on Ubuntu, optionally install / pack.
#
# Usage:
#   ./deploy/linux/install_deps.sh
#   ./deploy/linux/build.sh --release
#   ./deploy/linux/build.sh --release --prefix /opt/autoviz
#   ./deploy/linux/build.sh --release --bundle
#   ./deploy/linux/build.sh --release --deb

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

RELEASE=0
DO_BUNDLE=0
DO_DEB=0
PREFIX=""
JOBS="$(nproc_jobs)"
BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"
EXTRA_CMAKE=()

usage() {
  cat <<'EOF'
Usage: build.sh [options] [-- extra -D cmake flags]

  --release          CMAKE_BUILD_TYPE=Release
  --prefix DIR       cmake --install to DIR after build
  --bundle           Relocatable AppDir tarball (implies a staging install)
  --deb              Debian package via dpkg-deb
  --build-dir DIR    CMake build directory (default: ./build)
  --output DIR       dist directory (default: dist/linux)
  -j N               Parallel jobs (default: nproc)
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --release) RELEASE=1; shift ;;
    --ogre)
      log "note: --ogre is deprecated (Ogre viewport is always ON)"
      shift
      ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --bundle) DO_BUNDLE=1; shift ;;
    --deb) DO_DEB=1; shift ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output) OUT_DIR="$2"; shift 2 ;;
    -j) JOBS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_CMAKE+=("$@"); break ;;
    *) EXTRA_CMAKE+=("$1"); shift ;;
  esac
done

require_linux
command -v cmake >/dev/null 2>&1 || die "cmake not found (./deploy/linux/install_deps.sh)"

BUILD_TYPE="Debug"
if [[ "${RELEASE}" -eq 1 ]]; then
  BUILD_TYPE="Release"
fi

CONFIGURE=(
  cmake -S "${AUTOVIZ_ROOT}" -B "${BUILD_DIR}"
  -G Ninja
  "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
)
if [[ ${#EXTRA_CMAKE[@]} -gt 0 ]]; then
  CONFIGURE+=("${EXTRA_CMAKE[@]}")
fi

log "Configure (${BUILD_TYPE})"
"${CONFIGURE[@]}"

log "Build autoviz_app (-j${JOBS})"
cmake --build "${BUILD_DIR}" --target autoviz_app -j"${JOBS}"

BIN="${BUILD_DIR}/bin/autoviz"
[[ -x "${BIN}" ]] || die "missing ${BIN}"
file "${BIN}" | grep -q ELF || die "build produced non-ELF binary"
log "Binary OK: ${BIN}"

if [[ -n "${PREFIX}" ]]; then
  log "Install to ${PREFIX}"
  cmake --install "${BUILD_DIR}" --prefix "${PREFIX}"
fi
if [[ "${DO_BUNDLE}" -eq 1 ]]; then
  "${SCRIPT_DIR}/create_bundle.sh" --build-dir "${BUILD_DIR}" --output "${OUT_DIR}"
fi
if [[ "${DO_DEB}" -eq 1 ]]; then
  "${SCRIPT_DIR}/create_deb.sh" --build-dir "${BUILD_DIR}" --output "${OUT_DIR}"
fi

log "Done. Run: ${BIN}"
