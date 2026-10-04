#!/usr/bin/env bash
# One-shot macOS configure → build → optional .app / .dmg.
#
# Usage:
#   ./deploy/macos/build.sh
#   ./deploy/macos/build.sh --release --app
#   ./deploy/macos/build.sh --release --app --dmg

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

RELEASE=0
MAKE_APP=0
MAKE_DMG=0
JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"
EXTRA_CMAKE=()

usage() {
  cat <<'EOF'
Usage: build.sh [options] [-- extra -D cmake flags]

  --release       CMAKE_BUILD_TYPE=Release
  --app           Run create_app_bundle.sh after build
  --dmg           Run create_dmg.sh (implies --app)
  --build-dir DIR CMake build directory
  --output DIR    dist directory for .app / .dmg
  -j N            Parallel build jobs (default: ncpu)
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
    --app) MAKE_APP=1; shift ;;
    --dmg) MAKE_APP=1; MAKE_DMG=1; shift ;;
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output) OUT_DIR="$2"; shift 2 ;;
    -j) JOBS="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    --) shift; EXTRA_CMAKE+=("$@"); break ;;
    *) EXTRA_CMAKE+=("$1"); shift ;;
  esac
done

[[ "$(uname -s)" == "Darwin" ]] || die "macOS only"

QT_PREFIX="$(find_qt_prefix)" || die "Qt 6 / macdeployqt not found (brew install qt@6)"
log "Qt prefix: ${QT_PREFIX}"

CONFIGURE=(
  cmake -S "${AUTOVIZ_ROOT}" -B "${BUILD_DIR}"
  -G Ninja
  "-DCMAKE_PREFIX_PATH=${QT_PREFIX}"
  "-DCMAKE_BUILD_TYPE=$([ "${RELEASE}" -eq 1 ] && echo Release || echo Debug)"
)
if [[ ${#EXTRA_CMAKE[@]} -gt 0 ]]; then
  CONFIGURE+=("${EXTRA_CMAKE[@]}")
fi

log "Configure"
"${CONFIGURE[@]}"

log "Build autoviz_app (-j${JOBS})"
cmake --build "${BUILD_DIR}" --target autoviz_app -j"${JOBS}"

BIN="${BUILD_DIR}/bin/autoviz"
[[ -x "${BIN}" ]] || die "missing ${BIN}"
file "${BIN}" | grep -q Mach-O || die "build produced non-Mach-O binary"
log "Binary OK: ${BIN}"
if [[ "${MAKE_APP}" -eq 1 ]]; then
  "${SCRIPT_DIR}/create_app_bundle.sh" --build-dir "${BUILD_DIR}" --output "${OUT_DIR}"
fi
if [[ "${MAKE_DMG}" -eq 1 ]]; then
  "${SCRIPT_DIR}/create_dmg.sh" --app "${OUT_DIR}/Autoviz.app" --output "${OUT_DIR}"
fi

log "Done. Run: ${BIN}"
if [[ "${MAKE_APP}" -eq 1 ]]; then
  log "App: ${OUT_DIR}/Autoviz.app"
fi
