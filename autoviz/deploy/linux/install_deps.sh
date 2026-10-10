#!/usr/bin/env bash
# Install Ubuntu build dependencies for Autoviz.
#
# Ubuntu 24.04 (Noble) is the supported baseline: stock Qt 6.4+ includes
# Qt6::OpenGLWidgets. Ubuntu 22.04 ships Qt 6.2, which cannot satisfy that
# component — use 24.04 or point CMAKE_PREFIX_PATH at Qt >= 6.4.
#
# Usage:
#   ./deploy/linux/install_deps.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ogre)
      echo "note: --ogre is deprecated (Ogre deps are always installed)"
      shift
      ;;
    -h|--help)
      echo "Usage: install_deps.sh"
      exit 0
      ;;
    *) die "unknown argument: $1" ;;
  esac
done

require_linux
command -v apt-get >/dev/null 2>&1 || die "apt-get not found (Ubuntu/Debian)"

# shellcheck disable=SC2207
APT=($(apt_cmd))

BASE=(
  build-essential
  ca-certificates
  cmake
  git
  ninja-build
  pkg-config
  python3
  dpkg-dev
  patchelf
  libgl1-mesa-dev
  libglu1-mesa-dev
  libgoogle-glog-dev
  libgflags-dev
  libprotobuf-dev
  protobuf-compiler
  libavcodec-dev
  libavutil-dev
  libswscale-dev
  librsvg2-bin
  qt6-base-dev
  qt6-base-dev-tools
  libqt6svg6-dev
  libqt6opengl6-dev
)

OPTIONAL=(
  libqt6openglwidgets6-dev
  qt6-tools-dev
  qt6-l10n-tools
  qt6-tools-dev-tools
  # Ogre viewport is required; system 1.12 is rare — CMake auto-vendors 1.12.10.
  libogre-1.12-dev
  libassimp-dev
  libeigen3-dev
)

log "apt-get update"
"${APT[@]}" update

log "Install base packages"
"${APT[@]}" install -y "${BASE[@]}"

for pkg in "${OPTIONAL[@]}"; do
  if "${APT[@]}" install -y "${pkg}"; then
    log "Installed ${pkg}"
  else
    log "Skipped ${pkg} (not in this release)"
  fi
done

if ! find /usr -name 'Qt6OpenGLWidgetsConfig.cmake' -print -quit 2>/dev/null | grep -q .; then
  log "Qt6::OpenGLWidgets not found."
  log "Ubuntu 22.04 Qt 6.2 does not ship this module. Use Ubuntu 24.04, or install Qt >= 6.4 and set CMAKE_PREFIX_PATH."
fi

log "Dependencies ready"
