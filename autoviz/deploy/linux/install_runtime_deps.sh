#!/usr/bin/env bash
# Install Ubuntu *runtime* packages needed by an Autoviz .deb / AppDir that
# links against system Qt (does not vendor Qt).
#
# Usage:
#   ./deploy/linux/install_runtime_deps.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

require_linux
command -v apt-get >/dev/null 2>&1 || die "apt-get not found"

# shellcheck disable=SC2207
APT=($(apt_cmd))

RUNTIME=(
  libqt6core6
  libqt6gui6
  libqt6widgets6
  libqt6opengl6
  libqt6openglwidgets6
  libqt6network6
  libqt6svg6
  libqt6xml6
  libqt6dbus6
  qt6-qpa-plugins
  libgl1
  libegl1
  libxkbcommon0
  libxcb-cursor0
  libyaml-cpp0.8
  libtinyxml2-10
  libavcodec60
  libavutil58
  libswscale7
  # glog / protobuf / assimp are often vendored into the .deb from /usr/local;
  # keep apt names as soft fallbacks for builds that link system packages.
  libgoogle-glog0v6
  libgflags2.2
  libprotobuf32
  libassimp5
)

# Soft aliases for older Ubuntu package names.
OPTIONAL=(
  libgoogle-glog0v5
  libyaml-cpp0.7
  libprotobuf23
  libavcodec58
  libtinyxml2-9
  libassimp5v5
)

log "apt-get update"
"${APT[@]}" update

log "Install runtime packages"
for pkg in "${RUNTIME[@]}"; do
  if "${APT[@]}" install -y "${pkg}"; then
    log "Installed ${pkg}"
  else
    log "Skipped ${pkg}"
  fi
done

for pkg in "${OPTIONAL[@]}"; do
  "${APT[@]}" install -y "${pkg}" 2>/dev/null && log "Installed ${pkg}" || true
done

log "Runtime dependencies ready"
log "Install the .deb with: sudo apt install ./dist/linux/autoviz_*.deb"
