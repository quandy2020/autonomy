#!/usr/bin/env bash
# Copy QGroundControl DJI F450 Qt Quick 3D meshes into autoviz (BSD-3-Clause assets).
set -euo pipefail

QGC_ROOT="${1:-${QGC_ROOT:-$HOME/workspace/github/ros/qgroundcontrol}}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$(cd "${SCRIPT_DIR}/../../qml/meshes" && pwd)"
SRC="${QGC_ROOT}/src/Viewer3D/Viewer3DQml/Drones/Djif450"

if [[ ! -d "${SRC}" ]]; then
  echo "QGC mesh source not found: ${SRC}" >&2
  echo "Clone qgroundcontrol and run: git lfs pull" >&2
  exit 1
fi

mkdir -p "${DEST}"
rsync -a --delete "${SRC}/" "${DEST}/"
echo "Copied F450 meshes to ${DEST}"
