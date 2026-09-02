#!/usr/bin/env bash
# Copyright 2026 The Openbot Authors
#
# Start Cartographer SLAM (autosim slam mode).
#
#   ./autonomy/localization/scripts/start_cartographer.sh
#   ./autonomy/localization/scripts/start_cartographer.sh stop
#
# From repo root (/workspace/autonomy):
#   src/autonomy/autonomy/localization/scripts/start_cartographer.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTONOMY_PKG="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_ROOT="$(cd "${AUTONOMY_PKG}/../.." && pwd)"

export PATH="${REPO_ROOT}/build/autonomy/bin:${PATH}"
export AUTOLINK_LAUNCH_PATH="${AUTONOMY_PKG}/autonomy/localization/launch"

mkdir -p "${AUTONOMY_PKG}/data/maps"

ACTION="${1:-start}"
LAUNCH="localization_cartographer.launch"

case "${ACTION}" in
  start|stop)
    exec autolink launch "${ACTION}" "${LAUNCH}"
    ;;
  *)
    echo "usage: $0 [start|stop]" >&2
    exit 1
    ;;
esac
