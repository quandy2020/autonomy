#!/usr/bin/env bash
# Run in the Habitat-enabled Python environment (e.g. SpaceHero container).
set -euo pipefail
AUTOSIM_VIEWER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSIM_VIEWER_PYTHON="${AUTOSIM_PYTHON:-python3}"
if [[ -z "${AUTOSIM_PYTHON:-}" && -x /opt/venv/bin/python3 ]]; then
  AUTOSIM_VIEWER_PYTHON=/opt/venv/bin/python3
fi
AUTOSIM_VIEWER_WORKSPACE="$(cd "${AUTOSIM_VIEWER_ROOT}/../../.." && pwd)"
AUTOSIM_VIEWER_BINDINGS="${AUTOLINK_PYTHON_DIR:-${AUTOSIM_VIEWER_WORKSPACE}/install/autonomy/python}"
AUTOSIM_VIEWER_MESSAGES="${AUTOSIM_VIEWER_WORKSPACE}/install/autonomy/lib/python"
export LD_LIBRARY_PATH="${AUTOSIM_VIEWER_WORKSPACE}/install/autonomy/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export AUTOLINK_PATH="${AUTOLINK_PATH:-${AUTOSIM_VIEWER_ROOT}/../autolink}"
export PYTHONPATH="${AUTOSIM_VIEWER_ROOT}:${AUTOSIM_VIEWER_BINDINGS}:${AUTOSIM_VIEWER_MESSAGES}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${AUTOSIM_VIEWER_PYTHON}" -m autosim.robot.viewer "$@"
