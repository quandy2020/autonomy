#!/usr/bin/env bash
# Run in the Habitat-enabled Python environment (e.g. SpaceHero container).
set -euo pipefail
AUTOSIM_VIEWER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSIM_VIEWER_PYTHON="${AUTOSIM_PYTHON:-python3}"
if [[ -z "${AUTOSIM_PYTHON:-}" && -x /opt/venv/bin/python3 ]]; then
  AUTOSIM_VIEWER_PYTHON=/opt/venv/bin/python3
fi
export PYTHONPATH="${AUTOSIM_VIEWER_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${AUTOSIM_VIEWER_PYTHON}" -m autosim.robot.viewer "$@"
