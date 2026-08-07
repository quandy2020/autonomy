#!/usr/bin/env bash
# Isaac Sim Docker aliases python3 -> _isaac_sim/python.sh, which breaks stdlib
# (SRE module mismatch). Always invoke system Python for autoviz examples.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/usr/bin:/bin:/usr/local/bin:${PATH}"
unset PYTHONHOME PYTHONPATH
exec /usr/bin/python3 "${SCRIPT_DIR}/01_tutorial_show_path.py" "$@"
