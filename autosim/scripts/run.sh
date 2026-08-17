#!/usr/bin/env bash
# Launch autosim bridge and optional keyboard telop.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CONFIG="${CONFIG:-config/default.yaml}"
MODE="${1:-sim}"

export PYTHONPATH="${ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

case "$MODE" in
  sim|bridge)
    exec python -m autosim --config "$CONFIG"
    ;;
  telop|teleop)
    exec python -c "from autosim.telop import Telop; Telop.main(['--config', '$CONFIG'])"
    ;;
  both)
    python -m autosim --config "$CONFIG" &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    sleep 0.5
    python -c "from autosim.telop import Telop; Telop.main(['--config', '$CONFIG'])"
    ;;
  *)
    echo "usage: $0 [sim|telop|both]" >&2
    echo "  CONFIG=config/default.yaml $0 sim" >&2
    exit 1
    ;;
esac
