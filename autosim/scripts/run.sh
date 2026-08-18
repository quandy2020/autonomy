#!/usr/bin/env bash
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Launch autosim bridge and optional keyboard teleop.
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
    exec python -c "from autosim.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  both)
    python -m autosim --config "$CONFIG" &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    sleep 0.5
    python -c "from autosim.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  *)
    echo "usage: $0 [sim|telop|teleop|both]" >&2
    echo "  CONFIG=config/default.yaml $0 sim" >&2
    exit 1
    ;;
esac
