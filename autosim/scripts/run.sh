#!/usr/bin/env bash
# Launch autosim bridge and optional keyboard teleop.
set -euo pipefail

AUTOSIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTONOMY="$(cd "${AUTOSIM}/.." && pwd)"
ROOT="$(cd "${AUTONOMY}/../.." && pwd)"

# Prefer the autonomy venv: interactive shells may alias python3 to Isaac Sim.
PY=/usr/bin/python3
[[ -x /opt/venv/bin/python3 ]] && PY=/opt/venv/bin/python3

export AUTONOMY_ROOT="${AUTONOMY_ROOT:-${ROOT}}"
if [[ -d "${ROOT}/build/autonomy" ]]; then
  export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${ROOT}/build/autonomy}"
elif [[ -d "${AUTONOMY}/build" ]]; then
  export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY}/build}"
fi

PY_TAG="$("${PY}" - <<'PY'
import sys
print(f"cpython-{sys.version_info.major}{sys.version_info.minor}")
PY
)"

if [[ -z "${AUTOLINK_PYTHON_DIR:-}" ]]; then
  for candidate in \
    "${ROOT}/install/autonomy/python" \
    "${ROOT}/install/autoviz/python" \
    "${ROOT}/build/autoviz/python" \
    "${ROOT}/build/autolink-python/python" \
    "${ROOT}/build/autonomy/python" \
    "${ROOT}/build/python"
  do
    if compgen -G "${candidate}/autolink/_core.${PY_TAG}"*.so > /dev/null; then
      export AUTOLINK_PYTHON_DIR="${candidate}"
      break
    fi
  done
fi

if [[ -z "${AUTOLINK_PYTHON_DIR:-}" ]]; then
  echo "error: autolink Python bindings for ${PY_TAG} were not found." >&2
  echo "Build/install autonomy so build/autonomy/python/autolink/_core.${PY_TAG}-*.so exists," >&2
  echo "or set AUTOLINK_PYTHON_DIR to that python directory." >&2
  exit 1
fi

PYTHON_ENTRIES=("${AUTOSIM}" "${AUTOLINK_PYTHON_DIR}")
for messages in \
  "${ROOT}/build/autonomy/automsgs/proto/gen/python" \
  "${ROOT}/build/automsgs/proto/gen/python"
do
  [[ -d "${messages}" ]] && PYTHON_ENTRIES+=("${messages}")
done
export PYTHONPATH="$(IFS=:; echo "${PYTHON_ENTRIES[*]}")${PYTHONPATH:+:${PYTHONPATH}}"

LIBRARY_ENTRIES=()
for library in \
  "${ROOT}/install/autonomy/lib" \
  "${AUTONOMY_BUILD_DIR:-}/lib" \
  "${ROOT}/build/autonomy/lib"
do
  [[ -d "${library}" ]] && LIBRARY_ENTRIES+=("${library}")
done
if ((${#LIBRARY_ENTRIES[@]})); then
  export LD_LIBRARY_PATH="$(IFS=:; echo "${LIBRARY_ENTRIES[*]}")${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

export AUTOLINK_PATH="${AUTOLINK_PATH:-${AUTONOMY}/autolink}"
export GLOG_minloglevel="${GLOG_minloglevel:-1}"
export GLOG_stderrthreshold="${GLOG_stderrthreshold:-2}"

# Run from the workspace root so glog/autolink INFO files land here, not under
# src/autonomy/autosim.
WORKSPACE="${AUTONOMY_ROOT:-${ROOT}}"
export GLOG_log_dir="${GLOG_log_dir:-${WORKSPACE}}"
cd "${WORKSPACE}"

CONFIG="${CONFIG:-config/default.yaml}"
if [[ "${CONFIG}" != /* ]]; then
  CONFIG="${AUTOSIM}/${CONFIG}"
fi
MODE="${1:-sim}"

case "$MODE" in
  sim|bridge)
    exec "${PY}" -m autosim --config "$CONFIG"
    ;;
  telop|teleop)
    exec "${PY}" -c "from autosim.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  both)
    "${PY}" -m autosim --config "$CONFIG" &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    sleep 0.5
    exec "${PY}" -c "from autosim.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  *)
    echo "usage: $0 [sim|telop|teleop|both]" >&2
    echo "  CONFIG=config/default.yaml $0 sim" >&2
    exit 1
    ;;
esac
