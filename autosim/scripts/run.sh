#!/usr/bin/env bash
# Launch autosim bridge and optional keyboard teleop.
set -euo pipefail

AUTOSIM="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTONOMY="$(cd "${AUTOSIM}/.." && pwd)"
ROOT="$(cd "${AUTONOMY}/../.." && pwd)"

export AUTONOMY_ROOT="${AUTONOMY_ROOT:-${ROOT}}"
if [[ -d "${ROOT}/build/autonomy" ]]; then
  export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${ROOT}/build/autonomy}"
elif [[ -d "${AUTONOMY}/build" ]]; then
  export AUTONOMY_BUILD_DIR="${AUTONOMY_BUILD_DIR:-${AUTONOMY}/build}"
fi

# Resolve CONFIG early so we can pick Isaac Lab python for backend=isaac.
CONFIG="${CONFIG:-config/default.yaml}"
if [[ "${CONFIG}" != /* ]]; then
  if [[ -f "${AUTOSIM}/${CONFIG}" ]]; then
    CONFIG="${AUTOSIM}/${CONFIG}"
  elif [[ -f "${AUTONOMY_ROOT:-${ROOT}}/${CONFIG}" ]]; then
    CONFIG="${AUTONOMY_ROOT:-${ROOT}}/${CONFIG}"
  else
    CONFIG="${AUTOSIM}/${CONFIG}"
  fi
fi

BACKEND="habitat"
if [[ -f "${CONFIG}" ]]; then
  BACKEND="$(
    awk '
      /^[[:space:]]*backend:[[:space:]]*/ {
        gsub(/["\047]/, "", $2);
        print $2;
        exit
      }
    ' "${CONFIG}"
  )"
  BACKEND="${BACKEND:-habitat}"
fi

resolve_isaac_python() {
  # Prefer explicit override, then Isaac Lab tree under /workspace/isaaclab.
  if [[ -n "${AUTOSIM_ISAAC_PYTHON:-}" && -x "${AUTOSIM_ISAAC_PYTHON}" ]]; then
    echo "${AUTOSIM_ISAAC_PYTHON}"
    return 0
  fi
  local lab="${ISAACLAB_PATH:-/workspace/isaaclab}"
  if [[ -x "${lab}/_isaac_sim/python.sh" ]]; then
    echo "${lab}/_isaac_sim/python.sh"
    return 0
  fi
  if [[ -x /isaac-sim/python.sh ]]; then
    echo /isaac-sim/python.sh
    return 0
  fi
  return 1
}

# Default: autonomy venv / system python (Habitat + fake).
# Isaac backend: must use Isaac Sim kit python so omni.* / isaacsim load.
PY=/usr/bin/python3
[[ -x /opt/venv/bin/python3 ]] && PY=/opt/venv/bin/python3
if [[ "${BACKEND}" == "isaac" ]]; then
  if ISAAC_PY="$(resolve_isaac_python)"; then
    PY="${ISAAC_PY}"
    export ISAACLAB_PATH="${ISAACLAB_PATH:-/workspace/isaaclab}"
    # Default headless in Docker / CI; set AUTOSIM_ISAAC_HEADLESS=0 for GUI.
    export AUTOSIM_ISAAC_HEADLESS="${AUTOSIM_ISAAC_HEADLESS:-1}"
    export HEADLESS="${HEADLESS:-${AUTOSIM_ISAAC_HEADLESS}}"
    # isaaclab packages (AppLauncher) live under source/ when not site-installed.
    if [[ -d "${ISAACLAB_PATH}/source/isaaclab" ]]; then
      export PYTHONPATH="${ISAACLAB_PATH}/source/isaaclab:${ISAACLAB_PATH}/source/isaaclab_assets:${ISAACLAB_PATH}/source/isaaclab_tasks:${ISAACLAB_PATH}/source/isaaclab_rl:${ISAACLAB_PATH}/source/isaaclab_mimic${PYTHONPATH:+:${PYTHONPATH}}"
    fi
    echo "[autosim] backend=isaac → python ${PY} (ISAACLAB_PATH=${ISAACLAB_PATH}, headless=${AUTOSIM_ISAAC_HEADLESS})" >&2
  else
    echo "error: habitat.backend=isaac but Isaac Sim python was not found." >&2
    echo "  Set ISAACLAB_PATH (default /workspace/isaaclab) or AUTOSIM_ISAAC_PYTHON." >&2
    exit 1
  fi
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
  if [[ "${BACKEND}" == "isaac" ]]; then
    echo "  Isaac Sim uses ${PY_TAG}; rebuild autolink against Isaac's interpreter, e.g.:" >&2
    echo "    cmake -DPython3_EXECUTABLE=${PY} ... && cmake --build ... --target _core" >&2
    echo "  Copy _core.${PY_TAG}-*.so next to the existing cpython-310 module under install/autonomy/python/autolink/." >&2
  else
    echo "Build/install autonomy so build/autonomy/python/autolink/_core.${PY_TAG}-*.so exists," >&2
    echo "or set AUTOLINK_PYTHON_DIR to that python directory." >&2
  fi
  exit 1
fi

PYTHON_ENTRIES=("${AUTOSIM}" "${AUTOLINK_PYTHON_DIR}")
for messages in \
  "${ROOT}/build/autonomy/automsgs/proto/gen/python" \
  "${ROOT}/build/automsgs/proto/gen/python" \
  "${ROOT}/install/autonomy/lib/python"
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

MODE="${1:-sim}"

case "$MODE" in
  sim|bridge)
    exec "${PY}" -m autosim --config "$CONFIG"
    ;;
  telop|teleop)
    exec "${PY}" -c "from autosim.io.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  both)
    "${PY}" -m autosim --config "$CONFIG" &
    SIM_PID=$!
    trap 'kill "$SIM_PID" 2>/dev/null || true' EXIT INT TERM
    sleep 0.5
    exec "${PY}" -c "from autosim.io.teleop import Teleop; Teleop.main(['--config', '$CONFIG'])"
    ;;
  *)
    echo "usage: $0 [sim|telop|teleop|both]" >&2
    echo "  CONFIG=config/default.yaml $0 sim" >&2
    echo "  CONFIG=config/isaac.yaml ISAACLAB_PATH=/workspace/isaaclab $0 sim" >&2
    exit 1
    ;;
esac
