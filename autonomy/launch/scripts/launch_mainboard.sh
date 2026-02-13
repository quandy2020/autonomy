#!/usr/bin/env bash
#
# Optional: start mainboard with autonomy DAG and env set.
# Usage:
#   ./launch_mainboard.sh [launch_type]
#   launch_type: "direct" (default) or "python"
#
# Prerequisites:
#   - mainboard on PATH (e.g. after install: source install/setup.bash or set PATH)
#   - For "direct": set AUTOLINK_DAG_PATH and optionally AUTOLINK_LIB_PATH
#   - For "python": set AUTONOMY_LAUNCH_PATH and run from repo root or set PATH to autolink_launch.py
#

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DAG_DIR="${LAUNCH_DIR}/dag"
LAUNCH_TYPE="${1:-direct}"

export AUTOLINK_DAG_PATH="${AUTOLINK_DAG_PATH:-${LAUNCH_DIR}}"
export AUTOLINK_LIB_PATH="${AUTOLINK_LIB_PATH:-}"

case "${LAUNCH_TYPE}" in
  direct)
    if [[ ! -f "${DAG_DIR}/autonomy.dag" ]]; then
      echo "Warning: ${DAG_DIR}/autonomy.dag not found or not configured. Create/edit it for your components."
    fi
    exec mainboard -d dag/autonomy.dag -p autonomy_main
    ;;
  python)
    export AUTONOMY_LAUNCH_PATH="${AUTONOMY_LAUNCH_PATH:-${LAUNCH_DIR}}"
    AUTOLINK_LAUNCH_SCRIPT=""
    for d in "${LAUNCH_DIR}/../../autolink/autolink/tools/autolink_launch" \
             "${LAUNCH_DIR}/../../../autolink/autolink/tools/autolink_launch" \
             "/opt/autolink/tools/autolink_launch"; do
      if [[ -f "${d}/autolink_launch.py" ]]; then
        AUTOLINK_LAUNCH_SCRIPT="${d}/autolink_launch.py"
        break
      fi
    done
    if [[ -z "${AUTOLINK_LAUNCH_SCRIPT}" ]]; then
      echo "autolink_launch.py not found. Set AUTOLINK_LAUNCH_PATH or install autolink."
      exit 1
    fi
    exec python3 "${AUTOLINK_LAUNCH_SCRIPT}" start mainboard.launch
    ;;
  *)
    echo "Usage: $0 [direct|python]"
    exit 1
    ;;
esac
