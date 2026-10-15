#!/usr/bin/env bash
# Launch Arena Evaluation against Isaac (autosim sidecar + kinematic twin metrics).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSIM="$(cd "${ROOT}/../autosim" && pwd)"
export AUTONOMY_ROOT="${AUTONOMY_ROOT:-$(cd "${ROOT}/../../.." && pwd)}"
export ISAACLAB_PATH="${ISAACLAB_PATH:-/workspace/isaaclab}"
export CONFIG="${CONFIG:-${AUTOSIM}/config/isaac.yaml}"

cd "${ROOT}"
benchmark eval \
  -c configs/default.yaml \
  --episodes "${EPISODES:-5}" \
  --set simulator.name=isaac \
  --set "simulator.autosim_config=${CONFIG}" \
  --set "simulator.isaaclab_path=${ISAACLAB_PATH}" \
  --set simulator.launch_autosim=true \
  --set simulator.autolink=true \
  --set simulator.sync_odom=true \
  --set task.mode=scenario \
  "$@"
