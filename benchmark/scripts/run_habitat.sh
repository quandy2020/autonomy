#!/usr/bin/env bash
# Launch Arena Evaluation against Habitat (autosim sidecar + kinematic twin).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSIM="$(cd "${ROOT}/../autosim" && pwd)"
export AUTONOMY_ROOT="${AUTONOMY_ROOT:-$(cd "${ROOT}/../../.." && pwd)}"
export CONFIG="${CONFIG:-${AUTOSIM}/config/default.yaml}"

cd "${ROOT}"
benchmark eval \
  -c configs/default.yaml \
  --episodes "${EPISODES:-5}" \
  --set simulator.name=habitat \
  --set "simulator.autosim_config=${CONFIG}" \
  --set simulator.launch_autosim="${LAUNCH:-false}" \
  --set simulator.autolink=true \
  --set simulator.sync_odom=true \
  --set simulator.subscribe_scan=true \
  --set simulator.inject_scan=true \
  --set task.mode=scenario \
  "$@"
