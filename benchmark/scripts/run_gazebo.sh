#!/usr/bin/env bash
# Gazebo-role sidecar eval (autosim fake/habitat twin + optional autolink).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTOSIM="$(cd "${ROOT}/../autosim" && pwd)"
export CONFIG="${CONFIG:-${AUTOSIM}/config/fake.yaml}"

cd "${ROOT}"
benchmark eval \
  -c configs/default.yaml \
  --episodes "${EPISODES:-5}" \
  --set simulator.name=gazebo \
  --set "simulator.autosim_config=${CONFIG}" \
  --set simulator.launch_autosim="${LAUNCH:-false}" \
  --set simulator.autolink=true \
  --set task.mode=random \
  "$@"
