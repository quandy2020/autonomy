#!/usr/bin/env bash
# Opt-in Habitat acceptance, separate from quick CTest and existing simulators.
set -euo pipefail
NAVIGATION="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$(cd "${NAVIGATION}/../.." && pwd)"
WORKSPACE="$(cd "${REPO}/../.." && pwd)"
BUILD="${AUTONOMY_BUILD_DIR:-${WORKSPACE}/build/autonomy}"
OUTPUT="${1:-${WORKSPACE}/data/navigation-autosim-$(date +%Y%m%d-%H%M%S)}"
PY="${AUTOSIM_PYTHON:-/opt/venv/bin/python3}"
export PYTHONPATH="${BUILD}/python:${BUILD}/automsgs/proto/gen/python:${BUILD}/autonomy/navigation/generated/python:${REPO}/autosim${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="${BUILD}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export AUTOLINK_PATH="${REPO}/autolink"
export AUTOLINK_LIB_PATH="${BUILD}/lib"
export NAVIGATION_CONF_DIR="${NAVIGATION}"
export GLOG_minloglevel=2 HABITAT_SIM_LOG=quiet MAGNUM_LOG=quiet
exec bash "${NAVIGATION}/tools/run_component_integration.sh" \
  "${PY}" "${NAVIGATION}/tools/autosim_acceptance.py" \
  --root "${REPO}" --build "${BUILD}" --output "${OUTPUT}" "${@:2}"
