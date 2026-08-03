#!/usr/bin/env bash
# Publish a BICMap example for autoviz preview.
set -euo pipefail

AUTOVIZ_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTONOMY_ROOT="$(cd "${AUTOVIZ_PKG}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${AUTONOMY_ROOT}/../.." && pwd)"
BUILD="${BUILD_DIR:-${WORKSPACE_ROOT}/build}"
BIN="${BUILD}/bin/autoviz_bicmap_publisher"
EXAMPLE="${1:-Slam}"
MODE="${2:-publish}"

if [[ ! -x "${BIN}" ]]; then
  echo "Missing ${BIN}; build with:" >&2
  echo "  cmake --build ${BUILD} --target autoviz_bicmap_publisher" >&2
  exit 1
fi

case "${MODE}" in
  list)
    "${BIN}" --list
    ;;
  cycle)
    shift 2 || true
    "${BIN}" --cycle "$@"
    ;;
  autoviz)
    echo "Starting publisher + autoviz (example=${EXAMPLE})..." >&2
    "${BIN}" --example "${EXAMPLE}" &
    PUB_PID=$!
    sleep 1
    trap 'kill ${PUB_PID} 2>/dev/null || true' EXIT
    "${AUTOVIZ_PKG}/scripts/run_autoviz.sh" -c "${AUTOVIZ_PKG}/config/strata.autoviz"
    ;;
  publish|*)
    "${BIN}" --example "${EXAMPLE}" "${@:3}"
    ;;
esac
