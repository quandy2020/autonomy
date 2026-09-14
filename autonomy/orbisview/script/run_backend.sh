#!/usr/bin/env bash
# Start OrbisView backend (Civet :8766, WS /ws).
# If frontend/dist (or installed www/) exists, also serve static UI.
#
# Usage:
#   ./script/run_backend.sh
#   ./script/run_backend.sh --mock=false --autolink=true

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONF="${ROOT}/conf/orbisview.conf"

if ! command -v autonomy.orbisview >/dev/null 2>&1; then
  echo "autonomy.orbisview not on PATH (export build/.../bin)" >&2
  exit 1
fi

ARGS=(--flagfile="${CONF}")

DOC_ROOT=""
if [[ -f "${ROOT}/frontend/dist/index.html" ]]; then
  DOC_ROOT="${ROOT}/frontend/dist"
elif [[ -f "${ROOT}/www/index.html" ]]; then
  DOC_ROOT="${ROOT}/www"
fi
if [[ -n "${DOC_ROOT}" ]]; then
  ARGS+=(--document_root="${DOC_ROOT}")
fi

exec autonomy.orbisview "${ARGS[@]}" "$@"
