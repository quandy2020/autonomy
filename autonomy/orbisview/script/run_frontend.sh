#!/usr/bin/env bash
# Start OrbisView Vite frontend (dev server :5173).
# Used by launch/orbisview.launch; cwd-independent.
#
# Usage:
#   ./script/run_frontend.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FE="${ROOT}/frontend"

if ! command -v npm >/dev/null 2>&1; then
  echo "npm not found; install with: bash ${ROOT}/script/install_npm.sh" >&2
  exit 1
fi

cd "${FE}"
if [[ ! -d node_modules ]]; then
  echo "== npm ci (first run) =="
  npm ci
fi

exec npm start
