#!/usr/bin/env bash
# Run Molecule smoke test (requires Docker).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker not found" >&2
  exit 1
fi

if ! command -v molecule >/dev/null 2>&1; then
  echo "molecule not found. Install: pip install -r requirements-dev.txt" >&2
  exit 1
fi

export MOLECULE_PROJECT_DIRECTORY="${ROOT}"
exec molecule test -s default
