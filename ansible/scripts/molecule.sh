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

"${ROOT}/scripts/install_collection.sh"
export ANSIBLE_COLLECTIONS_PATH="${ROOT}/.ansible/collections:${HOME}/.ansible/collections"
MOLECULE_ROLE="${MOLECULE_ROLE:-services}"
if [[ ! "${MOLECULE_ROLE}" =~ ^[a-z][a-z0-9_]*$ || ! -f "${ROOT}/molecule/roles/${MOLECULE_ROLE}/converge.yml" ]]; then
  echo "Unknown Molecule role case" >&2
  exit 1
fi
export MOLECULE_CONVERGE_PLAYBOOK="${ROOT}/molecule/roles/${MOLECULE_ROLE}/converge.yml"
export MOLECULE_PROJECT_DIRECTORY="${ROOT}"
exec molecule test -s default
