#!/usr/bin/env bash
# Run ansible-lint and playbook syntax checks locally / in CI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if ! command -v ansible-lint >/dev/null 2>&1; then
  echo "ansible-lint not found. Install: pip install 'ansible-lint>=24,<26' (Python 3.10+)" >&2
  exit 1
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install 'ansible>=8,<10'" >&2
  exit 1
fi

if [[ -f requirements.yml ]]; then
  ansible-galaxy collection install -r requirements.yml 2>/dev/null || true
fi

echo "[ansible-ci] ansible-lint"
ansible-lint

PLAYBOOK="playbooks/site.yml"
MODES=(full build push check restart)
INVENTORIES=(
  inventory/hosts.yml
  inventory/robots/hosts.yml
  inventory/staging/hosts.yml
  inventory/production/hosts.yml
)

echo "[ansible-ci] syntax-check ${PLAYBOOK}"
ansible-playbook "${PLAYBOOK}" --syntax-check

for mode in "${MODES[@]}"; do
  echo "[ansible-ci] syntax-check ${PLAYBOOK} mode=${mode}"
  ansible-playbook "${PLAYBOOK}" -e "autonomy_play_mode=${mode}" --syntax-check
done

for inv in "${INVENTORIES[@]}"; do
  if [[ -f "${inv}" ]]; then
    echo "[ansible-ci] syntax-check ${PLAYBOOK} with ${inv}"
    ansible-playbook -i "${inv}" "${PLAYBOOK}" --syntax-check
  fi
done

echo "[ansible-ci] OK"
