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

echo "[ansible-ci] syntax-check playbooks"
for playbook in playbooks/*.yml; do
  ansible-playbook "${playbook}" --syntax-check
done

INVENTORIES=(inventory/hosts.yml inventory/staging/hosts.yml inventory/production/hosts.yml)
for inv in "${INVENTORIES[@]}"; do
  if [[ -f "${inv}" ]]; then
    echo "[ansible-ci] syntax-check site.yml with ${inv}"
    ansible-playbook -i "${inv}" playbooks/site.yml --syntax-check
  fi
done

echo "[ansible-ci] OK"
