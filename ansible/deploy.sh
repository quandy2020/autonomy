#!/usr/bin/env bash
# One-click entry for autonomy Ansible deployment.
#
# Usage:
#   ./deploy.sh                          # inventory/hosts.yml (local dev)
#   ./deploy.sh staging                  # inventory/staging/hosts.yml
#   ./deploy.sh production --tags config
#   AUTONOMY_DEPLOY_ENV=production ./deploy.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install ansible ansible-lint" >&2
  exit 1
fi

INVENTORY="inventory/hosts.yml"
EXTRA_ARGS=()

if [[ -n "${AUTONOMY_DEPLOY_ENV:-}" ]]; then
  if [[ -f "inventory/${AUTONOMY_DEPLOY_ENV}/hosts.yml" ]]; then
    INVENTORY="inventory/${AUTONOMY_DEPLOY_ENV}/hosts.yml"
  else
    echo "Unknown AUTONOMY_DEPLOY_ENV=${AUTONOMY_DEPLOY_ENV}" >&2
    exit 1
  fi
elif [[ $# -gt 0 && "${1}" != -* && -f "inventory/${1}/hosts.yml" ]]; then
  INVENTORY="inventory/${1}/hosts.yml"
  shift
fi

if [[ -f "${ROOT}/.vault_pass" ]]; then
  EXTRA_ARGS+=(--vault-password-file "${ROOT}/.vault_pass")
fi

exec ansible-playbook -i "${INVENTORY}" playbooks/site.yml "${EXTRA_ARGS[@]}" "$@"
