#!/usr/bin/env bash
# One-click entry for autonomy Ansible deployment.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install ansible" >&2
  exit 1
fi

exec ansible-playbook playbooks/site.yml "$@"
