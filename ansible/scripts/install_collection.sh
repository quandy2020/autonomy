#!/usr/bin/env bash
# Install this checkout locally; inventories and secrets are excluded from packaging.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ANSIBLE_CONFIG="${ROOT}/ansible.cfg"
exec ansible-galaxy collection install "${ROOT}" --force --no-deps \
  --collections-path "${ROOT}/.ansible/collections"
