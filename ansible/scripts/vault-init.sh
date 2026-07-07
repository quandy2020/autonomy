#!/usr/bin/env bash
# Initialize encrypted vault for Ansible secrets.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VAULT_EXAMPLE="${ROOT}/inventory/group_vars/vault.yml.example"
VAULT_FILE="${ROOT}/inventory/group_vars/vault.yml"
PASS_FILE="${ROOT}/.vault_pass"

if [[ -f "${VAULT_FILE}" ]]; then
  echo "vault.yml already exists: ${VAULT_FILE}" >&2
  exit 1
fi

if ! command -v ansible-vault >/dev/null 2>&1; then
  echo "ansible-vault not found. Install: pip install ansible" >&2
  exit 1
fi

cp "${VAULT_EXAMPLE}" "${VAULT_FILE}"
if [[ ! -f "${PASS_FILE}" ]]; then
  openssl rand -base64 32 > "${PASS_FILE}"
  chmod 600 "${PASS_FILE}"
  echo "Created vault password file: ${PASS_FILE}"
fi

ansible-vault encrypt --vault-password-file "${PASS_FILE}" "${VAULT_FILE}"
echo "Encrypted ${VAULT_FILE}"
echo "Edit: ansible-vault edit --vault-password-file ${PASS_FILE} ${VAULT_FILE}"
