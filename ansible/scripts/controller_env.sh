#!/usr/bin/env bash
# Sourced by deploy.sh only when controller tooling is absent from PATH.
set -euo pipefail
controller_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
controller_venv="${controller_root}/.venv"
controller_bootstrap_python="$(command -v python3.12 || true)"
controller_stamp="${controller_venv}/.deployment-requirements.sha256"
controller_digest="$(cat "${controller_root}/requirements-ci.txt" "${controller_root}/requirements-lock.txt" "${controller_root}/requirements.yml" | sha256sum)"

if [[ ! -x "${controller_venv}/bin/python3" ]]; then
  if ! command -v python3.12 >/dev/null 2>&1; then
    echo "Install Python 3.12 with venv support on the controller, then retry." >&2
    exit 1
  fi
  python3.12 -m venv --without-pip "${controller_venv}"
fi
export PATH="${controller_venv}/bin:${PATH}"
if [[ ! -x "${controller_venv}/bin/ansible-playbook" || ! -f "${controller_stamp}" || "$(cat "${controller_stamp}")" != "${controller_digest}" ]]; then
  if python3 -m pip --version >/dev/null 2>&1; then
    python3 -m pip install -r "${controller_root}/requirements-ci.txt"
  elif [[ -n "${controller_bootstrap_python}" ]] && "${controller_bootstrap_python}" -m pip --version >/dev/null 2>&1; then
    "${controller_bootstrap_python}" -m pip --python "${controller_venv}/bin/python3" install -r "${controller_root}/requirements-ci.txt"
  else
    python3 -m ensurepip
    python3 -m pip install -r "${controller_root}/requirements-ci.txt"
  fi
  ansible-galaxy collection install -r "${controller_root}/requirements.yml"
  printf '%s\n' "${controller_digest}" > "${controller_stamp}"
fi
