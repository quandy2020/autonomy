#!/usr/bin/env bash
# Run ansible-lint and playbook syntax checks locally / in CI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if ! command -v ansible-lint >/dev/null 2>&1; then
  echo "ansible-lint not found. Install: pip install -r requirements-ci.txt" >&2
  exit 1
fi

if ! command -v ansible-playbook >/dev/null 2>&1; then
  echo "ansible-playbook not found. Install: pip install -r requirements-ci.txt" >&2
  exit 1
fi

"${ROOT}/scripts/install_collection.sh"

echo "[ansible-ci] shell syntax and deployment regressions"
for script in deploy.sh scripts/*.sh; do
  bash -n "$script"
done
python3 -m unittest discover -s tests -v

echo "[ansible-ci] ansible-lint"
ansible-lint

echo "[ansible-ci] syntax-check deployment entrypoint"
ansible-playbook playbooks/site.yml --syntax-check

for environment in robots production staging sim demo; do
  expected=artifact
  dependencies=false
  install_prefix=/opt/autonomy
  system_install=true
  if [[ "${environment}" == demo ]]; then
    install_prefix=/home/quandy/workspace/demo
    system_install=false
  fi
  if [[ "${environment}" == staging || "${environment}" == sim ]]; then
    expected=build
    dependencies=true
  fi
  ansible-playbook -i "inventory/${environment}/hosts.yml" tests/inventory.yml \
    -c local -e "expected_deploy_mode=${expected}" -e "expected_dependencies=${dependencies}" \
    -e "expected_install_prefix=${install_prefix}" -e "expected_system_install=${system_install}"
done
ansible-playbook -i inventory/hosts.yml tests/inventory.yml \
  -e expected_deploy_mode=build -e expected_dependencies=true

ansible-playbook -i localhost, tests/build.yml

echo "[ansible-ci] OK"
