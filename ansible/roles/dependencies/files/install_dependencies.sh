#!/usr/bin/env bash
# Invoked by Ansible dependencies role. Runs: python3 -m install_deps
set -euo pipefail

repo_root="${1:?repo root required}"
shift

mkdir -p /thirdparty

scripts_dir="${repo_root}/scripts"
marker="${scripts_dir}/install_deps/app.py"
if [[ ! -f "${marker}" ]]; then
  echo "Missing install_deps package at ${marker}" >&2
  exit 1
fi

cd "${scripts_dir}"
exec python3 -m install_deps "$@"
