#!/usr/bin/env bash
# Read-only prerequisites for the local build or cross-build controller.
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
case "${1:-native}" in
  native) tools=(git python3 cmake) ;;
  cross) tools=(git python3 docker ssh rsync) ;;
  *) echo "Usage: bash ansible/scripts/doctor.sh [native|cross]" >&2; exit 2 ;;
esac

for tool in "${tools[@]}"; do
  command -v "${tool}" >/dev/null || { echo "Missing tool: ${tool}" >&2; exit 1; }
done

submodules="$(git -C "${root}" submodule status --recursive)"
if [[ -n "${submodules}" ]]; then
  printf '%s\n' "${submodules}"
fi
if grep -Eq '^[-+U]' <<<"${submodules}"; then
  echo 'Initialize missing submodules and reconcile diverged pins before building.' >&2
  exit 1
fi

python3 --version
python3 "${root}/tools/python/clang_format_sources.py" --version
if [[ "${1:-native}" == 'cross' ]]; then
  docker version --format '{{.Server.Version}}'
  ssh -V
  rsync --version
else
  cmake --version
fi
