#!/usr/bin/env bash
# Initialize git submodules required to build autonomy (autolink).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

if [[ -f autolink/CMakeLists.txt ]]; then
  echo "autolink submodule already initialized."
  exit 0
fi

if [[ ! -f .gitmodules ]]; then
  echo "No .gitmodules found; nothing to do." >&2
  exit 1
fi

export GIT_HTTP_VERSION=HTTP/1.1
git submodule update --init --recursive autolink

if [[ ! -f autolink/CMakeLists.txt ]]; then
  echo "autolink submodule still missing after git submodule update." >&2
  exit 1
fi

echo "autolink submodule ready."
