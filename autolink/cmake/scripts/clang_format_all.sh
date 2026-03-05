#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT_DIR"

CLANG_FORMAT=$(command -v clang-format || true)
if [[ -z "${CLANG_FORMAT}" ]]; then
  echo "clang-format not found in PATH" >&2
  exit 1
fi

mapfile -t FILES < <(find autolink -type f \( \
  -name "*.c" -o -name "*.cc" -o -name "*.cpp" -o \
  -name "*.h" -o -name "*.hpp" \))

if [[ ${#FILES[@]} -eq 0 ]]; then
  echo "No C/C++ files found to format."
  exit 0
fi

"${CLANG_FORMAT}" -i "${FILES[@]}"
echo "Formatted ${#FILES[@]} files with ${CLANG_FORMAT}."


