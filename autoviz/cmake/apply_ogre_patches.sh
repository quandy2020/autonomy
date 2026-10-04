#!/usr/bin/env bash
# Apply Autoviz Ogre 1.12 vendor fixes (policy + Apple sysroot + optional patches).
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <ogre-source-dir>" >&2
  exit 1
fi

OGRE_SRC="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_DIR="${SCRIPT_DIR}/ogre_patches"

[[ -d "${OGRE_SRC}" ]] || { echo "missing Ogre source: ${OGRE_SRC}" >&2; exit 1; }

echo "bump cmake_minimum_required -> 3.10 under ${OGRE_SRC}"
while IFS= read -r -d '' file; do
  perl -i -pe 's/cmake_minimum_required\s*\(\s*VERSION\s+[0-9.]+/cmake_minimum_required(VERSION 3.10/i' \
    "${file}"
done < <(find "${OGRE_SRC}" -name CMakeLists.txt -print0)

# Ninja/CLT: stock Ogre sets CMAKE_OSX_SYSROOT=macosx (Xcode token only).
python3 - "${OGRE_SRC}" <<'PY'
import sys
from pathlib import Path

p = Path(sys.argv[1]) / "CMakeLists.txt"
text = p.read_text()
if "xcrun --sdk macosx --show-sdk-path" in text:
    print("Apple CMAKE_OSX_SYSROOT already patched")
    sys.exit(0)

start = text.find("elseif (APPLE AND NOT APPLE_IOS)")
marker = "  # Make sure that the OpenGL render system is selected"
end = text.find(marker, start) if start >= 0 else -1
if start < 0 or end < 0 or "set(CMAKE_OSX_SYSROOT macosx)" not in text[start:end]:
    print("warning: Apple CMAKE_OSX_SYSROOT block not found", file=sys.stderr)
    sys.exit(0)

new = (
    "elseif (APPLE AND NOT APPLE_IOS)\n\n"
    "  set(XCODE_ATTRIBUTE_SDKROOT macosx)\n"
    "  execute_process(COMMAND xcrun --sdk macosx --show-sdk-path\n"
    "    OUTPUT_VARIABLE CMAKE_OSX_SYSROOT OUTPUT_STRIP_TRAILING_WHITESPACE)\n"
    "  if(NOT CMAKE_OSX_SYSROOT)\n"
    "    set(CMAKE_OSX_SYSROOT macosx)\n"
    "  endif()\n\n"
)
p.write_text(text[:start] + new + text[end:])
print("patched Apple CMAKE_OSX_SYSROOT for Ninja/CLT")
PY

if [[ ! -d "${PATCH_DIR}" ]]; then
  echo "no optional patch directory; continuing"
  exit 0
fi

shopt -s nullglob
patches=("${PATCH_DIR}"/*.patch)
shopt -u nullglob
if [[ ${#patches[@]} -eq 0 ]]; then
  exit 0
fi

cd "${OGRE_SRC}"
for patch in "${patches[@]}"; do
  if patch -p1 -R --dry-run -i "${patch}" >/dev/null 2>&1; then
    echo "skip already applied: $(basename "${patch}")"
    continue
  fi
  echo "apply: $(basename "${patch}")"
  patch -p1 -i "${patch}"
done
