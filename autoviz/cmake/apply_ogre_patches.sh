#!/usr/bin/env bash
# Apply rviz-style Ogre 1.12 patches for Autoviz vendor build.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <ogre-source-dir>" >&2
  exit 1
fi

OGRE_SRC="$1"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/ogre_patches" && pwd)"

cd "${OGRE_SRC}"
for patch in "${PATCH_DIR}"/*.patch; do
  [[ -f "${patch}" ]] || continue
  # Skip if this patch was already applied (re-configure / re-populate).
  if patch -p1 -R --dry-run -i "${patch}" >/dev/null 2>&1; then
    echo "skip already applied: $(basename "${patch}")"
    continue
  fi
  echo "apply: $(basename "${patch}")"
  patch -p1 -i "${patch}"
done
