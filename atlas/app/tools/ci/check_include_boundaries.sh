#!/usr/bin/env bash
# Only translation units under a backend_gtsam directory may include gtsam.
set -euo pipefail

root="$(cd "$(dirname "$0")/../../.." && pwd)"
status=0

while IFS= read -r source; do
  [[ "$source" == *"/backend_gtsam/"* ]] && continue
  if grep -nE '^[[:space:]]*#include[[:space:]]*[<"]gtsam/' "$source" >&2; then
    echo "${source#"$root"/}: gtsam include outside backend_gtsam" >&2
    status=1
  fi
done < <(find "$root/atlas" -type f \( -name '*.hpp' -o -name '*.h' -o -name '*.cpp' -o -name '*.cc' \) | sort)

exit "$status"
