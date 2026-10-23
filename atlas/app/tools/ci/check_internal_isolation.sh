#!/usr/bin/env bash
# A module may include only its own internal/ headers.
# Module is the first directory under atlas/atlas (common, map, frontend, ...).
set -euo pipefail

root="$(cd "$(dirname "$0")/../../.." && pwd)"
library="$root/atlas"
status=0

owner_of_include() {
  local include_path="$1"
  local source_module="$2"
  local prefix owner
  case "$include_path" in
    internal/*) printf '%s\n' "$source_module"; return ;;
    */internal/*)
      prefix="${include_path%%/internal/*}"
      owner="${prefix##*/}"
      printf '%s\n' "$owner"
      return
      ;;
    *) return 1 ;;
  esac
}

while IFS= read -r source; do
  relative="${source#"$library"/}"
  source_module="${relative%%/*}"
  while IFS= read -r include_path; do
    [[ -z "$include_path" ]] && continue
    if ! owner="$(owner_of_include "$include_path" "$source_module")"; then
      continue
    fi
    if [[ "$owner" != "$source_module" ]]; then
      echo "${relative}: includes ${include_path} (module ${source_module} cannot use ${owner}/internal)" >&2
      status=1
    fi
  done < <(grep -oE '^[[:space:]]*#include[[:space:]]*[<"][^>"]+[>"]' "$source" |
    sed -E 's/^[[:space:]]*#include[[:space:]]*[<"]([^>"]+)[>"].*/\1/')
done < <(find "$library" -type f \( -name '*.hpp' -o -name '*.h' -o -name '*.cpp' -o -name '*.cc' \) | sort)

exit "$status"
