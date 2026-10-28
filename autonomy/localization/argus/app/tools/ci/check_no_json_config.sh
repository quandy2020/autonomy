#!/usr/bin/env bash
# Configuration is YAML and protobuf text. Reject JSON config files and
# source that names a .json config or pulls in a JSON library.
set -euo pipefail

root="$(cd "$(dirname "$0")/../../.." && pwd)"
status=0

while IFS= read -r json; do
  echo "json config is not allowed: ${json#"$root"/}" >&2
  status=1
done < <(find "$root/argus" "$root/config" -name '*.json' -type f 2>/dev/null | sort)

while IFS= read -r source; do
  if grep -nE 'nlohmann/json|#include[[:space:]]*[<"]json/json\.h[>"]|[^[:space:]]+\.json' "$source" >&2; then
    echo "${source#"$root"/}: JSON config reference" >&2
    status=1
  fi
done < <(find "$root/argus" "$root/config" -type f \
  \( -name '*.hpp' -o -name '*.h' -o -name '*.cpp' -o -name '*.cc' -o -name '*.yaml' -o -name '*.yml' -o -name '*.py' \) |
  sort)

exit "$status"
