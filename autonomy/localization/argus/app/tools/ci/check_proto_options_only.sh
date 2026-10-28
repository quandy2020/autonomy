#!/usr/bin/env bash
# Every protobuf message must end with Options.
# Vector3d, Quaterniond, and Rigid3d are the Eigen wire types inside
# argus/common/common_options.proto and are the only exceptions.
set -euo pipefail

root="$(cd "$(dirname "$0")/../../.." && pwd)"
status=0

while IFS= read -r proto; do
  line_number=0
  while IFS= read -r line || [[ -n "$line" ]]; do
    line_number=$((line_number + 1))
    [[ "$line" =~ ^[[:space:]]*// ]] && continue
    if [[ "$line" =~ ^[[:space:]]*message[[:space:]]+([A-Za-z_][A-Za-z0-9_]*) ]]; then
      name="${BASH_REMATCH[1]}"
      if [[ "$name" == *Options ]]; then
        continue
      fi
      relative="${proto#"$root"/}"
      if [[ "$relative" == "argus/common/common_options.proto" && "$name" =~ ^(Vector3d|Quaterniond|Rigid3d)$ ]]; then
        continue
      fi
      echo "${relative}:${line_number}: message ${name} must end with Options" >&2
      status=1
    fi
  done <"$proto"
done < <(find "$root/argus" "$root/config" -name '*.proto' -type f 2>/dev/null | sort)

exit "$status"
