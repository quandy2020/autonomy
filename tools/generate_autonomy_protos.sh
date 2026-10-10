#!/usr/bin/env bash
# Stage automsgs protos + compile autonomy/**/*.proto with BCR protoc.
# Usage: generate_autonomy_protos.sh <protoc> <automsgs_pkg_root> <autonomy_workspace_root> <output_dir>
set -euo pipefail

protoc="${1:?protoc}"
automsgs_root="${2:?automsgs package root}"
autonomy_root="${3:?autonomy workspace root}"
out_dir="${4:?output dir}"

stage="$(mktemp -d)"
trap 'rm -rf "${stage}"' EXIT

mkdir -p "${stage}/automsgs"
for d in msgs srvs rpcs actions task; do
  if [[ -d "${automsgs_root}/proto/${d}" ]]; then
    cp -aL "${automsgs_root}/proto/${d}" "${stage}/automsgs/"
  fi
done

mapfile -t protos < <(
  find "${autonomy_root}/autonomy" -name '*.proto' \
    ! -path '*/tools/*' \
    ! -path '*/.proto_gen/*' \
    | sort
)
if [[ "${#protos[@]}" -eq 0 ]]; then
  echo "no autonomy .proto files under ${autonomy_root}/autonomy" >&2
  exit 1
fi

# Batch to stay under ARG_MAX; imports resolve via --proto_path.
printf '%s\0' "${protos[@]}" | xargs -0 -n 40 \
  "${protoc}" \
  --proto_path="${stage}" \
  --proto_path="${autonomy_root}" \
  --cpp_out="${out_dir}"
