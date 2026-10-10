#!/usr/bin/env bash
# Stage proto/{msgs,srvs,rpcs,actions,task} as automsgs/... and run protoc.
# Usage: generate_messages.sh <automsgs_generate_factory.py> <output_dir>
set -euo pipefail

factory_py="${1:?factory script}"
out_dir="${2:?output dir}"

# Prefer Bazel-provided protoc (PROTOC=) so generated *.pb.h match BCR headers.
if [[ -n "${PROTOC:-}" && -x "${PROTOC}" ]]; then
  protoc="${PROTOC}"
elif [[ -x /usr/local/bin/protoc ]]; then
  protoc=/usr/local/bin/protoc
elif command -v protoc >/dev/null 2>&1; then
  protoc="$(command -v protoc)"
else
  echo "protoc not found (set PROTOC= or install /usr/local/bin/protoc)" >&2
  exit 1
fi

# proto3 optional is stable from 3.15; older protoc needs the experimental flag.
extra=()
ver="$("$protoc" --version | sed -n 's/.* \([0-9][0-9]*\)\.\([0-9][0-9]*\).*/\1 \2/p')"
major="${ver%% *}"
minor="${ver##* }"
if [[ -n "${major}" && -n "${minor}" ]]; then
  if (( major < 3 || (major == 3 && minor < 15) )); then
    extra+=(--experimental_allow_proto3_optional)
  fi
fi

stage="$(mktemp -d)"
trap 'rm -rf "${stage}"' EXIT
mkdir -p "${stage}/automsgs"
cp -aL proto/msgs proto/srvs proto/rpcs proto/actions proto/task "${stage}/automsgs/"

mapfile -t protos < <(find "${stage}/automsgs" -name '*.proto' | sort)
if [[ "${#protos[@]}" -eq 0 ]]; then
  echo "no .proto files staged under ${stage}/automsgs" >&2
  exit 1
fi

mkdir -p "${out_dir}/python"
"${protoc}" \
  "${extra[@]}" \
  --proto_path="${stage}" \
  --cpp_out="${out_dir}" \
  --python_out="${out_dir}/python" \
  "${protos[@]}"

while IFS= read -r hdr; do
  dir="$(dirname "${hdr}")"
  base="$(basename "${hdr}")"
  mkdir -p "${dir}/details"
  cp -a "${hdr}" "${dir}/details/${base}"
done < <(find "${out_dir}/automsgs" -name '*.pb.h' ! -path '*/details/*')

python3 "${factory_py}" \
  --output-cpp-path "${out_dir}" \
  --output-header "${out_dir}/automsgs/msgs/MessageTypes.hh"
