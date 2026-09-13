# OrbisView frontend setup (Dreamview setup.sh counterpart).
# Generate proto JS bundles when protobufjs is available.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p proto_bundle assets/images assets/models
if [[ -x node_modules/.bin/pbjs ]] || [[ -f node_modules/protobufjs/bin/pbjs ]]; then
  ./gen_pbjs.sh
else
  echo "[orbisview] protobufjs not installed — skip gen_pbjs (npm install first)"
fi
