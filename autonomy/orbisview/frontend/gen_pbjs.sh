#!/usr/bin/env bash
# Bundle OrbisView .proto → proto_bundle/*.json (Dreamview gen_pbjs.sh counterpart).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p proto_bundle

PROTO_DIR="../proto"
if [[ -x node_modules/.bin/pbjs ]]; then
  PBJS=node_modules/.bin/pbjs
elif [[ -f node_modules/protobufjs-cli/bin/pbjs ]]; then
  PBJS="node node_modules/protobufjs-cli/bin/pbjs"
else
  echo "pbjs not found; run: npm install -D protobufjs-cli"
  exit 0
fi

$PBJS -t json \
  "$PROTO_DIR/stream_envelope.proto" \
  "$PROTO_DIR/render.proto" \
  "$PROTO_DIR/plugin_config.proto" \
  "$PROTO_DIR/record.proto" \
  "$PROTO_DIR/point_cloud.proto" \
  > proto_bundle/orbisview_proto_bundle.json

echo "wrote proto_bundle/orbisview_proto_bundle.json"
