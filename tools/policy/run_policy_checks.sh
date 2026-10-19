#!/usr/bin/env bash
# Policy contract + unit-test entrypoint for local / CI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

echo "== policy contract checklist =="
python3 tools/policy/validate_policy_contract.py --checklist

if [[ -f autonomy/control/conf/models/policy_toy.onnx ]]; then
  echo "== validate toy ONNX vs sidecar =="
  python3 tools/policy/validate_policy_contract.py \
    --config autonomy/control/conf/policy_config_toy_example.pb.txt \
    --onnx autonomy/control/conf/models/policy_toy.onnx
else
  echo "== skip ONNX validate (no autonomy/control/conf/models/policy_toy.onnx) =="
fi

echo "== bazel policy unit tests =="
bazel test \
  //autonomy/control:policy_action_schema_test \
  //autonomy/control:policy_safety_filter_test \
  //autonomy/control:policy_path_encoder_test \
  //autonomy/control:policy_frame_align_test
