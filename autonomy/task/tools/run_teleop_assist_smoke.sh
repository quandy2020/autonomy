#!/usr/bin/env bash
# Dev smoke: fakedata point cloud + teleop assist + cmd_vel (SpaceHero / repo root).
set -euo pipefail

ROOT="${AUTONOMY_ROOT:-/workspace/autonomy}"
cd "$ROOT"

export GLOG_logtostderr=1
export AUTONOMY_BT_PLUGIN_PATH="${AUTONOMY_BT_PLUGIN_PATH:-$ROOT/build/lib}"
export TELEOP_ASSIST_CONFIG="${TELEOP_ASSIST_CONFIG:-task/teleop_assist_smoke.lua}"

FAKEDATA_PID=""
cleanup() {
  if [[ -n "$FAKEDATA_PID" ]]; then
    kill "$FAKEDATA_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "==> fakedata (/fake/point_cloud2)"
./build/bin/autonomy_foxglove_fakedata --channel_prefix=/fake --rate_hz=10 &
FAKEDATA_PID=$!
sleep 2

echo "==> teleop smoke (assist via $TELEOP_ASSIST_CONFIG)"
./build/bin/task_teleop_smoke \
  --config_directory=config \
  --duration_sec=3 \
  --linear_x=0.25 \
  --rate_hz=20

echo "Done. Check log: no 'serialize to array failed' on cmd_vel; assist logs if enabled."
