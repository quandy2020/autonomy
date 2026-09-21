#!/usr/bin/env bash
# Helper for replay.launch player module.
set -euo pipefail
REC="${RECORD_FILE:-}"
if [[ -n "${PACKAGE_DIR:-}" && -z "${REC}" ]]; then
  REC="${PACKAGE_DIR}/records/main.record"
fi
if [[ -z "${REC}" || ! -f "${REC}" ]]; then
  echo "RECORD_FILE or PACKAGE_DIR/records/main.record required" >&2
  exit 1
fi
exec autolink recorder play -f "${REC}" -a -r "${REPLAY_RATE:-1.0}" -p 3 -k /cmd_vel
