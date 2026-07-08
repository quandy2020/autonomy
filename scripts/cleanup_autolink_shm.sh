#!/usr/bin/env bash
# Remove stale autolink SysV SHM segment for condition_notifier.
# Key matches autolink::transport::ConditionNotifier (std::hash of
# "/autolink/transport/shm/notifier", truncated to key_t).
set -euo pipefail

AUTOLINK_NOTIFIER_SHM_KEY="${AUTOLINK_NOTIFIER_SHM_KEY:-0x960f6a27}"

if ! command -v ipcrm >/dev/null 2>&1; then
  echo "ipcrm not found; cannot clean autolink SHM" >&2
  exit 0
fi

if ipcrm -M "${AUTOLINK_NOTIFIER_SHM_KEY}" 2>/dev/null; then
  echo "Removed autolink notifier SHM key ${AUTOLINK_NOTIFIER_SHM_KEY}"
else
  echo "No autolink notifier SHM segment at key ${AUTOLINK_NOTIFIER_SHM_KEY} (ok)"
fi
