#!/usr/bin/env bash
# Validate DataPackage manifest and print RECORD_FILE for replay.
set -euo pipefail

PKG="${1:?usage: validate_datapackage.sh <package_dir>}"
MANIFEST="${PKG}/manifest.json"
REC="${PKG}/records/main.record"

if [[ ! -f "${MANIFEST}" ]]; then
  echo "missing manifest.json" >&2
  exit 1
fi
if [[ ! -f "${REC}" ]]; then
  echo "missing records/main.record" >&2
  exit 1
fi

python3 - <<'PY' "${MANIFEST}"
import json, sys
m = json.load(open(sys.argv[1]))
for k in ("schema_version", "case_id", "channels", "record_relpath"):
    if k not in m:
        raise SystemExit(f"manifest missing {k}")
print("ok", m.get("case_id"), "channels", len(m.get("channels", [])))
PY

echo "RECORD_FILE=${REC}"
echo "PACKAGE_DIR=${PKG}"
