#!/usr/bin/env bash
# Build BicMap from local source (third_party/bic-map-src) into dashboard vendor.
# No website / CDN download at runtime.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/third_party/bic-map-src"
OUT="$ROOT/src/benchmark/dashboard/static/vendor/bic-map"

if [[ ! -f "$SRC/package.json" ]]; then
  echo "missing $SRC — restore BicMap library source under third_party/bic-map-src" >&2
  exit 1
fi

cd "$SRC"
npm install --legacy-peer-deps --no-fund --no-audit
npm run build:cdn

mkdir -p "$OUT"
cp -f "$SRC/dist/cdn/bicMap.ext.min.js" "$OUT/bicMap.ext.min.js"
cp -f "$SRC/LICENSE" "$OUT/LICENSE"
cp -f "$SRC/THIRD-PARTY-LICENSES.md" "$OUT/THIRD-PARTY-LICENSES.md"
python3 - <<PY
import json
from pathlib import Path
pkg = json.loads(Path("$SRC/package.json").read_text())
meta = {
    "name": pkg["name"],
    "version": pkg["version"],
    "source": "third_party/bic-map-src",
    "built": "local",
}
Path("$OUT/package.meta.json").write_text(json.dumps(meta, indent=2) + "\n")
print("wrote", meta)
PY

echo "OK → $OUT/bicMap.ext.min.js"
