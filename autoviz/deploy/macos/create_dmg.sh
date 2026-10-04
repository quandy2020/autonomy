#!/usr/bin/env bash
# Create a compressed DMG from Autoviz.app.
#
# Usage:
#   ./deploy/macos/create_dmg.sh
#   ./deploy/macos/create_dmg.sh --app dist/macos/Autoviz.app --output dist/macos

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

OUT_DIR="$(default_dist_dir)"
APP="${OUT_DIR}/Autoviz.app"
VOL_NAME="Autoviz"

usage() {
  cat <<'EOF'
Usage: create_dmg.sh [options]

  --app PATH       Path to Autoviz.app (default: dist/macos/Autoviz.app)
  --output DIR     Directory for the .dmg (default: same as app parent / dist/macos)
  --volume NAME    Volume name inside the DMG (default: Autoviz)
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --app) APP="$2"; shift 2 ;;
    --output) OUT_DIR="$2"; shift 2 ;;
    --volume) VOL_NAME="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ "$(uname -s)" == "Darwin" ]] || die "macOS only"
[[ -d "${APP}" ]] || die "missing app bundle: ${APP} (run create_app_bundle.sh first)"

VERSION="$(autoviz_version)"
ARCH="$(uname -m)"
mkdir -p "${OUT_DIR}"
DMG="${OUT_DIR}/Autoviz-${VERSION}-${ARCH}.dmg"
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/autoviz-dmg.XXXXXX")"
cleanup() { rm -rf "${STAGE}"; }
trap cleanup EXIT

log "Staging DMG contents"
cp -R "${APP}" "${STAGE}/"
ln -s /Applications "${STAGE}/Applications"

# Optional README drop-in for end users
cat > "${STAGE}/README.txt" <<EOF
Autoviz ${VERSION} (${ARCH})

1. Drag Autoviz.app to Applications.
2. First launch: if Gatekeeper blocks, open System Settings → Privacy & Security → Open Anyway.
3. Optional: set AUTOVIZ_RESOURCE_PATH / AUTOVIZ_PLUGIN_PATH for extra meshes / plugins.

Docs: https://github.com/openbot (see autoviz/docs)
EOF

rm -f "${DMG}"
log "Creating ${DMG}"
hdiutil create \
  -volname "${VOL_NAME}" \
  -srcfolder "${STAGE}" \
  -ov \
  -format UDZO \
  "${DMG}"

log "Done: ${DMG}"
