#!/usr/bin/env bash
# Create Autoviz.app from a configured build tree (Release recommended).
#
# Usage (from autoviz/ or any cwd):
#   ./deploy/macos/create_app_bundle.sh
#   ./deploy/macos/create_app_bundle.sh --build-dir build --output dist/macos
#   ./deploy/macos/create_app_bundle.sh --skip-macdeployqt
#   ./deploy/macos/create_app_bundle.sh --sign "Developer ID Application: …"
#
# Layout (matches applicationDirPath()/../share/autonomy):
#   Autoviz.app/Contents/
#     MacOS/autoviz
#     Frameworks/          # Qt + project + Homebrew dylibs
#     PlugIns/             # Qt plugins (via macdeployqt)
#     Resources/Autoviz.icns
#     share/autonomy/autoviz/{default.autoviz,ogre_media/}
#     Info.plist

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./_common.sh
source "${SCRIPT_DIR}/_common.sh"

BUILD_DIR="$(default_build_dir)"
OUT_DIR="$(default_dist_dir)"
SKIP_MACDEPLOYQT=0
SIGN_IDENTITY="-"   # ad-hoc by default; empty = skip sign
APP_NAME="Autoviz"

usage() {
  cat <<'EOF'
Usage: create_app_bundle.sh [options]

  --build-dir DIR     CMake build directory (default: $AUTOVIZ_BUILD_DIR or ./build)
  --output DIR        Output directory for Autoviz.app (default: dist/macos)
  --name NAME         Bundle name without .app (default: Autoviz)
  --skip-macdeployqt  Only stage binary + project libs (dev use)
  --sign IDENTITY     codesign identity ("-" = ad-hoc, "" = skip)
  -h, --help          Show help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --output) OUT_DIR="$2"; shift 2 ;;
    --name) APP_NAME="$2"; shift 2 ;;
    --skip-macdeployqt) SKIP_MACDEPLOYQT=1; shift ;;
    --sign) SIGN_IDENTITY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[[ "$(uname -s)" == "Darwin" ]] || die "macOS only"

BIN="${BUILD_DIR}/bin/autoviz"
LIB_DIR="${BUILD_DIR}/lib"
[[ -x "${BIN}" ]] || die "missing executable: ${BIN} (build target autoviz_app first)"
[[ -d "${LIB_DIR}" ]] || die "missing lib dir: ${LIB_DIR}"

VERSION="$(autoviz_version)"
APP="${OUT_DIR}/${APP_NAME}.app"
CONTENTS="${APP}/Contents"
MACOS="${CONTENTS}/MacOS"
FRAMEWORKS="${CONTENTS}/Frameworks"
RESOURCES="${CONTENTS}/Resources"
SHARE="${CONTENTS}/share/autonomy/autoviz"
PLUGINS_DIR="${CONTENTS}/lib/autoviz_plugins"

log "Staging ${APP} (v${VERSION})"
rm -rf "${APP}"
mkdir -p "${MACOS}" "${FRAMEWORKS}" "${RESOURCES}" "${SHARE}" "${PLUGINS_DIR}"

cp -f "${BIN}" "${MACOS}/autoviz"
chmod +x "${MACOS}/autoviz"

# Project shared libraries (macdeployqt follows @rpath after we add Frameworks rpath).
shopt -s nullglob
for lib in "${LIB_DIR}"/libautoviz*.dylib \
           "${LIB_DIR}"/libautolink*.dylib \
           "${LIB_DIR}"/libautomsgs*.dylib; do
  cp -a "${lib}" "${FRAMEWORKS}/"
done
shopt -u nullglob

# Drop absolute build RPATHs; keep relative Frameworks lookup.
if command -v install_name_tool >/dev/null 2>&1; then
  # Remove known absolute rpaths from the staged binary (best-effort).
  while IFS= read -r line; do
    if [[ "${line}" == /* ]]; then
      install_name_tool -delete_rpath "${line}" "${MACOS}/autoviz" 2>/dev/null || true
    fi
  done < <(otool -l "${MACOS}/autoviz" | awk '/LC_RPATH/{getline; getline; sub(/^ *path /,""); sub(/ \(offset.*/,""); print}')
  install_name_tool -add_rpath "@executable_path/../Frameworks" "${MACOS}/autoviz" 2>/dev/null || true
  install_name_tool -add_rpath "@loader_path/../Frameworks" "${MACOS}/autoviz" 2>/dev/null || true
fi

# Icon + Info.plist
ICNS="${AUTOVIZ_ROOT}/resources/icons/aviz.icns"
[[ -f "${ICNS}" ]] || die "missing icon: ${ICNS}"
cp -f "${ICNS}" "${RESOURCES}/Autoviz.icns"

sed -e "s/@AUTOVIZ_VERSION@/${VERSION}/g" \
  "${SCRIPT_DIR}/Info.plist.in" > "${CONTENTS}/Info.plist"

# Runtime assets expected next to MacOS via ../share/autonomy
cp -f "${AUTOVIZ_ROOT}/config/default.autoviz" "${SHARE}/"
if [[ -d "${AUTOVIZ_ROOT}/resources/ogre_media" ]]; then
  rsync -a --delete \
    "${AUTOVIZ_ROOT}/resources/ogre_media/" "${SHARE}/ogre_media/"
fi

if [[ "${SKIP_MACDEPLOYQT}" -eq 0 ]]; then
  QT_PREFIX="$(find_qt_prefix)" || die "macdeployqt not found (brew install qt@6)"
  MACDEPLOYQT="${QT_PREFIX}/bin/macdeployqt"
  log "Running macdeployqt (${QT_PREFIX})"
  BREW_LIB=""
  for _bp in /opt/homebrew /usr/local; do
    if [[ -d "${_bp}/lib" ]]; then
      BREW_LIB="${_bp}/lib"
      break
    fi
  done

  # -libpath helps resolve Homebrew + project dylibs for non-Qt deps.
  DEPLOY_ARGS=(
    "${APP}"
    -always-overwrite
    -executable="${MACOS}/autoviz"
    -libpath="${FRAMEWORKS}"
    -libpath="${LIB_DIR}"
  )
  if [[ -n "${BREW_LIB}" ]]; then
    DEPLOY_ARGS+=(-libpath="${BREW_LIB}")
  fi
  "${MACDEPLOYQT}" "${DEPLOY_ARGS[@]}"
else
  log "Skipped macdeployqt (bundle may require Homebrew at runtime)"
fi

# Best-effort: copy any remaining absolute Homebrew dylibs referenced by the tree.
_bundle_abs_deps() {
  local root="$1"
  local changed=1
  local round=0
  while [[ "${changed}" -eq 1 && "${round}" -lt 8 ]]; do
    changed=0
    round=$((round + 1))
    local file dep dest
    while IFS= read -r -d '' file; do
      while IFS= read -r dep; do
        # Keep system / rpath / relative load commands; bundle absolute third-party paths.
        if [[ "${dep}" != /* ]]; then
          continue
        fi
        if [[ "${dep}" == /System/* || "${dep}" == /usr/lib/* ]]; then
          continue
        fi
        if [[ ! -f "${dep}" ]]; then
          continue
        fi
        dest="${FRAMEWORKS}/$(basename "${dep}")"
        if [[ ! -e "${dest}" ]]; then
          log "Bundling $(basename "${dep}")"
          cp -a "${dep}" "${dest}"
          changed=1
        fi
        local base
        base="$(basename "${dep}")"
        install_name_tool -change "${dep}" "@rpath/${base}" "${file}" 2>/dev/null || true
      done < <(otool -L "${file}" 2>/dev/null | awk 'NR>1 {print $1}')
    done < <(find "${MACOS}" "${FRAMEWORKS}" -type f \( -perm -111 -o -name '*.dylib' -o -name '*.so' \) -print0 2>/dev/null)
  done
}

if command -v otool >/dev/null 2>&1; then
  _bundle_abs_deps "${APP}"
  # Ensure Frameworks dylibs identify as @rpath/name for consistency.
  shopt -s nullglob
  for lib in "${FRAMEWORKS}"/*.dylib; do
    base="$(basename "${lib}")"
    install_name_tool -id "@rpath/${base}" "${lib}" 2>/dev/null || true
  done
  shopt -u nullglob
fi

if [[ -n "${SIGN_IDENTITY}" ]]; then
  log "codesign identity=${SIGN_IDENTITY}"
  codesign --force --deep --sign "${SIGN_IDENTITY}" "${APP}"
fi

log "Done: ${APP}"
log "Run: open \"${APP}\"   or   \"${MACOS}/autoviz\""
