#!/usr/bin/env bash
# Shared helpers for Autoviz Linux deploy scripts.
# shellcheck shell=bash

set -euo pipefail

_linux_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOVIZ_ROOT="$(cd "${_linux_script_dir}/../.." && pwd)"

autoviz_version() {
  local v
  v="$(sed -nE 's/^project\(autoviz VERSION ([0-9.]+).*/\1/p' \
    "${AUTOVIZ_ROOT}/CMakeLists.txt" | head -1)"
  printf '%s\n' "${v:-0.1.0}"
}

# Resolve the CMake build tree that contains bin/autoviz.
# Order: AUTOVIZ_BUILD_DIR → autoviz/build → monorepo build/autonomy → AUTONOMY_BUILD_DIR.
default_build_dir() {
  local candidates=()
  if [[ -n "${AUTOVIZ_BUILD_DIR:-}" ]]; then
    candidates+=("${AUTOVIZ_BUILD_DIR}")
  fi
  candidates+=("${AUTOVIZ_ROOT}/build")
  # src/autonomy/autoviz → <workspace>/build/autonomy
  if [[ -n "${AUTONOMY_BUILD_DIR:-}" ]]; then
    candidates+=("${AUTONOMY_BUILD_DIR}")
  fi
  local workspace
  workspace="$(cd "${AUTOVIZ_ROOT}/../../.." && pwd)"
  candidates+=("${workspace}/build/autonomy")
  candidates+=("${workspace}/build")

  local c
  for c in "${candidates[@]}"; do
    if [[ -x "${c}/bin/autoviz" ]]; then
      printf '%s\n' "${c}"
      return 0
    fi
  done
  # Fall back so callers can print a clear "missing binary" error.
  printf '%s\n' "${AUTOVIZ_BUILD_DIR:-${AUTOVIZ_ROOT}/build}"
}

default_dist_dir() {
  printf '%s\n' "${AUTOVIZ_DIST_DIR:-${AUTOVIZ_ROOT}/dist/linux}"
}

nproc_jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    printf '%s\n' 4
  fi
}

require_linux() {
  [[ "$(uname -s)" == "Linux" ]] || die "Linux only"
}

log() { printf '==> %s\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

apt_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    printf '%s\n' apt-get
  else
    command -v sudo >/dev/null 2>&1 || die "need root or sudo"
    printf '%s\n' "sudo apt-get"
  fi
}

# Copy a shared library (resolving symlinks) and recreate SONAME / .so links.
# Prefer ELF SONAME (e.g. libglog.so.0.6.0 → SONAME libglog.so.1) over the
# first numeric component of the filename, which is often wrong for glog.
copy_shared_lib() {
  local src="$1"
  local dest_dir="$2"
  local extra_link="${3:-}"   # optional: force-create this basename → real file
  [[ -e "${src}" ]] || return 1
  mkdir -p "${dest_dir}"
  local real base stem soname
  real="$(readlink -f "${src}")"
  base="$(basename "${real}")"
  install -m 0644 "${real}" "${dest_dir}/${base}"

  stem="$(printf '%s' "${base}" | sed -E 's/\.so(\..*)?$//')"
  ln -sfn "${base}" "${dest_dir}/${stem}.so"

  soname=""
  if command -v readelf >/dev/null 2>&1; then
    soname="$(readelf -d "${dest_dir}/${base}" 2>/dev/null \
      | sed -n 's/.*SONAME[^[]*\[\([^]]*\)\].*/\1/p' | head -1)"
  fi
  if [[ -z "${soname}" ]] && command -v objdump >/dev/null 2>&1; then
    soname="$(objdump -p "${dest_dir}/${base}" 2>/dev/null \
      | awk '/SONAME/ {print $2; exit}')"
  fi
  # Never overwrite the real shared object with a symlink to itself
  # (common when SONAME == basename, e.g. libOgreMain.so.1.12.10).
  if [[ -n "${soname}" && "${soname}" != "${base}" ]]; then
    ln -sfn "${base}" "${dest_dir}/${soname}"
  elif [[ -z "${soname}" && "${base}" =~ \.so\.([0-9]+)(\.|$) ]]; then
    local major_link="${stem}.so.${BASH_REMATCH[1]}"
    if [[ "${major_link}" != "${base}" ]]; then
      ln -sfn "${base}" "${dest_dir}/${major_link}"
    fi
  fi

  if [[ -n "${extra_link}" && "${extra_link}" != "${base}" ]]; then
    ln -sfn "${base}" "${dest_dir}/${extra_link}"
  fi
  return 0
}

# Locate libNAME.so* under the build tree (prefer build/lib).
find_build_lib() {
  local build_dir="$1"
  local stem="$2"   # e.g. libautoviz
  if [[ -e "${build_dir}/lib/${stem}.so" ]]; then
    printf '%s\n' "${build_dir}/lib/${stem}.so"
    return 0
  fi
  local found
  found="$(find "${build_dir}/lib" "${build_dir}/_deps" \
    \( -path '*/CMakeFiles/*' \) -prune -o \
    -name "${stem}.so*" \( -type f -o -type l \) -print 2>/dev/null | head -1)"
  if [[ -n "${found}" ]]; then
    printf '%s\n' "${found}"
    return 0
  fi
  return 1
}

# Stage a relocatable / Debian prefix tree under DEST (e.g. AppDir or deb/usr).
# Layout:
#   DEST/bin/autoviz                 wrapper
#   DEST/lib/autoviz/autoviz         real binary
#   DEST/lib/autoviz/*.so*           project + vendored Ogre
#   DEST/lib/autoviz/ogre/           Ogre plugins
#   DEST/share/autonomy/autoviz/     media + default.autoviz
#   DEST/share/applications|metainfo|icons|mime
stage_autoviz_prefix() {
  local dest="$1"
  local build_dir="$2"
  local mode="${3:-bundle}"   # bundle | deb

  [[ -x "${build_dir}/bin/autoviz" ]] || die "missing ${build_dir}/bin/autoviz"

  local lib_priv="${dest}/lib/autoviz"
  local ogre_plug="${lib_priv}/ogre"
  mkdir -p "${dest}/bin" "${lib_priv}" "${ogre_plug}" \
    "${dest}/share/autonomy/autoviz" \
    "${dest}/share/applications" \
    "${dest}/share/metainfo" \
    "${dest}/share/mime/packages"

  install -m 0755 "${build_dir}/bin/autoviz" "${lib_priv}/autoviz"

  local stem src
  for stem in libautoviz libautolink libautomsgs libOgreMain libOgreOverlay; do
    src="$(find_build_lib "${build_dir}" "${stem}" || true)"
    if [[ -n "${src}" && -e "${src}" ]]; then
      copy_shared_lib "${src}" "${lib_priv}"
      log "Staged ${stem}"
    else
      case "${stem}" in
        libautoviz|libautolink|libautomsgs) die "required library not found: ${stem}" ;;
        *) log "Optional library missing: ${stem}" ;;
      esac
    fi
  done

  # Recursively vendor non-system runtime libs so a Jammy-built .deb still
  # starts on Noble (FFmpeg/codec SONAMEs differ across LTS). Keep Qt / GL /
  # X11 / libc on the host; skip CUDA.
  keep_system_lib() {
    local name="$1" path="$2"
    case "${path}" in
      /usr/local/cuda/*|/usr/local/cuda-*) return 0 ;;
    esac
    case "${name}" in
      linux-vdso.so*|ld-linux*.so*|libc.so*|libm.so*|libdl.so*|librt.so*|libpthread.so*|libresolv.so*|libutil.so*)
        return 0 ;;
      libgcc_s.so*|libstdc++.so*|libatomic.so*|libgomp.so*)
        return 0 ;;
      libQt6*.so*) return 0 ;;
      libGL.so*|libGLX.so*|libOpenGL.so*|libEGL.so*|libGLdispatch.so*|libOpenCL.so*)
        return 0 ;;
      libX*.so*|libxcb*.so*|libxkbcommon*.so*) return 0 ;;
      libglib-2.0.so*|libgobject-2.0.so*|libgio-2.0.so*|libgmodule-2.0.so*|libffi.so*)
        return 0 ;;
      # Note: do NOT keep libicu* — SONAME differs (Jammy .70 vs Noble .74).
      libz.so*|libzstd.so*|libbz2.so*|liblzma.so*|liblz4.so*|libpng*.so*|libjpeg*.so*|libwebp*.so*)
        return 0 ;;
      libfreetype.so*|libfontconfig.so*|libharfbuzz.so*|libexpat.so*|libuuid.so*|libpcre*.so*)
        return 0 ;;
      libdbus-1.so*|libsystemd.so*|libudev.so*|libselinux.so*|libmount.so*|libblkid.so*)
        return 0 ;;
      libdouble-conversion.so*|libb2.so*|libmd4c.so*|libbrotli*.so*|libproxy.so*)
        return 0 ;;
      libdrm.so*|libva*.so*|libvdpau.so*|libnuma.so*|libasound.so*|libpulse*.so*)
        return 0 ;;
    esac
    return 1
  }

  local needed_lib needed_path vendor_line pass newly
  for pass in 1 2 3 4 5 6 7 8; do
    newly=0
    while IFS= read -r vendor_line || [[ -n "${vendor_line}" ]]; do
      needed_lib="${vendor_line%% *}"
      needed_path="${vendor_line#* }"
      [[ -n "${needed_lib}" && -n "${needed_path}" && -e "${needed_path}" ]] || continue
      [[ -e "${lib_priv}/${needed_lib}" ]] && continue
      keep_system_lib "${needed_lib}" "${needed_path}" && continue
      copy_shared_lib "${needed_path}" "${lib_priv}" "${needed_lib}"
      log "Vendored lib: ${needed_lib} ← ${needed_path}"
      newly=1
    done < <(
      {
        ldd "${lib_priv}/autoviz" 2>/dev/null || true
        shopt -s nullglob
        for src in "${lib_priv}"/*.so*; do
          [[ -f "${src}" && ! -L "${src}" ]] || continue
          ldd "${src}" 2>/dev/null || true
        done
        shopt -u nullglob
      } | awk '/=>/ && $3 ~ /^\// {print $1, $3}' | sort -u
    )
    [[ "${newly}" -eq 1 ]] || break
  done

  local plugin
  for plugin in RenderSystem_GL.so RenderSystem_GL3Plus.so Codec_STBI.so; do
    src=""
    if [[ -e "${build_dir}/lib/${plugin}" ]]; then
      src="${build_dir}/lib/${plugin}"
    else
      src="$(find "${build_dir}/lib" "${build_dir}/_deps" -name "${plugin}*" \
        \( -type f -o -type l \) 2>/dev/null | head -1 || true)"
    fi
    if [[ -n "${src}" && -e "${src}" ]]; then
      local real
      real="$(readlink -f "${src}")"
      install -m 0644 "${real}" "${ogre_plug}/$(basename "${real}")"
      ln -sfn "$(basename "${real}")" "${ogre_plug}/${plugin}"
      log "Staged Ogre plugin ${plugin}"
    fi
  done

  if command -v patchelf >/dev/null 2>&1; then
    patchelf --set-rpath '$ORIGIN' "${lib_priv}/autoviz" 2>/dev/null || true
    shopt -s nullglob
    for src in "${lib_priv}"/*.so*; do
      [[ -f "${src}" && ! -L "${src}" ]] || continue
      patchelf --set-rpath '$ORIGIN' "${src}" 2>/dev/null || true
    done
    shopt -u nullglob
  fi

  if command -v strip >/dev/null 2>&1; then
    strip --strip-unneeded "${lib_priv}/autoviz" 2>/dev/null || true
    shopt -s nullglob
    for src in "${lib_priv}"/*.so* "${ogre_plug}"/*.so*; do
      [[ -f "${src}" && ! -L "${src}" ]] || continue
      strip --strip-unneeded "${src}" 2>/dev/null || true
    done
    shopt -u nullglob
  fi

  # Desktop / AppStream from build tree if generated, else templates.
  if [[ -f "${build_dir}/org.autonomy.autoviz.desktop" ]]; then
    install -m 0644 "${build_dir}/org.autonomy.autoviz.desktop" \
      "${dest}/share/applications/org.autonomy.autoviz.desktop"
  elif [[ -f "${build_dir}/autoviz/org.autonomy.autoviz.desktop" ]]; then
    install -m 0644 "${build_dir}/autoviz/org.autonomy.autoviz.desktop" \
      "${dest}/share/applications/org.autonomy.autoviz.desktop"
  else
    sed -e "s|@AUTOVIZ_APP_DESCRIPTION@|Autolink native 3D robot visualizer|g" \
      "${_linux_script_dir}/org.autonomy.autoviz.desktop.in" \
      > "${dest}/share/applications/org.autonomy.autoviz.desktop"
  fi

  if [[ -f "${build_dir}/org.autonomy.autoviz.appdata.xml" ]]; then
    install -m 0644 "${build_dir}/org.autonomy.autoviz.appdata.xml" \
      "${dest}/share/metainfo/org.autonomy.autoviz.appdata.xml"
  elif [[ -f "${build_dir}/autoviz/org.autonomy.autoviz.appdata.xml" ]]; then
    install -m 0644 "${build_dir}/autoviz/org.autonomy.autoviz.appdata.xml" \
      "${dest}/share/metainfo/org.autonomy.autoviz.appdata.xml"
  else
    local ver
    ver="$(autoviz_version)"
    sed -e "s|@AUTOVIZ_VERSION@|${ver}|g" \
        -e "s|@AUTOVIZ_BUILD_DATE@|$(date -u +%Y-%m-%d)|g" \
      "${_linux_script_dir}/org.autonomy.autoviz.appdata.xml.in" \
      > "${dest}/share/metainfo/org.autonomy.autoviz.appdata.xml"
  fi

  install -m 0644 "${_linux_script_dir}/org.autonomy.autoviz.mime.xml" \
    "${dest}/share/mime/packages/org.autonomy.autoviz.xml"

  # App icon: same squirrel artwork as macOS (resources/icons/aviz*.png / .icns).
  # Do NOT fall back to deploy/linux/autoviz.svg (legacy robot mark).
  local icon_src size icon_dest
  if [[ -f "${AUTOVIZ_ROOT}/resources/icons/aviz.svg" ]]; then
    install -m 0644 "${AUTOVIZ_ROOT}/resources/icons/aviz.svg" \
      "${dest}/share/icons/hicolor/scalable/apps/aviz.svg"
  fi
  for size in 32 64 128 256; do
    icon_src="${AUTOVIZ_ROOT}/resources/icons/aviz_${size}.png"
    [[ -f "${icon_src}" ]] || continue
    icon_dest="${dest}/share/icons/hicolor/${size}x${size}/apps"
    mkdir -p "${icon_dest}"
    install -m 0644 "${icon_src}" "${icon_dest}/aviz.png"
  done
  # 512×512 master (also used as 48×48 / 512 fallback for theme lookup).
  if [[ -f "${AUTOVIZ_ROOT}/resources/icons/aviz.png" ]]; then
    for size in 48 512; do
      icon_dest="${dest}/share/icons/hicolor/${size}x${size}/apps"
      mkdir -p "${icon_dest}"
      install -m 0644 "${AUTOVIZ_ROOT}/resources/icons/aviz.png" \
        "${icon_dest}/aviz.png"
    done
  fi
  if [[ -f "${AUTOVIZ_ROOT}/resources/icons/aviz_1024.png" ]]; then
    icon_dest="${dest}/share/icons/hicolor/1024x1024/apps"
    mkdir -p "${icon_dest}"
    install -m 0644 "${AUTOVIZ_ROOT}/resources/icons/aviz_1024.png" \
      "${icon_dest}/aviz.png"
  fi

  install -m 0644 "${AUTOVIZ_ROOT}/config/default.autoviz" \
    "${dest}/share/autonomy/autoviz/default.autoviz"
  rm -rf "${dest}/share/autonomy/autoviz/ogre_media"
  cp -a "${AUTOVIZ_ROOT}/resources/ogre_media" \
    "${dest}/share/autonomy/autoviz/ogre_media"

  # Autolink conf (required at process start; build tree path is not portable).
  local autolink_conf_src
  autolink_conf_src="$(cd "${AUTOVIZ_ROOT}/../autolink/autolink/conf" 2>/dev/null && pwd || true)"
  if [[ -z "${autolink_conf_src}" || ! -f "${autolink_conf_src}/autolink.pb.conf" ]]; then
    die "missing autolink conf (expected ../autolink/autolink/conf/autolink.pb.conf)"
  fi
  rm -rf "${dest}/share/autolink/conf"
  mkdir -p "${dest}/share/autolink"
  cp -a "${autolink_conf_src}" "${dest}/share/autolink/conf"
  # Hardcoded fallback used by GlobalData::InitConfig when AUTOLINK_PATH is unset.
  if [[ "${mode}" == "deb" ]]; then
    mkdir -p "${dest}/local/share/autolink"
    rm -rf "${dest}/local/share/autolink/conf"
    cp -a "${autolink_conf_src}" "${dest}/local/share/autolink/conf"
  fi

  # Wrapper: private libs + installed media / plugins (overrides baked build paths).
  cat > "${dest}/bin/autoviz" <<EOF
#!/bin/bash
# Autoviz launcher (generated by deploy/linux).
set -euo pipefail
if [ -n "\${APPDIR:-}" ]; then
  _ROOT="\${APPDIR}"
elif command -v readlink >/dev/null 2>&1; then
  _ROOT="\$(cd "\$(dirname "\$(readlink -f "\$0")")/.." && pwd)"
else
  _ROOT="\$(cd "\$(dirname "\$0")/.." && pwd)"
fi
export LD_LIBRARY_PATH="\${_ROOT}/lib/autoviz:\${LD_LIBRARY_PATH:-}"
export AUTOLINK_PATH="\${AUTOLINK_PATH:-\${_ROOT}/share/autolink}"
export AUTOVIZ_OGRE_MEDIA_PATH="\${AUTOVIZ_OGRE_MEDIA_PATH:-\${_ROOT}/share/autonomy/autoviz/ogre_media}"
export AUTOVIZ_OGRE_PLUGIN_DIR="\${AUTOVIZ_OGRE_PLUGIN_DIR:-\${_ROOT}/lib/autoviz/ogre}"
export AUTOVIZ_RESOURCE_PATH="\${AUTOVIZ_RESOURCE_PATH:-\${_ROOT}/share/autonomy:\${_ROOT}/share}"
if [ "\${XDG_SESSION_TYPE:-}" = "wayland" ]; then
  export QT_QPA_PLATFORM="\${QT_QPA_PLATFORM:-xcb}"
fi
exec "\${_ROOT}/lib/autoviz/autoviz" "\$@"
EOF
  chmod 0755 "${dest}/bin/autoviz"
}
