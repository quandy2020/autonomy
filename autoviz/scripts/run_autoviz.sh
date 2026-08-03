#!/usr/bin/env bash
# Launch autoviz with Mesa software GL defaults (Docker / no GPU passthrough).
set -euo pipefail

AUTOVIZ_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AUTONOMY_ROOT="$(cd "${AUTOVIZ_PKG}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${AUTONOMY_ROOT}/../.." && pwd)"
BUILD="${BUILD_DIR:-${AUTOVIZ_PKG}/build}"
BIN="${BUILD}/bin/autoviz"
SRC="${AUTOVIZ_PKG}/autoviz/platform/opengl_setup.cpp"

if [[ ! -x "${BIN}" ]]; then
  BIN="${WORKSPACE_ROOT}/build/bin/autoviz"
fi
if [[ ! -x "${BIN}" ]]; then
  echo "autoviz binary not found; build with:" >&2
  echo "  cmake --build ${AUTOVIZ_PKG}/build --target autoviz" >&2
  exit 1
fi

if [[ -f "${SRC}" && "${SRC}" -nt "${BIN}" ]]; then
  echo "autoviz: rebuilding (OpenGL setup changed)..." >&2
  cmake --build "${BUILD}" --target autoviz -j"$(nproc)"
fi

if [[ -z "${AUTOVIZ_USE_HARDWARE_GL:-}" ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export AUTOVIZ_SOFTWARE_GL="${AUTOVIZ_SOFTWARE_GL:-1}"
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-mesa}"
  export QT_OPENGL="${QT_OPENGL:-software}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
  export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
  # Quick3D needs extra GLX contexts; disable by default in Docker/Mesa.
  export AUTOVIZ_DISABLE_QML="${AUTOVIZ_DISABLE_QML:-1}"
fi

export AUTOVIZ_ALLOW_ROOT_GUI=1
"${AUTOVIZ_PKG}/scripts/check_gui_env.sh"

cd "${AUTOVIZ_PKG}"
exec "${BIN}" "$@"
