#!/usr/bin/env bash
# Verify X11 + Mesa GLX before launching Qt/OpenGL GUI apps in Docker.
set -euo pipefail

fail() {
  echo "check_gui_env: $*" >&2
  exit 1
}

warn() {
  echo "check_gui_env: warning: $*" >&2
}

if [[ -z "${DISPLAY:-}" ]]; then
  fail "DISPLAY is unset. On the host run: xhost +local:docker"
fi

if ! command -v xdpyinfo >/dev/null 2>&1; then
  warn "xdpyinfo not installed (apt-get install -y x11-utils)"
else
  if ! xdpyinfo >/dev/null 2>&1; then
    fail "Cannot connect to DISPLAY=${DISPLAY}. On the host: xhost +local:docker"
  fi
fi

# Isaac/NVIDIA images ship NVIDIA GLX; force Mesa when using software rendering.
if [[ -z "${AUTOVIZ_USE_HARDWARE_GL:-}" ]]; then
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-mesa}"
  export QT_OPENGL="${QT_OPENGL:-software}"
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
fi

if command -v glxinfo >/dev/null 2>&1; then
  if ! renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -1)"; then
    warn "glxinfo failed; install: bash src/autonomy/docker/install/install_mesa_gl.sh"
  elif [[ -n "${renderer}" ]]; then
    echo "check_gui_env: OpenGL renderer: ${renderer}"
    if [[ "${renderer}" == *NVIDIA* && -z "${AUTOVIZ_USE_HARDWARE_GL:-}" ]]; then
      fail "NVIDIA GLX is active but GPU passthrough is off. Set __GLX_VENDOR_LIBRARY_NAME=mesa and LIBGL_ALWAYS_SOFTWARE=1 (use scripts/run_autoviz.sh)."
    fi
  fi
else
  warn "glxinfo missing; run: bash src/autonomy/docker/install/install_mesa_gl.sh"
fi

if [[ "$(id -u)" -eq 0 && -z "${AUTOVIZ_ALLOW_ROOT_GUI:-}" ]]; then
  warn "Running GUI as root; if GLX fails, restart container with: python3 run_autonomy.py --as-host-user"
fi

echo "check_gui_env: OK (DISPLAY=${DISPLAY}) — use ./scripts/run_autoviz.sh to launch"
