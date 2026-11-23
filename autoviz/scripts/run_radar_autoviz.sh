#!/usr/bin/env bash
# Run autoviz radar layout inside SpaceHero with host X11.
# Fixed Frame = radar_link (UM10034: X lateral, Y up, Z out of board).
#
# Host (once per login): xhost +local:
set -euo pipefail

ROOT="${AUTONOMY_ROOT:-/workspace/autonomy}"
CONFIG="${1:-$ROOT/src/autonomy/autoviz/config/radar.autoviz}"
BIN="${AUTOVIZ_BIN:-$ROOT/build/autonomy/bin/autoviz}"

export DISPLAY="${DISPLAY:-:0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export AUTOVIZ_ALLOW_ROOT_GUI="${AUTOVIZ_ALLOW_ROOT_GUI:-1}"
export LD_LIBRARY_PATH="$ROOT/build/autonomy/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "$ROOT/.docker.xauth" ]]; then
  export XAUTHORITY="$ROOT/.docker.xauth"
fi

if [[ ! -S /tmp/.X11-unix/X0 && ! -S /tmp/.X11-unix/X${DISPLAY#:} ]]; then
  echo "No X11 socket under /tmp/.X11-unix — remount /tmp/.X11-unix into SpaceHero." >&2
  exit 1
fi

if [[ ! -x "$BIN" ]]; then
  echo "autoviz not found: $BIN" >&2
  exit 1
fi

exec "$BIN" --config "$CONFIG"
