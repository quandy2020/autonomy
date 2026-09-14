#!/usr/bin/env bash
# One-shot OrbisView mock backend + Vite frontend (LAN-reachable by default).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
ORBIS="$ROOT/autonomy/orbisview"
BUILD="$ROOT/build-orbisview"
BIN="$BUILD/bin/autonomy.orbisview"
PORT="${ORBISVIEW_PORT:-8766}"
HOST="${ORBISVIEW_HOST:-0.0.0.0}"
FE_HOST="${ORBISVIEW_FE_HOST:-0.0.0.0}"
FE_PORT="${ORBISVIEW_FE_PORT:-5173}"

BE_PID=""
FE_PID=""

lan_ips() {
  if command -v ipconfig >/dev/null 2>&1; then
    ipconfig getifaddr en0 2>/dev/null || true
    ipconfig getifaddr en1 2>/dev/null || true
  elif command -v hostname >/dev/null 2>&1; then
    hostname -I 2>/dev/null | awk '{print $1}' || true
  fi
}

cleanup() {
  echo ""
  echo "[orbisview] shutting down…"
  [[ -n "$FE_PID" ]] && kill "$FE_PID" 2>/dev/null || true
  [[ -n "$BE_PID" ]] && kill "$BE_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -x "$BIN" ]]; then
  echo "[orbisview] building backend → $BUILD"
  cmake -S "$ORBIS" -B "$BUILD" -DBUILD_TEST=OFF
  cmake --build "$BUILD" --target autonomy.orbisview -j
fi

if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "[orbisview] port $PORT already in use — reuse existing backend"
else
  echo "[orbisview] backend  ws://$HOST:$PORT/ws"
  "$BIN" --mock=true --host="$HOST" --port="$PORT" &
  BE_PID=$!
  sleep 0.6
fi

cd "$ORBIS/frontend"
if [[ ! -d node_modules ]]; then
  echo "[orbisview] npm install…"
  npm install
fi

echo "[orbisview] frontend  http://$FE_HOST:$FE_PORT"
npm start -- --host "$FE_HOST" --port "$FE_PORT" &
FE_PID=$!

IPS="$(lan_ips | tr '\n' ' ' | xargs)"
echo ""
echo "  Local:   http://127.0.0.1:$FE_PORT → Connect (ws://127.0.0.1:$PORT/ws)"
if [[ -n "$IPS" ]]; then
  for ip in $IPS; do
    echo "  LAN:     http://$ip:$FE_PORT → Connect (ws://$ip:$PORT/ws)"
  done
fi
echo "  Ctrl+C to stop both"
echo ""

wait "$FE_PID"
