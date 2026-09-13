#!/usr/bin/env bash
# One-shot OrbisView mock backend + Vite frontend.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
ORBIS="$ROOT/autonomy/orbisview"
BUILD="$ROOT/build-orbisview"
BIN="$BUILD/bin/autonomy.orbisview"
PORT="${ORBISVIEW_PORT:-8766}"
HOST="${ORBISVIEW_HOST:-127.0.0.1}"

BE_PID=""
FE_PID=""

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

echo "[orbisview] frontend  http://127.0.0.1:5173  (Connect → ws://$HOST:$PORT/ws)"
npm start &
FE_PID=$!

echo ""
echo "  Open http://127.0.0.1:5173 → Connect"
echo "  Ctrl+C to stop both"
echo ""

wait "$FE_PID"
