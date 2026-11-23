#!/usr/bin/env bash
# EVB1122 USB bring-up for JoySpace MmwaveController / autodriver icl1122.
#
# Applies Linux usbcore helpers that often help STM32 CDC boards which fail
# with "device descriptor read/64, error -71", then waits for /dev/ttyACM*.
#
# Usage:
#   ./scripts/evb1122_usb_bringup.sh              # quirks + wait
#   ./scripts/evb1122_usb_bringup.sh --run-controller
#
# Privilege: prefers `sudo`; if passwordless sudo is unavailable and Docker
# is in the group, uses a privileged local image to write /sys.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MMWAVE_ROOT="$ROOT/thirdparty/24GMmwave"
CONTROLLER="$MMWAVE_ROOT/build/MmwaveController"
REGCFG="$MMWAVE_ROOT/config/RegisterConfig16_8_2db_840"
RUN_CONTROLLER=0
WAIT_SEC=60
DOCKER_IMG="${EVB1122_DOCKER_IMG:-autonomy-ops-verify:24.04}"

for arg in "$@"; do
  case "$arg" in
    --run-controller) RUN_CONTROLLER=1 ;;
    --wait=*) WAIT_SEC="${arg#*=}" ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
  esac
done

run_as_root() {
  local script="$1"
  if [[ "$(id -u)" -eq 0 ]]; then
    bash -c "$script"
    return
  fi
  if sudo -n true 2>/dev/null; then
    sudo bash -c "$script"
    return
  fi
  if command -v docker >/dev/null 2>&1 && docker image inspect "$DOCKER_IMG" >/dev/null 2>&1; then
    echo "[priv] using docker --privileged ($DOCKER_IMG) to write usbcore sysfs"
    docker run --rm --privileged \
      -v /sys/module/usbcore:/sys/module/usbcore \
      -v /sys/bus/usb:/sys/bus/usb \
      -v /lib/modules:/lib/modules:ro \
      "$DOCKER_IMG" bash -c "$script"
    return
  fi
  echo "Need root (sudo) or a local Docker image ($DOCKER_IMG) for usbcore params." >&2
  exit 1
}

echo "[1/4] usbcore enumeration helpers + cdc_acm"
run_as_root '
set +e
echo 1 > /sys/module/usbcore/parameters/old_scheme_first
echo 1 > /sys/module/usbcore/parameters/use_both_schemes
echo 10000 > /sys/module/usbcore/parameters/initial_descriptor_timeout
echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null
modprobe cdc_acm 2>/dev/null || true
echo "old_scheme=$(cat /sys/module/usbcore/parameters/old_scheme_first) both=$(cat /sys/module/usbcore/parameters/use_both_schemes) timeout=$(cat /sys/module/usbcore/parameters/initial_descriptor_timeout)"
'

echo "[2/4] hardware checklist (UM10034P / JoySpace)"
cat <<'EOF'
  - Type-C *data* cable on board J14 (charge-only cables → error -71)
  - Power LED D3 on; after plug, short-press reset K1
  - Prefer a rear-panel USB *2.0* port (avoid dock / front hub)
  - If still -71: feed 5V on J1, or USB-TTL UART on J1 @ 921600
  - Unplug now, then re-plug when prompted
EOF

echo
echo "[3/4] waiting ${WAIT_SEC}s for /dev/ttyACM* (re-plug EVB1122 + press K1)..."
deadline=$((SECONDS + WAIT_SEC))
dev=""
while (( SECONDS < deadline )); do
  mapfile -t acms < <(compgen -G '/dev/ttyACM*' || true)
  if ((${#acms[@]} > 0)); then
    dev="${acms[0]}"
    break
  fi
  printf '\r  waiting... %2ds left   ' "$((deadline - SECONDS))"
  sleep 0.5
done
echo

if [[ -z "$dev" ]]; then
  echo "FAIL: no /dev/ttyACM* — kernel never finished enumeration." >&2
  echo "Typical journal lines:" >&2
  journalctl -k -n 40 --no-pager 2>/dev/null | grep -iE 'usb|ttyACM|cdc|error -71|setup address' | tail -20 >&2 || true
  echo >&2
  echo "This is almost always cable / port power / board power — not autodriver." >&2
  exit 2
fi

echo "[4/4] OK: $dev"
lsusb | grep -iE '0483|STMicro|ACM' || lsusb
if [[ -n "${SUDO_USER:-}" ]] || [[ "$(id -u)" -eq 0 ]]; then
  chmod a+rw "$dev" 2>/dev/null || true
fi

if [[ "$RUN_CONTROLLER" -eq 1 ]]; then
  if [[ ! -x "$CONTROLLER" ]]; then
    echo "Building MmwaveController..."
    cmake -S "$MMWAVE_ROOT" -B "$MMWAVE_ROOT/build"
    cmake --build "$MMWAVE_ROOT/build" -j"$(nproc)"
  fi
  [[ -f "$REGCFG" ]] || { echo "Missing $REGCFG" >&2; exit 3; }
  mkdir -p "$MMWAVE_ROOT/rangebin"
  echo "Starting: $CONTROLLER $dev $REGCFG 16 8"
  cd "$MMWAVE_ROOT"
  if [[ -n "${SUDO_USER:-}" && "$(id -u)" -eq 0 ]]; then
    exec sudo -u "$SUDO_USER" "$CONTROLLER" "$dev" "$REGCFG" 16 8
  fi
  exec "$CONTROLLER" "$dev" "$REGCFG" 16 8
fi

echo
echo "Next (JoySpace):"
echo "  cd $MMWAVE_ROOT && mkdir -p rangebin"
echo "  ./build/MmwaveController $dev config/RegisterConfig16_8_2db_840 16 8"
echo "Expect: Got a complete frame  →  files under rangebin/"
