#!/usr/bin/env bash
# One-shot live stack: Argus LIO + Autoviz + Oxford Spires dataset publisher.
#
# Usage:
#   ./scripts/run_spires_live.sh                 # start all, block until Ctrl+C
#   ./scripts/run_spires_live.sh stop
#   ./scripts/run_spires_live.sh status
#   RATE=0.5 SEQ_PATH=/data/Oxford/sequences/... ./scripts/run_spires_live.sh
#   ./scripts/run_spires_live.sh --rate 1.0 --loop --no-autoviz
#
# Env overrides:
#   SEQ_PATH / ARGUS_SPIRES_SEQ_PATH   sequence directory
#   RATE / ARGUS_LIVE_RATE            playback rate (default 0.5)
#   ARGUS_LIVO_CONFIG                 Livo YAML (default livo_esikf_spires.yaml)
#   AUTOVIZ_CONFIG                    .autoviz session (default config/argus.autoviz)
#   LOG_DIR                           logs (default /tmp/argus_live)
#   NO_AUTOVIZ=1                      skip Autoviz
#   NO_IMAGE=1                        skip camera publish (default: publish images)
#   LOOP=1                            loop dataset forever
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_argus_env.sh
source "${SCRIPT_DIR}/_argus_env.sh"

# ARGUS_REPO = src/autonomy → workspace root is ../..
WORKSPACE="$(cd "${ARGUS_REPO}/../.." && pwd)"
if [[ ! -d "${WORKSPACE}/src/autonomy" && -d /workspace/autonomy/src/autonomy ]]; then
  WORKSPACE=/workspace/autonomy
fi

BUILD_BIN="${WORKSPACE}/build/autonomy/bin"
BUILD_LIB="${WORKSPACE}/build/autonomy/lib"
INSTALL_LIB="${WORKSPACE}/install/autonomy/lib"
INSTALL_PY="${WORKSPACE}/install/autonomy/python"
SRC_AUTONOMY="${WORKSPACE}/src/autonomy"

LOG_DIR="${LOG_DIR:-/tmp/argus_live}"
PID_DIR="${LOG_DIR}/pids"
RATE="${RATE:-${ARGUS_LIVE_RATE:-0.5}}"
LOOP="${LOOP:-0}"
NO_AUTOVIZ="${NO_AUTOVIZ:-0}"
NO_IMAGE="${NO_IMAGE:-0}"
MAX_FRAMES="${MAX_FRAMES:-}"

# Prefer container mount; else downloads layout from _argus_env.
DEFAULT_SEQ="/data/Oxford/sequences/2024-03-12-keble-college-02"
if [[ ! -d "${DEFAULT_SEQ}" ]]; then
  DEFAULT_SEQ="${SPIRES_DEST}/sequences/${SPIRES_SEQ}"
fi
SEQ_PATH="${SEQ_PATH:-${ARGUS_SPIRES_SEQ_PATH:-${DEFAULT_SEQ}}}"

LIVO_CFG="${ARGUS_LIVO_CONFIG:-${LIVO_CONFIG}}"
AUTOVIZ_CFG="${AUTOVIZ_CONFIG:-${SRC_AUTONOMY}/autoviz/config/argus.autoviz}"

LOC_BIN=""
for c in \
  "${BUILD_BIN}/autonomy.localization" \
  /usr/local/bin/autonomy.localization \
  "$(command -v autonomy.localization 2>/dev/null || true)"; do
  if [[ -n "${c}" && -x "${c}" ]]; then
    LOC_BIN="${c}"
    break
  fi
done

AUTOVIZ_BIN=""
for c in \
  "${BUILD_BIN}/autoviz" \
  /usr/local/bin/autoviz \
  "$(command -v autoviz 2>/dev/null || true)"; do
  if [[ -n "${c}" && -x "${c}" ]]; then
    AUTOVIZ_BIN="${c}"
    break
  fi
done

usage() {
  sed -n '2,16p' "$0" | sed 's/^# \?//'
  exit "${1:-0}"
}

kill_exact() {
  local pat="$1"
  local pids
  pids=$(ps -eo pid=,args= | awk -v p="$pat" 'index($0, p) > 0 { print $1 }')
  if [[ -n "${pids}" ]]; then
    # shellcheck disable=SC2086
    kill -TERM ${pids} 2>/dev/null || true
    sleep 0.5
    # shellcheck disable=SC2086
    kill -9 ${pids} 2>/dev/null || true
  fi
}

stop_all() {
  echo "[live] stopping Argus / Autoviz / dataset publisher..."
  kill_exact "autonomy.localization --localization_mode=argus"
  kill_exact "${LOC_BIN} --localization_mode"
  kill_exact "python3 -m autonomy.tools.datasets publish"
  kill_exact "/usr/bin/python3 -m autonomy.tools.datasets publish"
  kill_exact "${AUTOVIZ_BIN} --config"
  kill_exact "autoviz --config"
  # Best-effort: Autoviz may be started without matching argv pattern.
  if [[ -f "${PID_DIR}/autoviz.pid" ]]; then
    local pid
    pid=$(cat "${PID_DIR}/autoviz.pid" 2>/dev/null || true)
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 0.3
      kill -9 "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${PID_DIR}"/*.pid 2>/dev/null || true
  echo "[live] stopped"
}

status_all() {
  echo "[live] workspace=${WORKSPACE}"
  echo "[live] seq=${SEQ_PATH}"
  echo "[live] rate=${RATE} loop=${LOOP} no_autoviz=${NO_AUTOVIZ}"
  echo "[live] config=${LIVO_CFG}"
  echo "[live] autoviz_cfg=${AUTOVIZ_CFG}"
  echo "[live] logs=${LOG_DIR}"
  for name in argus datasets autoviz; do
    local pid_file="${PID_DIR}/${name}.pid"
    if [[ -f "${pid_file}" ]] && kill -0 "$(cat "${pid_file}")" 2>/dev/null; then
      echo "  ${name}: RUNNING pid=$(cat "${pid_file}")"
    else
      echo "  ${name}: stopped"
    fi
  done
  pgrep -af "autonomy.localization --localization_mode=argus" || true
  pgrep -af "autonomy.tools.datasets publish" || true
  pgrep -af "[a]utoviz" || true
}

setup_env() {
  export GLOG_logtostderr=1
  export LD_LIBRARY_PATH="${BUILD_LIB}:${INSTALL_LIB}:/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export PYTHONPATH="${SRC_AUTONOMY}:${INSTALL_PY}:${INSTALL_LIB}/python${PYTHONPATH:+:${PYTHONPATH}}"
  export PATH="${BUILD_BIN}:/usr/local/bin:${PATH}"
  # Keep RPATH ../lib consumers (e.g. /usr/local/bin/*) on the fresh SO.
  if [[ -d "${BUILD_LIB}" ]]; then
    cp -af "${BUILD_BIN}/autonomy.localization" /usr/local/bin/ 2>/dev/null || true
    cp -af "${BUILD_LIB}"/libautonomy_localization.so* /usr/local/lib/ 2>/dev/null || true
    cp -af "${BUILD_LIB}"/libargus.so* /usr/local/lib/ 2>/dev/null || true
  fi
}

start_argus() {
  argus_need "${LOC_BIN}"
  argus_need "${LIVO_CFG}"
  : >"${LOG_DIR}/argus.log"
  stdbuf -oL -eL "${LOC_BIN}" \
    --localization_mode=argus \
    --argus_config="${LIVO_CFG}" \
    --argus_imu_topic=/imu \
    --argus_lidar_topic=/points \
    >"${LOG_DIR}/argus.log" 2>&1 &
  echo $! >"${PID_DIR}/argus.pid"
  echo "[live] argus pid=$(cat "${PID_DIR}/argus.pid") log=${LOG_DIR}/argus.log"
}

start_autoviz() {
  if [[ "${NO_AUTOVIZ}" == "1" ]]; then
    echo "[live] Autoviz skipped (NO_AUTOVIZ=1)"
    return 0
  fi
  argus_need "${AUTOVIZ_BIN}"
  argus_need "${AUTOVIZ_CFG}"
  : >"${LOG_DIR}/autoviz.log"
  # GUI on DISPLAY; keep stdout for crash diagnosis.
  stdbuf -oL -eL "${AUTOVIZ_BIN}" --config "${AUTOVIZ_CFG}" \
    >"${LOG_DIR}/autoviz.log" 2>&1 &
  echo $! >"${PID_DIR}/autoviz.pid"
  echo "[live] autoviz pid=$(cat "${PID_DIR}/autoviz.pid") cfg=${AUTOVIZ_CFG}"
}

start_datasets() {
  argus_need "${SEQ_PATH}"
  : >"${LOG_DIR}/datasets.log"
  local -a cmd=(
    python3 -m autonomy.tools.datasets publish
    --dataset spires
    --path "${SEQ_PATH}"
    --rate "${RATE}"
    --image-topic /camera/rgb/image_raw
  )
  if [[ "${NO_IMAGE}" == "1" ]]; then
    cmd+=(--no-image)
  fi
  if [[ "${LOOP}" == "1" ]]; then
    cmd+=(--loop)
  fi
  if [[ -n "${MAX_FRAMES}" ]]; then
    cmd+=(--max-frames "${MAX_FRAMES}")
  fi
  stdbuf -oL -eL "${cmd[@]}" >"${LOG_DIR}/datasets.log" 2>&1 &
  echo $! >"${PID_DIR}/datasets.pid"
  echo "[live] datasets pid=$(cat "${PID_DIR}/datasets.pid") rate=${RATE} image=$([ "${NO_IMAGE}" = 1 ] && echo off || echo on) path=${SEQ_PATH}"
}

wait_running() {
  trap 'echo; stop_all; exit 0' INT TERM
  echo "[live] running — Ctrl+C to stop"
  echo "[live]   tail -f ${LOG_DIR}/argus.log"
  echo "[live]   tail -f ${LOG_DIR}/datasets.log"
  # Wait until Argus or publisher exits (or Ctrl+C).
  while true; do
    local alive=0
    for name in argus datasets; do
      local pf="${PID_DIR}/${name}.pid"
      if [[ -f "${pf}" ]] && kill -0 "$(cat "${pf}")" 2>/dev/null; then
        alive=1
      fi
    done
    if [[ "${alive}" -eq 0 ]]; then
      echo "[live] Argus/datasets exited — check logs in ${LOG_DIR}"
      stop_all
      exit 1
    fi
    sleep 2
  done
}

# ---- argv ----
ACTION="start"
while [[ $# -gt 0 ]]; do
  case "$1" in
    start|stop|status) ACTION="$1"; shift ;;
    -h|--help) usage 0 ;;
    --rate) RATE="$2"; shift 2 ;;
    --seq|--path) SEQ_PATH="$2"; shift 2 ;;
    --config) LIVO_CFG="$2"; shift 2 ;;
    --autoviz-config) AUTOVIZ_CFG="$2"; shift 2 ;;
    --max-frames) MAX_FRAMES="$2"; shift 2 ;;
    --loop) LOOP=1; shift ;;
    --no-autoviz) NO_AUTOVIZ=1; shift ;;
    --no-image) NO_IMAGE=1; shift ;;
    --log-dir) LOG_DIR="$2"; PID_DIR="${LOG_DIR}/pids"; shift 2 ;;
    *)
      echo "unknown arg: $1" >&2
      usage 1
      ;;
  esac
done

mkdir -p "${LOG_DIR}" "${PID_DIR}"

case "${ACTION}" in
  stop) stop_all ;;
  status) status_all ;;
  start)
    stop_all
    setup_env
    echo "[live] workspace=${WORKSPACE}"
    echo "[live] loc_bin=${LOC_BIN}"
    echo "[live] autoviz_bin=${AUTOVIZ_BIN:-<none>}"
    start_argus
    sleep 2
    start_autoviz
    sleep 1
    start_datasets
    # Brief seed hint (non-blocking).
    sleep 3
    if grep -q "ArgusNode started" "${LOG_DIR}/argus.log" 2>/dev/null; then
      grep "ArgusNode started" "${LOG_DIR}/argus.log" | tail -1 || true
    fi
    wait_running
    ;;
  *) usage 1 ;;
esac
