#!/usr/bin/env bash
# Embedded / onboard smoke: lower steps, sample thermal + efficiency.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ATLA2_ROOT="$(cd "${BENCH_ROOT}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${ATLA2_ROOT}/../../../../.." && pwd)"

CONFIG_NAME="${1:-mun_frl}"
STEPS="${STEPS:-50}"
DURATION_THERMAL="${DURATION_THERMAL:-8}"
BIN_DIR="${BIN_DIR:-${WORKSPACE_ROOT}/build/bin}"
PLATFORM="${PLATFORM:-${ATLA2_ROOT}/config/platforms/drone_livo.yaml}"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${BENCH_ROOT}/reports/embedded_${CONFIG_NAME}/${STAMP}"
mkdir -p "${OUT_DIR}"

NODE_BIN="${BIN_DIR}/autonomy.localization.atla2_benchmark"
if [[ ! -x "${NODE_BIN}" ]]; then
  NODE_BIN="${BIN_DIR}/autonomy.localization.atla2_offline"
fi
if [[ ! -x "${NODE_BIN}" ]]; then
  echo "missing atla2 binary under ${BIN_DIR}" >&2
  exit 2
fi

LOG="${OUT_DIR}/run.log"
METRICS="${BENCH_ROOT}/metrics"

echo "[run_embedded] platform=${PLATFORM} steps=${STEPS}" | tee "${LOG}"

# Thermal in background while the run executes.
python3 "${METRICS}/thermal.py" --duration "${DURATION_THERMAL}" --interval 0.5 \
  --out "${OUT_DIR}/thermal.json" &
THERMAL_PID=$!

set +e
"${NODE_BIN}" --config "${PLATFORM}" --steps "${STEPS}" 2>&1 | tee -a "${LOG}"
RC=${PIPESTATUS[0]}
set -e

wait "${THERMAL_PID}" || true

python3 "${METRICS}/efficiency.py" --log "${LOG}" --sample --out "${OUT_DIR}/efficiency.json" || true
python3 "${METRICS}/robustness.py" --log "${LOG}" --out "${OUT_DIR}/robustness.json" || true

echo "[run_embedded] reports -> ${OUT_DIR} (exit=${RC})"
exit "${RC}"
