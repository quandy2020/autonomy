#!/usr/bin/env bash
# Run Atla2 offline benchmark for a dataset config (euroc/uma_vi/mun_frl/kitti).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ATLA2_ROOT="$(cd "${BENCH_ROOT}/../.." && pwd)"
WORKSPACE_ROOT="$(cd "${ATLA2_ROOT}/../../../../.." && pwd)"

CONFIG_NAME="${1:-euroc}"
STEPS="${STEPS:-200}"
BIN_DIR="${BIN_DIR:-${WORKSPACE_ROOT}/build/bin}"
CONFIG_YAML="${BENCH_ROOT}/configs/${CONFIG_NAME}.yaml"

if [[ ! -f "${CONFIG_YAML}" ]]; then
  echo "unknown config: ${CONFIG_NAME} (expected ${CONFIG_YAML})" >&2
  echo "usage: $0 <euroc|uma_vi|mun_frl|kitti>" >&2
  exit 1
fi

# Resolve platform_config relative to the dataset yaml when possible.
PLATFORM="$(python3 - <<PY
import re
from pathlib import Path
text = Path("${CONFIG_YAML}").read_text()
m = re.search(r"platform_config:\s*(\S+)", text)
print(m.group(1) if m else "")
PY
)"
if [[ -n "${PLATFORM}" && "${PLATFORM}" != /* ]]; then
  PLATFORM="$(cd "$(dirname "${CONFIG_YAML}")" && realpath -m "${PLATFORM}")"
fi
if [[ -z "${PLATFORM}" || ! -f "${PLATFORM}" ]]; then
  PLATFORM="${ATLA2_ROOT}/config/platforms/drone_vision_only.yaml"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${BENCH_ROOT}/reports/${CONFIG_NAME}/${STAMP}"
mkdir -p "${OUT_DIR}"

OFFLINE="${BIN_DIR}/autonomy.localization.atla2_offline"
BENCH_BIN="${BIN_DIR}/autonomy.localization.atla2_benchmark"
RUNNER_BIN="${OFFLINE}"
if [[ -x "${BENCH_BIN}" ]]; then
  RUNNER_BIN="${BENCH_BIN}"
fi
if [[ ! -x "${RUNNER_BIN}" ]]; then
  echo "missing binary: ${RUNNER_BIN} (build atla2 apps first)" >&2
  exit 2
fi

LOG="${OUT_DIR}/run.log"
echo "[run_offline] config=${CONFIG_NAME} platform=${PLATFORM} steps=${STEPS}" | tee "${LOG}"
echo "[run_offline] bin=${RUNNER_BIN}" | tee -a "${LOG}"

set +e
"${RUNNER_BIN}" --config "${PLATFORM}" --steps "${STEPS}" 2>&1 | tee -a "${LOG}"
RC=${PIPESTATUS[0]}
set -e

METRICS="${BENCH_ROOT}/metrics"
python3 "${METRICS}/efficiency.py" --log "${LOG}" --out "${OUT_DIR}/efficiency.json" || true
python3 "${METRICS}/robustness.py" --log "${LOG}" --out "${OUT_DIR}/robustness.json" || true
python3 "${METRICS}/thermal.py" --once --out "${OUT_DIR}/thermal.json" || true

# Accuracy needs est/gt TUM files when available.
if [[ -f "${OUT_DIR}/est.txt" && -f "${OUT_DIR}/gt.txt" ]]; then
  python3 "${METRICS}/accuracy.py" --est "${OUT_DIR}/est.txt" --gt "${OUT_DIR}/gt.txt" \
    --out "${OUT_DIR}/accuracy.json" || true
fi

echo "[run_offline] reports -> ${OUT_DIR} (exit=${RC})"
exit "${RC}"
