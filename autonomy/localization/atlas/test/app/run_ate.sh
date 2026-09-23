#!/usr/bin/env bash
# Run atlas_dataset on a sequence, then report ATE via eval_ate.py.
#
# Usage:
#   ./run_ate.sh euroc mono_inertial /path/MH_01_easy [config.yaml]
#   ./run_ate.sh tum_rgbd rgbd /path/rgbd_dataset_freiburg1_desk
#   ./run_ate.sh kitti stereo /path/dataset/sequences/00
#
# Env:
#   ATLAS_DATASET_BIN  path to autonomy.localization.atlas_dataset
#   TRAJ_OUT           output TUM traj (default /tmp/atlas_ate_traj.txt)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ATLAS_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
REPO_ROOT="$(cd "${ATLAS_ROOT}/../../.." && pwd)"

FORMAT="${1:?format: euroc|tum_rgbd|kitti}"
MODE="${2:?mode}"
DATASET="${3:?dataset path}"
CONFIG_OVERRIDE="${4:-}"

BIN="${ATLAS_DATASET_BIN:-}"
if [[ -z "${BIN}" ]]; then
  for cand in \
    "${REPO_ROOT}/bazel-bin/autonomy/localization/atlas/test/app/autonomy.localization.atlas_dataset" \
    "${REPO_ROOT}/bin/autonomy.localization.atlas_dataset" \
    "$(command -v autonomy.localization.atlas_dataset || true)"; do
    if [[ -n "${cand}" && -x "${cand}" ]]; then
      BIN="${cand}"
      break
    fi
  done
fi
if [[ -z "${BIN}" || ! -x "${BIN}" ]]; then
  echo "error: atlas_dataset binary not found; set ATLAS_DATASET_BIN" >&2
  exit 1
fi

TRAJ_OUT="${TRAJ_OUT:-/tmp/atlas_ate_traj.txt}"
CFG_DIR="${ATLAS_ROOT}/config"

pick_config() {
  case "${FORMAT}:${MODE}" in
    euroc:mono|euroc:vo|euroc:rgb) echo "${CFG_DIR}/euroc_mono.yaml" ;;
    euroc:mono_inertial|euroc:vio) echo "${CFG_DIR}/euroc_mono_inertial.yaml" ;;
    euroc:stereo) echo "${CFG_DIR}/euroc_stereo.yaml" ;;
    euroc:stereo_inertial) echo "${CFG_DIR}/euroc_stereo_inertial.yaml" ;;
    tum_rgbd:rgbd) echo "${CFG_DIR}/tum_rgbd1.yaml" ;;
    kitti:mono) echo "${CFG_DIR}/kitti_mono_00-02.yaml" ;;
    kitti:stereo) echo "${CFG_DIR}/kitti_stereo_00-02.yaml" ;;
    *) echo "" ;;
  esac
}

CONFIG="${CONFIG_OVERRIDE:-$(pick_config)}"
if [[ -z "${CONFIG}" || ! -f "${CONFIG}" ]]; then
  echo "error: missing config for ${FORMAT}/${MODE}: ${CONFIG}" >&2
  exit 1
fi

echo "==> ${BIN} --format=${FORMAT} --mode=${MODE}"
echo "    dataset=${DATASET}"
echo "    config=${CONFIG}"
echo "    traj=${TRAJ_OUT}"

"${BIN}" \
  --format="${FORMAT}" \
  --mode="${MODE}" \
  --dataset="${DATASET}" \
  --config="${CONFIG}" \
  --traj_out="${TRAJ_OUT}"

EVAL=(python3 "${SCRIPT_DIR}/eval_ate.py" --est "${TRAJ_OUT}")
case "${FORMAT}" in
  euroc)
    GT="${DATASET}/mav0/state_groundtruth_estimate0/data.csv"
    EVAL+=(--gt_format=euroc --gt "${GT}")
    ;;
  tum_rgbd)
    GT="${DATASET}/groundtruth.txt"
    EVAL+=(--gt_format=tum --fix_scale --gt "${GT}")
    ;;
  kitti)
    # DATASET = .../sequences/XX ; poses at .../poses/XX.txt
    SEQ="$(basename "${DATASET}")"
    POSES="$(cd "${DATASET}/../.." && pwd)/poses/${SEQ}.txt"
    TIMES="${DATASET}/times.txt"
    EVAL+=(--gt_format=kitti --fix_scale --gt "${POSES}" --times "${TIMES}")
    ;;
  *)
    echo "error: unknown format ${FORMAT}" >&2
    exit 1
    ;;
esac

echo "==> ATE"
"${EVAL[@]}"
