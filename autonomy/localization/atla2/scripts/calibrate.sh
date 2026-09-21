#!/usr/bin/env bash
# Launch Atla2 calibration_tool CLIs.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../../.." && pwd)"
BIN="${BIN_DIR:-${ROOT}/build/bin}"

MODE="${1:-}"
BAG="${2:-}"
shift 2 2>/dev/null || true

usage() {
  cat <<EOF
Usage: $0 <camera_imu|lidar_imu|camera_lidar|joint> <bag> [--out yaml]
EOF
}

case "${MODE}" in
  camera_imu) EXE=autonomy.localization.atla2_camera_imu_calib ;;
  lidar_imu) EXE=autonomy.localization.atla2_lidar_imu_calib ;;
  camera_lidar) EXE=autonomy.localization.atla2_camera_lidar_calib ;;
  joint) EXE=autonomy.localization.atla2_joint_calib ;;
  *) usage; exit 1 ;;
esac

if [[ -z "${BAG}" ]]; then
  usage
  exit 1
fi

exec "${BIN}/${EXE}" --bag "${BAG}" "$@"
