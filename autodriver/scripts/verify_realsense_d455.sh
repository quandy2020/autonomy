#!/usr/bin/env bash
# Verify RealSense D455 via autodriver (host or Docker with USB passthrough).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTODRIVER_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PKG_ROOT="$(cd "${AUTODRIVER_ROOT}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"

INSTALL_PREFIX="${INSTALL_PREFIX:-${WORKSPACE_ROOT}/install/autonomy}"
BUILD_PREFIX="${BUILD_PREFIX:-${WORKSPACE_ROOT}/build/autonomy}"

pick_prefix() {
  if [[ -x "${INSTALL_PREFIX}/bin/autodriver" ]]; then
    echo "${INSTALL_PREFIX}"
    return
  fi
  if [[ -x "${BUILD_PREFIX}/bin/autodriver" ]]; then
    echo "${BUILD_PREFIX}"
    return
  fi
  echo "${INSTALL_PREFIX}"
}

PREFIX="$(pick_prefix)"

export AUTODRIVER_PATH="${AUTODRIVER_ROOT}"
export AUTOLINK_PATH="${PKG_ROOT}/autolink"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${PREFIX}/bin:${PATH}"
export GLOG_logtostderr=1

find_rs_enumerate() {
  if command -v rs-enumerate-devices >/dev/null 2>&1; then
    command -v rs-enumerate-devices
    return 0
  fi
  local candidate
  for candidate in \
    /usr/local/bin/rs-enumerate-devices \
    "${REALSENSE_BUILD:-}/Release/rs-enumerate-devices" \
    "${WORKSPACE_ROOT}/../github/ros2/librealsense/build/Release/rs-enumerate-devices" \
    /home/quandy/workspace/github/ros2/librealsense/build/Release/rs-enumerate-devices; do
    if [[ -n "${candidate}" && -x "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

echo "== RealSense devices =="
if RS_ENUM="$(find_rs_enumerate)"; then
  "${RS_ENUM}" -s
elif command -v lsusb >/dev/null 2>&1; then
  lsusb | grep -iE 'intel|realsense' || {
    echo "No Intel RealSense USB device found (lsusb)" >&2
    exit 1
  }
  echo "(rs-enumerate-devices not installed; using lsusb only)"
else
  echo "Install librealsense2-utils or usbutils (apt install usbutils)" >&2
  exit 1
fi

LIB="${PREFIX}/lib/libautodriver.so"
if [[ ! -f "${LIB}" ]]; then
  echo "libautodriver.so not found under ${PREFIX}/lib" >&2
  exit 1
fi

if ! ldd "${LIB}" 2>/dev/null | grep -qi realsense; then
  echo "libautodriver.so is not linked against librealsense2." >&2
  echo "Rebuild: colcon build --packages-up-to autonomy \\" >&2
  echo "  --cmake-args -DBUILD_AUTODRIVER=ON -DAUTODRIVER_WITH_REALSENSE=ON" >&2
  if ! find_rs_enumerate >/dev/null 2>&1; then
    echo "Also install librealsense2-dev in this environment (host or container)." >&2
  fi
  exit 1
fi

if [[ ! -x "${PREFIX}/bin/autodriver" ]]; then
  echo "autodriver not found under ${PREFIX}/bin" >&2
  exit 1
fi

echo "== Using prefix: ${PREFIX} =="
echo "== Starting autodriver =="
"${PREFIX}/bin/autodriver" &
HUB_PID=$!
trap 'kill ${HUB_PID} >/dev/null 2>&1 || true' EXIT
sleep 4

channels=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_rect_raw
  /camera/depth/camera_info
  /camera/infra1/image_rect_raw
  /camera/infra1/camera_info
  /camera/infra2/image_rect_raw
  /camera/infra2/camera_info
  /camera/aligned_depth_to_color/image_raw
  /camera/aligned_depth_to_color/camera_info
  /camera/depth/color/points
  /camera/imu
)

echo "== Autolink channels =="
autolink channel list -v || true

for ch in "${channels[@]}"; do
  echo "-- echo ${ch} --"
  autolink channel echo "${ch}" --once || {
    echo "failed to receive ${ch}" >&2
    exit 1
  }
done

echo "D455 autodriver verification passed."
