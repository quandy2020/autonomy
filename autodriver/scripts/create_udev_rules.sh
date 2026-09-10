#!/usr/bin/env bash
# Install udev rule so Slamtec USB-UART appears as /dev/rplidar.
# (Adapted from rplidar_ros/scripts; no ROS required.)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/rplidar.rules"
RULES_DST="/etc/udev/rules.d/rplidar.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "Missing ${RULES_SRC}" >&2
  exit 1
fi

echo "Installing ${RULES_SRC} -> ${RULES_DST}"
echo "After reconnect, check: ls -l /dev/rplidar"
sudo cp "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Done. Unplug/replug the lidar if /dev/rplidar is missing."
