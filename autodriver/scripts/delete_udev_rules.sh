#!/usr/bin/env bash
# Remove /dev/rplidar udev remap.

set -euo pipefail

RULES_DST="/etc/udev/rules.d/rplidar.rules"

echo "Removing ${RULES_DST}"
sudo rm -f "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Done."
