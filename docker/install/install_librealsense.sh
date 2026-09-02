#!/usr/bin/env bash
# Install Intel librealsense2 SDK (dev + utils) on Ubuntu 20/22/24.
# Docker: skip dkms (stock uvcvideo is enough for D455).
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg apt-transport-https lsb-release usbutils

install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor -o /etc/apt/keyrings/librealsenseai.gpg
chmod a+r /etc/apt/keyrings/librealsenseai.gpg

codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo ${codename} main" \
  > /etc/apt/sources.list.d/librealsense.list

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  librealsense2 \
  librealsense2-dev \
  librealsense2-utils

ldconfig
echo "librealsense2 installed: $(pkg-config --modversion realsense2 2>/dev/null || true)"
command -v rs-enumerate-devices
rs-enumerate-devices -s || true
