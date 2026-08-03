#!/usr/bin/env bash
# Mesa software GL (llvmpipe) for autoviz in Docker without GPU passthrough.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  libgl1-mesa-dri \
  libglx-mesa0 \
  mesa-utils
rm -rf /var/lib/apt/lists/*

ok "Mesa software GL packages installed (llvmpipe)"
