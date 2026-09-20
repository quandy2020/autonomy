#!/usr/bin/env bash
# Ensure aarch64 cross compilers exist inside the (nvidia) container.
set -euo pipefail
if command -v aarch64-linux-gnu-g++ >/dev/null 2>&1; then
  echo "[cross] aarch64-linux-gnu-g++ already present: $(command -v aarch64-linux-gnu-g++)"
  exit 0
fi
export DEBIAN_FRONTEND=noninteractive
if command -v apt-get >/dev/null 2>&1; then
  apt-get update -qq
  apt-get install -y --no-install-recommends \
    gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binutils-aarch64-linux-gnu ninja-build
elif command -v yum >/dev/null 2>&1; then
  yum install -y gcc-aarch64-linux-gnu gcc-c++-aarch64-linux-gnu
else
  echo "[cross] cannot install aarch64-linux-gnu toolchain (no apt/yum)" >&2
  exit 1
fi
