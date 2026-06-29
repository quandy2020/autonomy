#!/usr/bin/env bash
# Verify protobuf 3.19.x is installed; install if missing or wrong version.
set -euo pipefail

INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

need_install() {
  if [[ ! -x /usr/local/bin/protoc ]]; then
    return 0
  fi
  local ver
  ver="$(/usr/local/bin/protoc --version 2>&1 || true)"
  if [[ "${ver}" != *"3.19."* ]]; then
    echo "Wrong protoc: ${ver}"
    return 0
  fi
  if [[ ! -f /usr/local/include/google/protobuf/port_def.inc ]]; then
    return 0
  fi
  return 1
}

if need_install; then
  echo "Installing protobuf 3.19.4 to /usr/local ..."
  bash "${INSTALLERS_DIR}/install_protobuf.sh"
else
  echo "protobuf 3.19.x already installed: $(/usr/local/bin/protoc --version)"
fi
