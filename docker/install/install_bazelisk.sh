#!/usr/bin/env bash

###############################################################################
# Copyright 2026 The Openbot Authors (duyongquan). All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Install bazelisk as `bazel` under the system prefix (docker image).
# Workspace builds can also auto-fetch into .cache/bin via ./autonomy.sh.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
# shellcheck source=./installer_base.sh
. ./installer_base.sh

VERSION="${BAZELISK_VERSION:-v1.25.0}"
TARGET_ARCH="$(uname -m)"

case "${TARGET_ARCH}" in
  x86_64|amd64) ASSET="bazelisk-linux-amd64" ;;
  aarch64|arm64) ASSET="bazelisk-linux-arm64" ;;
  *)
    error "unsupported arch for bazelisk: ${TARGET_ARCH}"
    exit 1
    ;;
esac

PKG_NAME="${ASSET}"
DOWNLOAD_LINK="https://github.com/bazelbuild/bazelisk/releases/download/${VERSION}/${ASSET}"

info "Installing bazelisk ${VERSION} (${ASSET})..."
download_if_not_cached "${PKG_NAME}" "" "${DOWNLOAD_LINK}"

install -m 0755 "${PKG_NAME}" "${SYSROOT_DIR}/bin/bazelisk"
ln -sfn "${SYSROOT_DIR}/bin/bazelisk" "${SYSROOT_DIR}/bin/bazel"
ok "bazelisk → ${SYSROOT_DIR}/bin/bazel"
rm -f "${PKG_NAME}"
