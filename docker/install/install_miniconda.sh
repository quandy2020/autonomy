#!/usr/bin/env bash

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
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

# Fail on first error.
set -e

# Env: MINICONDA_PREFIX (default ${HOME}/miniconda3), MINICONDA_URL, MINICONDA_SHA256, FORCE_INSTALL=1
# Ref: https://www.anaconda.com/docs/getting-started/miniconda/install/linux-install

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. "${CURR_DIR}/installer_base.sh"

MINICONDA_PREFIX="${MINICONDA_PREFIX:-${HOME}/miniconda3}"
FORCE_INSTALL="${FORCE_INSTALL:-0}"

if [[ "${FORCE_INSTALL}" != "1" ]] && [[ -x "${MINICONDA_PREFIX}/bin/conda" ]]; then
  info "Miniconda already at ${MINICONDA_PREFIX}, skip (FORCE_INSTALL=1 to reinstall)."
  exit 0
fi

###############################################################################
# Download tool
###############################################################################
apt_get_update_and_install wget ca-certificates

ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64) SUFFIX="x86_64" ;;
  aarch64 | arm64) SUFFIX="aarch64" ;;
  *)
    error "Unsupported arch: ${ARCH}"
    exit 1
    ;;
esac

INSTALLER="/tmp/Miniconda3-latest-Linux-${SUFFIX}.sh"
URL="${MINICONDA_URL:-https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-${SUFFIX}.sh}"

info "Downloading ${URL}"
wget -q "${URL}" -O "${INSTALLER}"

if [[ -n "${MINICONDA_SHA256:-}" ]]; then
  echo "${MINICONDA_SHA256}  ${INSTALLER}" | sha256sum -c -
fi

chmod +x "${INSTALLER}"

###############################################################################
# Install (batch: -b -p; reinstall: -f)
###############################################################################
if [[ "${FORCE_INSTALL}" == "1" ]]; then
  bash "${INSTALLER}" -b -f -p "${MINICONDA_PREFIX}"
else
  bash "${INSTALLER}" -b -p "${MINICONDA_PREFIX}"
fi

rm -f "${INSTALLER}"

"${MINICONDA_PREFIX}/bin/conda" init bash >/dev/null 2>&1 || true
ok "Miniconda installed: ${MINICONDA_PREFIX} ($("${MINICONDA_PREFIX}/bin/conda" --version))"
