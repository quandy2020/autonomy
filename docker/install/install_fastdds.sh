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

set -euo pipefail

# Install eProsima Fast DDS from source.
# Optional environment variables:
#   FASTDDS_VERSION   (default: v2.6.8)
#   FASTDDS_REPO      (default: https://github.com/eProsima/Fast-DDS.git)
#   INSTALL_PREFIX    (default: /usr/local)
#   FORCE_INSTALL     (default: 0, set to 1 to reinstall even if already found)

FASTDDS_VERSION="${FASTDDS_VERSION:-v2.6.8}"
FASTDDS_REPO="${FASTDDS_REPO:-https://github.com/eProsima/Fast-DDS.git}"
INSTALL_PREFIX="${INSTALL_PREFIX:-/usr/local}"
FORCE_INSTALL="${FORCE_INSTALL:-0}"
BUILD_ROOT="/thirdparty"
SRC_DIR="${BUILD_ROOT}/Fast-DDS"
BUILD_DIR="${SRC_DIR}/build"

echo "[fastdds] version: ${FASTDDS_VERSION}"
echo "[fastdds] repo: ${FASTDDS_REPO}"
echo "[fastdds] install prefix: ${INSTALL_PREFIX}"

if [[ "${FORCE_INSTALL}" != "1" ]] && [[ -f "${INSTALL_PREFIX}/lib/libfastrtps.so" || -f "${INSTALL_PREFIX}/lib/x86_64-linux-gnu/libfastrtps.so" ]]; then
  echo "[fastdds] existing installation detected, skip (set FORCE_INSTALL=1 to reinstall)."
  exit 0
fi

apt-get update && apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  ninja-build \
  libasio-dev \
  libtinyxml2-dev \
  libssl-dev \
  libfoonathan-memory-dev

mkdir -p "${BUILD_ROOT}"
cd "${BUILD_ROOT}"

if [[ -d "${SRC_DIR}" ]]; then
  rm -rf "${SRC_DIR}"
fi

git clone --depth 1 --branch "${FASTDDS_VERSION}" "${FASTDDS_REPO}" Fast-DDS

cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DBUILD_SHARED_LIBS=ON \
  -DTHIRDPARTY=ON \
  -DSECURITY=ON \
  -DCOMPILE_EXAMPLES=OFF \
  -DLOG_NO_INFO=ON \
  -DLOG_NO_WARNING=ON

cmake --build "${BUILD_DIR}" -- -j"$(nproc)"
cmake --install "${BUILD_DIR}"
ldconfig

rm -rf "${BUILD_DIR}"
echo "[fastdds] install complete."
