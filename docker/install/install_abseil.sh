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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Install abseil.
VERSION="20220623.0"
PKG_NAME="abseil-cpp-${VERSION}.tar.gz"
DOWNLOAD_LINK="https://github.com/abseil/abseil-cpp/archive/refs/tags/${VERSION}.tar.gz"
CHECKSUM="f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111"

download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"

DEST_DIR="/usr/local"

tar xzf "${PKG_NAME}"
pushd "abseil-cpp-${VERSION}"
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_INSTALL_PREFIX=${DEST_DIR}
    cmake --build . --target install
popd

ldconfig

# Clean up
rm -rf "abseil-cpp-${VERSION}" "${PKG_NAME}"