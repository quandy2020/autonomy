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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "${CURR_DIR}/installer_base.sh"

THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"

cd "${THIRDPARTY}"
if [[ ! -d assimp/.git ]]; then
  git clone -b v6.0.0 https://github.com/assimp/assimp.git
fi
cd assimp && git submodule update --init --recursive

mkdir -p build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    ..

make -j"$(nproc)"
autonomy_make_install

ok "Successfully installed assimp -> ${INSTALL_PREFIX}"
