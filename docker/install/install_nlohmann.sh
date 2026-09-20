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

THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
THREAD_NUM=$(nproc)

if [[ -f "${INSTALL_PREFIX}/include/nlohmann/json.hpp" ]]; then
    ok "nlohmann_json already installed under ${INSTALL_PREFIX}, skipping"
    exit 0
fi

info "Installing nlohmann_json -> ${INSTALL_PREFIX}"

cd "${THIRDPARTY}"
if [[ ! -d json/.git ]]; then
    rm -rf json
    git_clone_with_retry https://github.com/nlohmann/json.git v3.12.0 json
fi
cd json
git submodule init && git submodule update || true

rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    -DJSON_BuildTests=OFF \
    ..

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

ok "Successfully installed nlohmann_json -> ${INSTALL_PREFIX}"
