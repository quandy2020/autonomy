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

if [[ -f "${INSTALL_PREFIX}/lib/libglog.so" ]] \
    || [[ -f /usr/lib/x86_64-linux-gnu/libglog.so ]] \
    || [[ -f /usr/lib/aarch64-linux-gnu/libglog.so ]]; then
    ok "glog already installed, skipping source build"
    exit 0
fi

cd "${THIRDPARTY}"
if [[ ! -d glog ]]; then
    git_clone_with_retry https://github.com/google/glog.git v0.6.0 glog
fi
cd glog && git submodule init && git submodule update

mkdir -p builder && cd builder
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    ..

make -j"${THREAD_NUM}"
if [[ "$(id -u)" -eq 0 ]]; then
    make install
else
    sudo make install
fi

ldconfig 2>/dev/null || true

cd ../.. && rm -rf glog

ok "Successfully installed glog v0.6.0"
