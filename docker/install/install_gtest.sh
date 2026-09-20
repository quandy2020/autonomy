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

# Already under /usr/local — do not re-clone (works offline / bad DNS).
if [[ -f "${INSTALL_PREFIX}/lib/libgtest.so" ]] \
    || [[ -f "${INSTALL_PREFIX}/lib/libgtest.a" ]]; then
    ok "gtest already installed under ${INSTALL_PREFIX}, skipping source build"
    exit 0
fi

cd "${THIRDPARTY}"
if [[ ! -d googletest ]]; then
    git_clone_with_retry https://github.com/google/googletest.git v1.17.0 googletest
fi

pushd googletest >/dev/null
    mkdir -p builder && cd builder
    cmake \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DBUILD_SHARED_LIBS=ON \
        ..
    
    make -j${THREAD_NUM}
    autonomy_make_install
popd >/dev/null

autonomy_ldconfig

ok "Successfully installed googletest v1.17.0"

# Clean up sources only (keep /usr/local install).
rm -rf googletest

