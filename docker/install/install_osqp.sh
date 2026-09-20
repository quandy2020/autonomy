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

if [[ -f "${INSTALL_PREFIX}/lib/libosqp.so" ]]; then
    ok "OSQP already installed under ${INSTALL_PREFIX}, skipping"
    exit 0
fi

info "Installing OSQP -> ${INSTALL_PREFIX}"

cd "${THIRDPARTY}"
if [[ ! -d osqp/.git ]]; then
    rm -rf osqp
    git_clone_with_retry https://github.com/osqp/osqp.git release-0.6.3 osqp
fi
cd osqp
git submodule update --init --recursive

rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    ..

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

ok "Successfully installed OSQP -> ${INSTALL_PREFIX}"
