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

ceres_lib_present() {
    [[ -f "${INSTALL_PREFIX}/lib/libceres.so" ]] \
        || [[ -f /usr/local/lib/libceres.so ]] \
        || [[ -f /usr/lib/x86_64-linux-gnu/libceres.so ]] \
        || [[ -f /usr/lib/aarch64-linux-gnu/libceres.so ]]
}

if ceres_lib_present; then
    ok "Ceres already installed, skipping source build"
    exit 0
fi

cd "${THIRDPARTY}"
if [[ ! -d ceres-solver ]]; then
    git_clone_with_retry https://github.com/ceres-solver/ceres-solver.git 2.2.0 ceres-solver
fi

cd ceres-solver
rm -rf build
mkdir build && cd build

# Disable CUDA unless a working nvcc is available (avoids partial toolkit failures).
USE_CUDA=OFF
cuda_root="${CUDA_HOME:-${CUDA_PATH:-/usr/local/cuda}}"
if [[ -x "${cuda_root}/bin/nvcc" ]] && "${cuda_root}/bin/nvcc" --version >/dev/null 2>&1; then
    USE_CUDA=ON
fi

cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    -DUSE_CUDA="${USE_CUDA}" \
    ..

make -j"${THREAD_NUM}"
make install

ldconfig 2>/dev/null || true

cd ../.. && rm -rf ceres-solver/build

ok "Successfully installed Ceres 2.2.0"
