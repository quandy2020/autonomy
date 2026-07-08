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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/installer_base.sh"

# g2o graph optimization library, used by atlas localization.
G2O_COMMIT=20230223_git

autonomy_maybe_reexec_as_root "${BASH_SOURCE[0]}" "$@"

THIRDPARTY_DIR="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
G2O_SRC="${THIRDPARTY_DIR}/g2o"

if [[ -f "${INSTALL_PREFIX}/lib/cmake/g2o/g2oConfig.cmake" ]]; then
    ok "g2o already installed under ${INSTALL_PREFIX}"
    exit 0
fi

info "Building g2o in ${G2O_SRC} (install -> ${INSTALL_PREFIX})"

cd "${THIRDPARTY_DIR}"
if [[ ! -d g2o/.git ]]; then
    git_clone_with_retry https://github.com/RainerKuemmerle/g2o.git "${G2O_COMMIT}" g2o
fi
cd g2o
git fetch --depth 1 origin "${G2O_COMMIT}" 2>/dev/null || true
git checkout "${G2O_COMMIT}"

rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=OFF \
    -DG2O_BUILD_APPS=OFF \
    -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_BUILD_LINKED_APPS=OFF \
    ..

make -j"$(nproc)"
make install

cd .. && rm -rf build
ok "g2o installed to ${INSTALL_PREFIX}"

if [[ "${INSTALL_PREFIX}" != "/usr/local" ]]; then
    warning "Non-system prefix: add to your environment before colcon build:"
    warning "  export CMAKE_PREFIX_PATH=${INSTALL_PREFIX}:\${CMAKE_PREFIX_PATH}"
fi
