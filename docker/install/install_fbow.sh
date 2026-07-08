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

# stella-cv FBoW (Fast Bag of Words), used by atlas localization.
FBOW_COMMIT=30f45f1d97314145f81b49617bb37846a861dfe7

autonomy_maybe_reexec_as_root "${BASH_SOURCE[0]}" "$@"

THIRDPARTY_DIR="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
FBOW_SRC="${THIRDPARTY_DIR}/FBoW"

if [[ -f "${INSTALL_PREFIX}/lib/libfbow.so" || -f "${INSTALL_PREFIX}/lib/libfbow.a" ]]; then
    ok "FBoW already installed under ${INSTALL_PREFIX}"
    exit 0
fi

info "Building FBoW in ${FBOW_SRC} (install -> ${INSTALL_PREFIX})"

if [[ ! -d "${FBOW_SRC}/.git" ]]; then
    git clone https://github.com/stella-cv/FBoW.git "${FBOW_SRC}"
fi
cd "${FBOW_SRC}"
git fetch --depth 1 origin "${FBOW_COMMIT}" 2>/dev/null || true
git checkout "${FBOW_COMMIT}"

_apply_fbow_gcc13_patch() {
    local bow_vector_h="${FBOW_SRC}/include/fbow/bow_vector.h"
    if [[ ! -f "${bow_vector_h}" ]]; then
        error "FBoW bow_vector.h not found at ${bow_vector_h}"
        return 1
    fi
    if grep -q '#include <cstdint>' "${bow_vector_h}"; then
        return 0
    fi
    info "Patching FBoW for GCC 13 (add missing <cstdint> include)"
    sed -i '/#include <map>/a #include <cstdint>' "${bow_vector_h}"
}
_apply_fbow_gcc13_patch

rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_TESTS=OFF \
    -DBUILD_UTILS=OFF \
    ..

make -j"$(nproc)"
make install

cd .. && rm -rf build
ok "FBoW installed to ${INSTALL_PREFIX}"

if [[ "${INSTALL_PREFIX}" != "/usr/local" ]]; then
    warning "Non-system prefix: add to your environment before colcon build:"
    warning "  export CMAKE_PREFIX_PATH=${INSTALL_PREFIX}:\${CMAKE_PREFIX_PATH}"
fi
