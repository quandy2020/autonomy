#!/usr/bin/env bash

###############################################################################
# Copyright 2026 The Openbot Authors
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

# Install Taskflow (header-only C++ task-parallel library).
# https://github.com/taskflow/taskflow
#
# Provides:
#   - Headers:  ${PREFIX}/include/taskflow/
#   - CMake:    ${PREFIX}/share/cmake/Taskflow/TaskflowConfig.cmake
#   - pkg-config: ${PREFIX}/lib/pkgconfig/taskflow.pc
#
# Taskflow v4.x requires C++20 (g++ >= 11, clang >= 12).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/installer_base.sh"

VERSION="4.1.0"
TAG="v${VERSION}"
REPO_URL="https://github.com/taskflow/taskflow.git"

autonomy_maybe_reexec_as_root "${BASH_SOURCE[0]}" "$@"

THIRDPARTY_DIR="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
TASKFLOW_SRC="${THIRDPARTY_DIR}/taskflow"
TASKFLOW_HEADER="${INSTALL_PREFIX}/include/taskflow/taskflow.hpp"
TASKFLOW_CMAKE="${INSTALL_PREFIX}/share/cmake/Taskflow/TaskflowConfig.cmake"

if [[ -f "${TASKFLOW_HEADER}" && -f "${TASKFLOW_CMAKE}" ]]; then
    ok "Taskflow ${VERSION} already installed under ${INSTALL_PREFIX}"
    exit 0
fi

info "Building Taskflow ${TAG} (install -> ${INSTALL_PREFIX})"

if [[ ! -d "${TASKFLOW_SRC}/.git" ]]; then
    git clone --depth 1 --branch "${TAG}" "${REPO_URL}" "${TASKFLOW_SRC}"
else
    cd "${TASKFLOW_SRC}"
    git fetch --depth 1 origin "refs/tags/${TAG}:refs/tags/${TAG}" 2>/dev/null || \
        git fetch --depth 1 origin "${TAG}" 2>/dev/null || true
    git checkout "${TAG}"
fi

cd "${TASKFLOW_SRC}"
rm -rf build
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=20 \
    -DTF_BUILD_TESTS=OFF \
    -DTF_BUILD_EXAMPLES=OFF \
    -DTF_BUILD_BENCHMARKS=OFF \
    -DTF_BUILD_PROFILER=OFF \
    -DTF_BUILD_CUDA=OFF \
    ..
cmake --build . --target install -j"$(nproc)"

cd "${TASKFLOW_SRC}" && rm -rf build

if [[ ! -f "${TASKFLOW_HEADER}" ]]; then
    error "Taskflow install failed: missing ${TASKFLOW_HEADER}"
    exit 1
fi

if [[ "${INSTALL_PREFIX}" == "/usr/local" ]] && command -v ldconfig >/dev/null 2>&1; then
    ldconfig || true
fi

ok "Taskflow ${TAG} installed to ${INSTALL_PREFIX}"
info "Use in CMake: find_package(Taskflow CONFIG REQUIRED)"
info "Use in C++:   #include <taskflow/taskflow.hpp>  (requires -std=c++20 -pthread)"

if [[ "${INSTALL_PREFIX}" != "/usr/local" ]]; then
    warning "Non-system prefix: export before building autonomy:"
    warning "  export CMAKE_PREFIX_PATH=${INSTALL_PREFIX}:\${CMAKE_PREFIX_PATH}"
    warning "  export PKG_CONFIG_PATH=${INSTALL_PREFIX}/lib/pkgconfig:\${PKG_CONFIG_PATH}"
fi
