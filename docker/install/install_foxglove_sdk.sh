#!/usr/bin/env bash

###############################################################################
# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
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

set -euo pipefail

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. "${CURR_DIR}/installer_base.sh"

autonomy_maybe_reexec_as_root "$0" "$@"

FOXGLOVE_SDK_REF="${FOXGLOVE_SDK_REF:-main}"
FOXGLOVE_REMOTE_ACCESS="${FOXGLOVE_REMOTE_ACCESS:-OFF}"
FOXGLOVE_SDK_BUILD_EXAMPLES="${FOXGLOVE_SDK_BUILD_EXAMPLES:-OFF}"
FOXGLOVE_RUST_TOOLCHAIN="${FOXGLOVE_RUST_TOOLCHAIN:-1.85.0}"
FOXGLOVE_SDK_REPO_URL="${FOXGLOVE_SDK_REPO_URL:-https://github.com/foxglove/foxglove-sdk.git}"
FOXGLOVE_SDK_KEEP_SOURCE="${FOXGLOVE_SDK_KEEP_SOURCE:-0}"

THIRDPARTY_DIR="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
FOXGLOVE_SDK_SRC_DIR="${THIRDPARTY_DIR}/foxglove-sdk"
FOXGLOVE_SDK_BUILD_DIR="${FOXGLOVE_SDK_SRC_DIR}/build-autonomy"
BUILD_JOBS="${BUILD_JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 8)}"

version_ge() {
    local lhs="$1"
    local rhs="$2"
    [[ "$(printf '%s\n%s\n' "$rhs" "$lhs" | sort -V | tail -n1)" == "$lhs" ]]
}

ensure_cmake() {
    if command -v cmake >/dev/null 2>&1; then
        local cmake_version
        cmake_version="$(cmake --version | awk 'NR==1 {print $3}')"
        if version_ge "${cmake_version}" "3.22.0"; then
            info "Found cmake ${cmake_version}"
            return 0
        fi
        warning "cmake ${cmake_version} is too old, upgrading to >= 3.22"
    else
        warning "cmake not found, installing cmake >= 3.22"
    fi

    bash "${CURR_DIR}/install_cmake.sh"
}

ensure_rust_toolchain() {
    if ! command -v curl >/dev/null 2>&1; then
        apt_get_update_and_install curl
    fi

    if ! command -v rustup >/dev/null 2>&1; then
        info "Installing rustup..."
        curl https://sh.rustup.rs -sSf | sh -s -- -y --profile minimal
    fi

    # shellcheck disable=SC1091
    . "${HOME}/.cargo/env"

    info "Installing Rust toolchain ${FOXGLOVE_RUST_TOOLCHAIN}..."
    rustup toolchain install "${FOXGLOVE_RUST_TOOLCHAIN}" --profile minimal
}

install_build_dependencies() {
    info "Installing build dependencies for foxglove-sdk..."
    apt_get_update_and_install \
        build-essential \
        ca-certificates \
        clang \
        curl \
        git \
        pkg-config \
        perl
}

clone_repo() {
    if [[ -d "${FOXGLOVE_SDK_SRC_DIR}" ]]; then
        info "Removing existing source tree ${FOXGLOVE_SDK_SRC_DIR}"
        rm -rf "${FOXGLOVE_SDK_SRC_DIR}"
    fi

    info "Cloning foxglove-sdk (${FOXGLOVE_SDK_REF})..."
    git_clone_with_retry "${FOXGLOVE_SDK_REPO_URL}" "${FOXGLOVE_SDK_REF}" "${FOXGLOVE_SDK_SRC_DIR}"
}

build_and_install() {
    # shellcheck disable=SC1091
    . "${HOME}/.cargo/env"

    info "Configuring foxglove-sdk for install prefix ${INSTALL_PREFIX}..."
    cmake -S "${FOXGLOVE_SDK_SRC_DIR}/cpp" -B "${FOXGLOVE_SDK_BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
        -DFOXGLOVE_REMOTE_ACCESS="${FOXGLOVE_REMOTE_ACCESS}" \
        -DFOXGLOVE_BUILD_EXAMPLES="${FOXGLOVE_SDK_BUILD_EXAMPLES}" \
        -DUSE_PACKAGE_MANAGER_DEPENDENCIES=ON

    info "Building foxglove-sdk..."
    cmake --build "${FOXGLOVE_SDK_BUILD_DIR}" --parallel "${BUILD_JOBS}"

    info "Installing foxglove-sdk..."
    cmake --install "${FOXGLOVE_SDK_BUILD_DIR}"
}

verify_install() {
    local config_path="${INSTALL_PREFIX}/lib/cmake/foxglove-sdk/foxglove-sdkConfig.cmake"
    if [[ ! -f "${config_path}" ]]; then
        error "foxglove-sdkConfig.cmake not found at ${config_path}"
        exit 1
    fi

    ok "foxglove-sdk installed successfully."
    info "CMake package: ${config_path}"
    info "If your project does not search ${INSTALL_PREFIX}, set:"
    info "  export CMAKE_PREFIX_PATH=${INSTALL_PREFIX}:\${CMAKE_PREFIX_PATH:-}"
}

cleanup() {
    if [[ "${FOXGLOVE_SDK_KEEP_SOURCE}" == "1" ]]; then
        info "Keeping source tree at ${FOXGLOVE_SDK_SRC_DIR}"
        return 0
    fi

    info "Cleaning source tree..."
    rm -rf "${FOXGLOVE_SDK_SRC_DIR}"
}

main() {
    install_build_dependencies
    ensure_cmake
    ensure_rust_toolchain
    clone_repo
    build_and_install
    verify_install
    cleanup
}

main "$@"
