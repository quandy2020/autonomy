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

# Absolute path before any cd: sudo re-exec with relative $0 breaks after
# `cd "$(dirname ...)"` (looks for docker/install/docker/install/...).
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_SCRIPT_PATH="${_SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
cd "${_SCRIPT_DIR}"
. ./installer_base.sh

PROTOBUF_VERSION="v3.19.4"
PROTOBUF_REPO="https://github.com/protocolbuffers/protobuf.git"
THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
THREAD_NUM=$(nproc)

if [[ ! -w "${INSTALL_PREFIX}" ]]; then
    if [[ "$(id -u)" -eq 0 ]]; then
        mkdir -p "${INSTALL_PREFIX}"
    elif command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
        exec sudo -E bash "${_SCRIPT_PATH}" "$@"
    else
        error "protobuf 3.19.x must be installed under ${INSTALL_PREFIX} (not writable)"
        exit 1
    fi
fi

protobuf319_present() {
    local protoc="${INSTALL_PREFIX}/bin/protoc"
    [[ -x "${protoc}" ]] || return 1
    "${protoc}" --version 2>&1 | grep -q '3\.19\.'
}

if protobuf319_present; then
    ok "protobuf 3.19.x already installed at ${INSTALL_PREFIX}, skipping source build"
    exit 0
fi

info "Installing protobuf ${PROTOBUF_VERSION} -> ${INSTALL_PREFIX}"

cd "${THIRDPARTY}"
if [[ -d protobuf/.git ]]; then
    autonomy_git_safe_directory "$(pwd)/protobuf"
    cd protobuf
    # Root-owned trees from prior sudo installs: prefer wipe+reclone as current user.
    if ! git status >/dev/null 2>&1; then
        cd ..
        rm -rf protobuf
        git_clone_with_retry "${PROTOBUF_REPO}" "${PROTOBUF_VERSION}" protobuf
        cd protobuf
    else
        git fetch --tags origin
        git checkout "${PROTOBUF_VERSION}"
    fi
else
    rm -rf protobuf
    git_clone_with_retry "${PROTOBUF_REPO}" "${PROTOBUF_VERSION}" protobuf
    cd protobuf
fi

git submodule update --init --recursive || true

if [[ ! -f cmake/CMakeLists.txt ]]; then
    error "expected cmake/CMakeLists.txt at protobuf ${PROTOBUF_VERSION}"
    exit 1
fi

rm -rf build
mkdir -p build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -Dprotobuf_BUILD_TESTS=OFF \
    ../cmake

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

cd ../.. && rm -rf protobuf/build

ok "Successfully installed protobuf ${PROTOBUF_VERSION} to ${INSTALL_PREFIX}"
