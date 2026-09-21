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

# Absolute path before any cd: sudo re-exec with relative $0 breaks after cd.
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_SCRIPT_PATH="${_SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"
cd "${_SCRIPT_DIR}"
. ./installer_base.sh

# Match autonomy install prefix (/usr/local). Do not fall back to ~/.local.
THREAD_NUM=$(nproc)
THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"

if [[ ! -w "${INSTALL_PREFIX}" ]]; then
    if [[ "$(id -u)" -eq 0 ]]; then
        mkdir -p "${INSTALL_PREFIX}"
    elif command -v sudo >/dev/null 2>&1; then
        info "Elevating to install BehaviorTree.CPP under ${INSTALL_PREFIX}..."
        exec sudo -E bash "${_SCRIPT_PATH}" "$@"
    else
        error "BehaviorTree.CPP must be installed under ${INSTALL_PREFIX} (not writable; no sudo)"
        exit 1
    fi
fi

if [[ -f "${INSTALL_PREFIX}/lib/libbehaviortree_cpp.so" ]]; then
    ok "BehaviorTree.CPP already installed under ${INSTALL_PREFIX}, skipping source build"
    exit 0
fi

info "Installing BehaviorTree.CPP 4.7.2 -> ${INSTALL_PREFIX}"

cd "${THIRDPARTY}"
if [[ ! -d BehaviorTree.CPP ]]; then
    git_clone_with_retry https://github.com/BehaviorTree/BehaviorTree.CPP.git 4.7.2 BehaviorTree.CPP
fi

pushd BehaviorTree.CPP >/dev/null
mkdir -p build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    ..

make -j"${THREAD_NUM}"
autonomy_make_install
popd >/dev/null

autonomy_ldconfig
ok "Successfully installed BehaviorTree.CPP -> ${INSTALL_PREFIX}"
