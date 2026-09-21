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

# Autonomy binaries under /usr/local need libglog.so.1 on the same prefix.
# Do NOT fall back to ~/.local — that leaves /usr/local/bin/autolink broken.
THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
THREAD_NUM=$(nproc)

if [[ ! -w "${INSTALL_PREFIX}" ]]; then
    if [[ "$(id -u)" -eq 0 ]]; then
        mkdir -p "${INSTALL_PREFIX}"
    elif command -v sudo >/dev/null 2>&1; then
        info "Elevating to install glog under ${INSTALL_PREFIX}..."
        exec sudo -E bash "${_SCRIPT_PATH}" "$@"
    else
        error "glog must be installed under ${INSTALL_PREFIX} (not writable; no sudo)"
        exit 1
    fi
fi

# Apt libgoogle-glog (.so.0) must not count. Need glog 0.6 soname .so.1.
if [[ -f "${INSTALL_PREFIX}/lib/libglog.so.1" || -f "${INSTALL_PREFIX}/lib/libglog.so" ]]; then
    ok "glog already installed under ${INSTALL_PREFIX}, skipping source build"
    exit 0
fi

info "Installing glog v0.6.0 -> ${INSTALL_PREFIX}"

cd "${THIRDPARTY}"
if [[ ! -d glog ]]; then
    git_clone_with_retry https://github.com/google/glog.git v0.6.0 glog
fi
cd glog
git submodule init
git submodule update || true

rm -rf builder
mkdir -p builder && cd builder
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    ..

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

ok "Successfully installed glog v0.6.0 -> ${INSTALL_PREFIX}"
