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

# Must share the same prefix as protobuf 3.19.x (/usr/local). Homedir
# builds (e.g. ~/grpc with protobuf 3.14) cause FatalException at runtime.
THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="${AUTONOMY_INSTALL_PREFIX:-/usr/local}"
THREAD_NUM=$(nproc)

if [[ ! -w "${INSTALL_PREFIX}" ]]; then
    if [[ "$(id -u)" -eq 0 ]]; then
        mkdir -p "${INSTALL_PREFIX}"
    elif command -v sudo >/dev/null 2>&1; then
        info "Elevating to install gRPC under ${INSTALL_PREFIX}..."
        exec sudo -E bash "${_SCRIPT_PATH}" "$@"
    else
        error "gRPC must be installed under ${INSTALL_PREFIX} (not writable; no sudo)"
        exit 1
    fi
fi

# Require matching protobuf 3.19 on the same prefix first.
if [[ ! -x "${INSTALL_PREFIX}/bin/protoc" ]] \
    || ! "${INSTALL_PREFIX}/bin/protoc" --version 2>&1 | grep -q '3\.19\.'; then
    error "Need protobuf 3.19.x under ${INSTALL_PREFIX} first (bash docker/install/install_protobuf.sh)"
    exit 1
fi

# Prefer /usr/local CONFIG over apt / ~/grpc; never treat apt or homedir as OK.
if [[ -f "${INSTALL_PREFIX}/lib/libgrpc++.so" ]] \
    && [[ -x "${INSTALL_PREFIX}/bin/grpc_cpp_plugin" ]]; then
    ok "gRPC already installed under ${INSTALL_PREFIX}, skipping source build"
    exit 0
fi

info "Installing gRPC v1.48.0 -> ${INSTALL_PREFIX} (protobuf from package)"

cd "${THIRDPARTY}"
rm -rf grpc

git_clone_with_retry https://github.com/grpc/grpc.git v1.48.0 grpc
cd grpc

submodule_attempt=1
submodule_max="${GIT_SUBMODULE_RETRIES:-5}"
while [[ "${submodule_attempt}" -le "${submodule_max}" ]]; do
    if git submodule update --init --recursive --depth 1; then
        break
    fi
    warning "grpc submodule update failed (attempt ${submodule_attempt}/${submodule_max}), retrying..."
    submodule_attempt=$((submodule_attempt + 1))
    sleep "${GIT_CLONE_RETRY_SLEEP_SEC:-10}"
done
if [[ "${submodule_attempt}" -gt "${submodule_max}" ]]; then
    error "grpc submodule update failed after ${submodule_max} attempts"
    exit 1
fi

mkdir -p build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DgRPC_INSTALL=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DgRPC_BUILD_TESTS=OFF \
    -DgRPC_PROTOBUF_PROVIDER=package \
    -DgRPC_ZLIB_PROVIDER=package \
    -DgRPC_CARES_PROVIDER=module \
    -DgRPC_SSL_PROVIDER=package \
    ..

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

cd ../.. && rm -rf grpc/build

ok "Successfully installed gRPC v1.48.0 -> ${INSTALL_PREFIX}"
