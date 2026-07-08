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

grpc_lib_present() {
    [[ -f "${INSTALL_PREFIX}/lib/libgrpc++.so" ]] \
        || [[ -f /usr/lib/x86_64-linux-gnu/libgrpc++.so ]] \
        || [[ -f /usr/lib/aarch64-linux-gnu/libgrpc++.so ]] \
        || compgen -G "/usr/lib/*/libgrpc++*.so*" >/dev/null
}

if grpc_lib_present && command -v grpc_cpp_plugin >/dev/null 2>&1; then
    ok "gRPC already installed (system packages), skipping source build"
    exit 0
fi

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
if [[ "$(id -u)" -eq 0 ]]; then
    make install
else
    sudo make install
fi

ldconfig 2>/dev/null || true

cd ../.. && rm -rf grpc/build

ok "Successfully installed gRPC v1.48.0"
