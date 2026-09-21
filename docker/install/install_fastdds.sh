#!/usr/bin/env bash

###############################################################################
# Copyright 2026 The Openbot Authors (duyongquan). All Rights Reserved.
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

# Install eProsima Fast DDS 3.x (pin matches autolink/cmake/FastDDS.cmake).
# Optional dependency for Autolink cross-host RTPS
# (-DAUTOLINK_ENABLE_FASTDDS=ON + communication_mode.diff_host: RTPS).
#
# Usage:
#   bash docker/install/install_fastdds.sh
#   AUTONOMY_INSTALL_PREFIX=$HOME/.local bash docker/install/install_fastdds.sh

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

THIRDPARTY="$(autonomy_thirdparty_dir)"
INSTALL_PREFIX="$(autonomy_cmake_install_prefix)"
THREAD_NUM=$(nproc)

# Keep in sync with autolink/cmake/FastDDS.cmake (AUTOLINK_FASTDDS_GIT_TAG).
FASTDDS_GIT_TAG="${AUTONOMY_FASTDDS_GIT_TAG:-v3.6.2}"
FASTDDS_SRC_DIR="${THIRDPARTY}/Fast-DDS"

_fastdds_lib_present() {
    [[ -f "${INSTALL_PREFIX}/lib/libfastdds.so" ]] \
        || [[ -f "${INSTALL_PREFIX}/lib64/libfastdds.so" ]] \
        || [[ -f "${INSTALL_PREFIX}/lib/libfastrtps.so" ]] \
        || [[ -f "${INSTALL_PREFIX}/lib64/libfastrtps.so" ]]
}

_fastdds_config_version() {
    local verfile=""
    for verfile in \
        "${INSTALL_PREFIX}/lib/cmake/fastdds/fastddsConfigVersion.cmake" \
        "${INSTALL_PREFIX}/lib64/cmake/fastdds/fastddsConfigVersion.cmake" \
        "${INSTALL_PREFIX}/share/fastdds/cmake/fastddsConfigVersion.cmake" \
        "${INSTALL_PREFIX}/lib/cmake/fastrtps/fastrtpsConfigVersion.cmake" \
        "${INSTALL_PREFIX}/lib64/cmake/fastrtps/fastrtpsConfigVersion.cmake"; do
        if [[ -f "${verfile}" ]]; then
            # shellcheck disable=SC1090
            PACKAGE_VERSION=""
            # CMake version files set PACKAGE_VERSION when included; approximate
            # with grep for shell-only probe.
            grep -E 'set\(PACKAGE_VERSION "[0-9.]+"' "${verfile}" \
                | head -1 \
                | sed -E 's/.*PACKAGE_VERSION "([0-9.]+)".*/\1/'
            return 0
        fi
    done
    return 1
}

if [[ "${AUTONOMY_FORCE_THIRDPARTY:-0}" != "1" ]] && _fastdds_lib_present; then
    _ver="$(_fastdds_config_version || true)"
    if [[ -n "${_ver}" ]]; then
        _major="${_ver%%.*}"
        if [[ "${_major}" -lt 3 ]]; then
            error "Found Fast DDS ${_ver} under ${INSTALL_PREFIX}, but Autolink needs >= 3.0.
Uninstall the 2.x package (or clear ${INSTALL_PREFIX}), then re-run:
  AUTONOMY_FORCE_THIRDPARTY=1 bash docker/install/install_fastdds.sh"
            exit 1
        fi
        ok "Fast DDS ${_ver} already installed under ${INSTALL_PREFIX}, skipping"
        exit 0
    fi
    ok "Fast DDS already installed under ${INSTALL_PREFIX}, skipping"
    exit 0
fi

if ! command -v openssl >/dev/null 2>&1 \
    && [[ ! -f /usr/include/openssl/ssl.h ]] \
    && [[ ! -f /usr/local/include/openssl/ssl.h ]]; then
    error "OpenSSL headers/runtime required to build Fast DDS with SECURITY=ON.
  Debian/Ubuntu: sudo apt install libssl-dev openssl
  Then re-run this script."
    exit 1
fi

info "Installing Fast DDS ${FASTDDS_GIT_TAG} -> ${INSTALL_PREFIX} (SECURITY=ON)"

cd "${THIRDPARTY}"
if [[ ! -d "${FASTDDS_SRC_DIR}/.git" ]]; then
    rm -rf "${FASTDDS_SRC_DIR}"
    git_clone_with_retry \
        https://github.com/eProsima/Fast-DDS.git \
        "${FASTDDS_GIT_TAG}" \
        Fast-DDS
fi

cd "${FASTDDS_SRC_DIR}"
# Ensure we are on the requested tag when the tree already exists.
if ! git describe --tags --exact-match HEAD 2>/dev/null | grep -qx "${FASTDDS_GIT_TAG}"; then
    info "Checking out ${FASTDDS_GIT_TAG}"
    git fetch --depth 1 origin "refs/tags/${FASTDDS_GIT_TAG}:refs/tags/${FASTDDS_GIT_TAG}" \
        || git fetch --tags --depth 1 origin "${FASTDDS_GIT_TAG}" || true
    git checkout -f "${FASTDDS_GIT_TAG}"
fi

submodule_attempt=1
submodule_max="${GIT_SUBMODULE_RETRIES:-5}"
while [[ "${submodule_attempt}" -le "${submodule_max}" ]]; do
    if git submodule update --init --recursive --depth 1; then
        break
    fi
    warning "Fast-DDS submodule update failed (attempt ${submodule_attempt}/${submodule_max}), retrying..."
    submodule_attempt=$((submodule_attempt + 1))
    sleep "${GIT_CLONE_RETRY_SLEEP_SEC:-10}"
done
if [[ "${submodule_attempt}" -gt "${submodule_max}" ]]; then
    error "Fast-DDS submodule update failed after ${submodule_max} attempts"
    exit 1
fi

rm -rf build
mkdir build && cd build

# Options aligned with autolink/cmake/FastDDS.cmake FetchContent path.
cmake_args=(
    -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS=ON
    -DTHIRDPARTY=ON
    -DSECURITY=ON
    -DCOMPILE_EXAMPLES=OFF
    -DCOMPILE_TOOLS=OFF
    -DBUILD_DOCUMENTATION=OFF
)
if [[ -n "${OPENSSL_ROOT_DIR:-}" ]]; then
    cmake_args+=("-DOPENSSL_ROOT_DIR=${OPENSSL_ROOT_DIR}")
fi

cmake "${cmake_args[@]}" ..

make -j"${THREAD_NUM}"
autonomy_make_install
autonomy_ldconfig

if ! _fastdds_lib_present; then
    error "Fast DDS install finished but libfastdds/libfastrtps not found under ${INSTALL_PREFIX}"
    exit 1
fi

ok "Successfully installed Fast DDS ${FASTDDS_GIT_TAG} -> ${INSTALL_PREFIX}"
info "Enable in Autolink: -DAUTOLINK_ENABLE_FASTDDS=ON and set diff_host: RTPS"
