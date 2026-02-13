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

# ONNX Runtime 预编译包：https://github.com/microsoft/onnxruntime/releases
VERSION="1.18.1"
TARGET_ARCH="$(uname -m)"

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    PKG_SUFFIX="linux-x64"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    PKG_SUFFIX="linux-aarch64"
else
    error "ONNX Runtime prebuilt not available for ${TARGET_ARCH}. Exiting."
    exit 1
fi

PKG_NAME="onnxruntime-${PKG_SUFFIX}-${VERSION}.tgz"
DOWNLOAD_LINK="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${PKG_NAME}"

# 可选：若提供校验和且使用本地缓存，可取消下行注释并填写 CHECKSUM
# CHECKSUM=""
# download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"
info "Downloading ONNX Runtime ${VERSION} (${PKG_SUFFIX})..."
wget -q --show-progress "${DOWNLOAD_LINK}" -O "${PKG_NAME}" || {
    error "Failed to download ${PKG_NAME}"
    exit 1
}

info "Extracting to /usr/local..."
tar xzf "${PKG_NAME}" -C /tmp
# GitHub 预编译包解压后目录名：onnxruntime-<linux-x64|linux-aarch64>-<version>
ONNX_DIR="/tmp/onnxruntime-${PKG_SUFFIX}-${VERSION}"
if [[ ! -d "${ONNX_DIR}" ]]; then
    ONNX_DIR=$(find /tmp -maxdepth 1 -type d -name "onnxruntime-*" | head -1)
fi

if [[ -d "${ONNX_DIR}" ]]; then
    mkdir -p /usr/local/include /usr/local/lib
    cp -r "${ONNX_DIR}/include"/* /usr/local/include/ 2>/dev/null || true
    cp -r "${ONNX_DIR}/lib"/* /usr/local/lib/ 2>/dev/null || true
    rm -rf "${ONNX_DIR}"
else
    error "Unexpected archive layout. Expected dir onnxruntime-*-${VERSION} under /tmp."
    exit 1
fi

rm -f "${PKG_NAME}"
ldconfig

ok "Successfully installed ONNX Runtime ${VERSION} (${PKG_SUFFIX})."
