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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

TARGET_ARCH=$(uname -m)
BAZEL_VERSION="8.4.2"

# Download file with optional SHA256 verification
download_with_checksum() {
  local file_name=$1
  local checksum=$2
  local download_url=$3

  if [ -n "$checksum" ]; then
    download_if_not_cached "$file_name" "$checksum" "$download_url"
  else
    info "Downloading ${file_name} without SHA256 verification. Consider adding checksum for security."
    wget -q "${download_url}" -O "${file_name}" || {
      error "Failed to download ${file_name}"
      exit 1
    }
  fi
}

# Install buildifier
install_buildifier() {
  local arch=$1
  local version=$2
  local buildifier_name="buildifier-${version}-linux-${arch}"
  local checksum=""
  local download_url="https://github.com/bazelbuild/buildtools/releases/download/${version}/buildifier"

  download_with_checksum "${buildifier_name}" "${checksum}" "${download_url}"
  cp -f "${buildifier_name}" "${SYSROOT_DIR}/bin/buildifier"
  chmod a+x "${SYSROOT_DIR}/bin/buildifier"
  rm -rf "${buildifier_name}"
}

if [ "$TARGET_ARCH" == "x86_64" ]; then
  # 推荐方式：直接使用 Bazel 官方发布的单一二进制，不依赖 .deb 包和 dpkg
  bazel_binary="bazel-${BAZEL_VERSION}-linux-x86_64"
  checksum=""
  download_url="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${bazel_binary}"

  download_with_checksum "${bazel_binary}" "${checksum}" "${download_url}"
  chmod a+x "${bazel_binary}"
  # 安装到工具链 bin 目录，保持与其它脚本一致
  cp -f "${bazel_binary}" "${SYSROOT_DIR}/bin/bazel"
  chmod a+x "${SYSROOT_DIR}/bin/bazel"
  rm -rf "${bazel_binary}"

  install_buildifier "x86_64" "${BAZEL_VERSION}"
  info "Done installing bazel ${BAZEL_VERSION} (standalone binary) with buildifier ${BAZEL_VERSION}"

elif [ "$TARGET_ARCH" == "aarch64" ]; then
  bazel_binary="bazel-${BAZEL_VERSION}-linux-arm64"
  checksum=""
  download_url="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${bazel_binary}"

  download_with_checksum "${bazel_binary}" "${checksum}" "${download_url}"
  cp -f "${bazel_binary}" "${SYSROOT_DIR}/bin/bazel"
  chmod a+x "${SYSROOT_DIR}/bin/bazel"
  rm -rf "${bazel_binary}"

  install_buildifier "arm64" "${BAZEL_VERSION}"
  info "Done installing bazel ${BAZEL_VERSION} with buildifier ${BAZEL_VERSION}"
else
  error "Target arch ${TARGET_ARCH} not supported yet"
  exit 1
fi

# Clean up cache to reduce layer size
apt-get clean && rm -rf /var/lib/apt/lists/*