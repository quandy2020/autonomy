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

PROTOBUF_VERSION="v3.19.4"
PROTOBUF_REPO="https://github.com/protocolbuffers/protobuf.git"

mkdir -p /thirdparty
cd /thirdparty

if [[ -d protobuf/.git ]]; then
  cd protobuf
  git fetch --tags origin
  git checkout "${PROTOBUF_VERSION}"
else
  rm -rf protobuf
  git clone --single-branch --branch "${PROTOBUF_VERSION}" "${PROTOBUF_REPO}" protobuf
  cd protobuf
fi

git submodule update --init --recursive

mkdir -p build && cd build
cmake \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  ..

make -j"$(nproc)"
sudo make install

# Clean up build dir only; keep source for potential rebuilds.
cd .. && rm -rf build
