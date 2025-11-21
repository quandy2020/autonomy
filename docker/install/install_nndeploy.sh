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

# update
sudo ldconfig

cd /thirdparty
git clone https://github.com/nndeploy/nndeploy.git
cd grpc && git submodule init && git submodule update
mkdir build && cd build
cmake  -DCMAKE_BUILD_TYPE=Release   \
  -DCMAKE_CXX_STANDARD=17           \
  -DgRPC_INSTALL=ON                 \
  -DBUILD_SHARED_LIBS=ON            \
  -DgRPC_BUILD_TESTS=OFF            \
  ..  

make -j8
make install

# Clean up.
cd .. && rm -rf build
