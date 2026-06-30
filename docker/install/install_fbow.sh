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

# stella-cv FBoW (Fast Bag of Words), used by atlas localization.
FBOW_COMMIT=30f45f1d97314145f81b49617bb37846a861dfe7

cd /thirdparty
git clone https://github.com/stella-cv/FBoW.git
cd FBoW
git checkout "${FBOW_COMMIT}"
mkdir build && cd build
cmake \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_TESTS=OFF \
    -DBUILD_UTILS=OFF \
    ..

make -j6
make install

# Clean up.
cd .. && rm -rf build
