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

set -euo pipefail

LIGHTSFM_REPO="https://github.com/robotics-upo/lightsfm.git"
INSTALL_PREFIX="${LIGHTSFM_INSTALL_PREFIX:-/usr/local}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

git clone --depth 1 "${LIGHTSFM_REPO}" "${tmpdir}/lightsfm"
make -C "${tmpdir}/lightsfm" install

echo "[OK] lightsfm headers installed to ${INSTALL_PREFIX}/include/lightsfm"
