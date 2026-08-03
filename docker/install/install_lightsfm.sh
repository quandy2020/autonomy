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

# Install the header-only lightsfm library system-wide with a CMake package
# config so find_package(lightsfm) / lightsfm::lightsfm work.
# Prefer building the workspace package under
# src/autonomy_ros/autonomy_tools/autonomy_hunav/lightsfm when using colcon.

set -euo pipefail

LIGHTSFM_REPO="https://github.com/robotics-upo/lightsfm.git"
INSTALL_PREFIX="${LIGHTSFM_INSTALL_PREFIX:-/usr/local}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

git clone --depth 1 "${LIGHTSFM_REPO}" "${tmpdir}/lightsfm"

mkdir -p "${INSTALL_PREFIX}/include/lightsfm"
cp "${tmpdir}/lightsfm/include/"*.hpp "${INSTALL_PREFIX}/include/lightsfm/"

cmake_dir="${INSTALL_PREFIX}/lib/cmake/lightsfm"
mkdir -p "${cmake_dir}"

cat > "${cmake_dir}/lightsfmTargets.cmake" <<EOF
if(NOT TARGET lightsfm::lightsfm)
  add_library(lightsfm::lightsfm INTERFACE IMPORTED)
  set_target_properties(lightsfm::lightsfm PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${INSTALL_PREFIX}/include"
  )
endif()
EOF

cat > "${cmake_dir}/lightsfmConfig.cmake" <<'EOF'
include("${CMAKE_CURRENT_LIST_DIR}/lightsfmTargets.cmake")
EOF

cat > "${cmake_dir}/lightsfmConfigVersion.cmake" <<'EOF'
set(PACKAGE_VERSION "1.0.0")
if(PACKAGE_FIND_VERSION VERSION_LESS PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
endif()
if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
  set(PACKAGE_VERSION_EXACT TRUE)
endif()
EOF

echo "[OK] lightsfm headers installed to ${INSTALL_PREFIX}/include/lightsfm"
echo "[OK] CMake package config installed to ${cmake_dir}"
