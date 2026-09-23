# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# @file Autocmake.cmake
# @brief Entry point included by find_package(autocmake).

include_guard(GLOBAL)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

# Visible to every package in one superbuild. include_guard skips this
# file after the first package, and a normal variable would stay in
# that package's directory scope.
set(_AUTOCMAKE_ROOT "${CMAKE_CURRENT_LIST_DIR}" CACHE INTERNAL
  "Directory containing Autocmake.cmake")

include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeUtils.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeOptions.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeProject.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeTargets.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeProtobuf.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeTest.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeInstall.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeFindPackage.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeCodeCheck.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeDocs.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeSanitizer.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeEnvironment.cmake")
include("${_AUTOCMAKE_ROOT}/cmake/AutocmakeManifest.cmake")
