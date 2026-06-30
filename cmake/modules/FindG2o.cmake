# Copyright 2025 The Openbot Authors (duyongquan)
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

# FindG2o.cmake
#
# Locates g2o graph optimization library required by atlas.

if(G2o_FOUND)
  return()
endif()

find_package(
  g2o
  CONFIG
  QUIET
  PATHS
    ${CMAKE_PREFIX_PATH}
    $ENV{g2o_DIR}
    $ENV{G2O_DIR}
    /usr/local
    /usr
  PATH_SUFFIXES
    lib/cmake/g2o
    share/g2o/cmake
)

set(G2O_REQUIRED_TARGETS
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
)

if(g2o_FOUND)
  foreach(_g2o_target IN LISTS G2O_REQUIRED_TARGETS)
    if(NOT TARGET ${_g2o_target})
      message(FATAL_ERROR "Found g2o config but missing target: ${_g2o_target}")
    endif()
  endforeach()
  set(G2O_LIBRARIES ${G2O_REQUIRED_TARGETS})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  G2o
  REQUIRED_VARS g2o_FOUND
  FAIL_MESSAGE
    "Could not find g2o. Run docker/install/install_g2o.sh or set g2o_DIR."
)

if(G2o_FOUND)
  set(G2O_FOUND TRUE)
endif()

mark_as_advanced(g2o_DIR)
