# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

# @file FindIpopt.cmake
# @brief Find-module for COIN-OR Ipopt (interior-point NLP; optional for control).
#
# Ubuntu coinor-libipopt-dev ships IpSmartPtr.hpp that requires HAVE_CSTDDEF
# (or HAVE_STDDEF_H) before include; define HAVE_CSTDDEF on the imported target.
#
# @var Ipopt_FOUND IPOPT_INCLUDE_DIRS IPOPT_LIBRARIES

find_path(IPOPT_INCLUDE_DIR
  NAMES coin/IpIpoptApplication.hpp
  PATHS
    ${CMAKE_INSTALL_PREFIX}/include
    /usr/local/include
    /usr/include
    /usr/include/coin
    ${CMAKE_PREFIX_PATH}/include
)

find_library(IPOPT_LIBRARY
  NAMES ipopt
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib/aarch64-linux-gnu
    ${CMAKE_PREFIX_PATH}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ipopt DEFAULT_MSG IPOPT_LIBRARY IPOPT_INCLUDE_DIR)

if(IPOPT_FOUND)
  set(IPOPT_LIBRARIES ${IPOPT_LIBRARY})
  set(IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIR})
  if(NOT TARGET Ipopt::Ipopt)
    add_library(Ipopt::Ipopt UNKNOWN IMPORTED)
    set_target_properties(Ipopt::Ipopt PROPERTIES
      IMPORTED_LOCATION "${IPOPT_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${IPOPT_INCLUDE_DIR}"
      INTERFACE_COMPILE_DEFINITIONS "HAVE_CSTDDEF")
  endif()
endif()

mark_as_advanced(IPOPT_INCLUDE_DIR IPOPT_LIBRARY)
