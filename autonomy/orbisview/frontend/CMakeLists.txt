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

# OrbisView Vite frontend → dist/ (optional; needs Node/npm ≥18)
# Install toolchain: ../script/install_npm.sh

option(ORBISVIEW_BUILD_FRONTEND
  "Build OrbisView Vite frontend with npm during CMake build" ON)

if(NOT ORBISVIEW_BUILD_FRONTEND)
  return()
endif()

find_program(ORBISVIEW_NPM NAMES npm)
if(NOT ORBISVIEW_NPM)
  message(WARNING
    "ORBISVIEW_BUILD_FRONTEND=ON but npm not found; "
    "skipping OrbisView frontend (install Node.js ≥18 or set "
    "ORBISVIEW_BUILD_FRONTEND=OFF)")
  return()
endif()

file(GLOB_RECURSE _ORBISVIEW_FRONTEND_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*"
  "${CMAKE_CURRENT_SOURCE_DIR}/assets/*")
set(_ORBISVIEW_FRONTEND_STAMP
  "${CMAKE_CURRENT_BINARY_DIR}/orbisview_frontend.stamp")
set(_ORBISVIEW_FRONTEND_DIST "${CMAKE_CURRENT_SOURCE_DIR}/dist")

add_custom_command(
  OUTPUT "${_ORBISVIEW_FRONTEND_STAMP}"
  COMMAND "${ORBISVIEW_NPM}" ci
  COMMAND "${ORBISVIEW_NPM}" run build
  COMMAND "${CMAKE_COMMAND}" -E touch "${_ORBISVIEW_FRONTEND_STAMP}"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/package.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/package-lock.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/vite.config.ts"
    "${CMAKE_CURRENT_SOURCE_DIR}/tsconfig.json"
    "${CMAKE_CURRENT_SOURCE_DIR}/index.html"
    ${_ORBISVIEW_FRONTEND_SRCS}
  COMMENT "Building OrbisView frontend (npm ci && npm run build)"
  VERBATIM)

add_custom_target(orbisview_frontend ALL
  DEPENDS "${_ORBISVIEW_FRONTEND_STAMP}")

if(TARGET autonomy.orbisview)
  add_dependencies(autonomy.orbisview orbisview_frontend)
endif()

install(
  DIRECTORY "${_ORBISVIEW_FRONTEND_DIST}/"
  DESTINATION share/autonomy/orbisview/www)

message(STATUS "orbisview: frontend build enabled (${ORBISVIEW_NPM})")

unset(_ORBISVIEW_FRONTEND_SRCS)
unset(_ORBISVIEW_FRONTEND_STAMP)
unset(_ORBISVIEW_FRONTEND_DIST)
