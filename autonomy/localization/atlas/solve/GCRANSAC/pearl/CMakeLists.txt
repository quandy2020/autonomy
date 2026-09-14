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

# Add sources
target_sources(${PROJECT_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/block.h
        ${CMAKE_CURRENT_SOURCE_DIR}/graph.h
        ${CMAKE_CURRENT_SOURCE_DIR}/energy.h
        ${CMAKE_CURRENT_SOURCE_DIR}/GCoptimization.h
        ${CMAKE_CURRENT_SOURCE_DIR}/LinkedBlockList.h

        ${CMAKE_CURRENT_SOURCE_DIR}/graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GCoptimization.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/LinkedBlockList.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/maxflow.cpp
)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PLPSLAM_INCLUDE_INSTALL_DIR}/solve/GCRANSAC/pearl)
