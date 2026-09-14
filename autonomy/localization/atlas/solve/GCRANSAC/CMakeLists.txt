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
        ${CMAKE_CURRENT_SOURCE_DIR}/model.h
        ${CMAKE_CURRENT_SOURCE_DIR}/flann_neighborhood_graph.h
        ${CMAKE_CURRENT_SOURCE_DIR}/GCRANSAC.h
        ${CMAKE_CURRENT_SOURCE_DIR}/grid_neighborhood_graph.h
        ${CMAKE_CURRENT_SOURCE_DIR}/neighborhood_graph.h
        ${CMAKE_CURRENT_SOURCE_DIR}/preemption_empty.h
        ${CMAKE_CURRENT_SOURCE_DIR}/preemption_sprt.h
        ${CMAKE_CURRENT_SOURCE_DIR}/progressive_napsac_sampler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/prosac_sampler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sampler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/scoring_function.h
        ${CMAKE_CURRENT_SOURCE_DIR}/settings.h
        ${CMAKE_CURRENT_SOURCE_DIR}/single_point_sampler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/statistics.h
        ${CMAKE_CURRENT_SOURCE_DIR}/types.h
        ${CMAKE_CURRENT_SOURCE_DIR}/uniform_random_generator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/uniform_sampler.h
)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PLPSLAM_INCLUDE_INSTALL_DIR}/solve/GCRANSAC)

add_subdirectory(pearl) # Graph-cut RANSAC
add_subdirectory(solver) # Graph-cut RANSAC
add_subdirectory(estimator) # Graph-cut RANSAC
