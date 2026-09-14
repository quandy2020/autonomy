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
        ${CMAKE_CURRENT_SOURCE_DIR}/estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/essential_estimator.h

        # ${CMAKE_CURRENT_SOURCE_DIR}/perspective_n_point_estimator.h
        # ${CMAKE_CURRENT_SOURCE_DIR}/rigid_transformation_estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sample_consensus_estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/fundamental_estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/homography_estimator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/linear_model_estimator.h
)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PLPSLAM_INCLUDE_INSTALL_DIR}/solve/GCRANSAC/estimator)