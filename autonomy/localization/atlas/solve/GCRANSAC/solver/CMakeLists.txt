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

        # ${CMAKE_CURRENT_SOURCE_DIR}/solver_dls_pnp.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_engine.h

        # ${CMAKE_CURRENT_SOURCE_DIR}/solver_epnp_lm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_essential_matrix_five_point_stewenius.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_fundamental_matrix_eight_point.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_fundamental_matrix_plane_and_parallax.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_fundamental_matrix_seven_point.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_homography_four_point.h
        ${CMAKE_CURRENT_SOURCE_DIR}/solver_linear_model.h

        # ${CMAKE_CURRENT_SOURCE_DIR}/solver_p3p.h
        # ${CMAKE_CURRENT_SOURCE_DIR}/solver_rigid_transformation_svd.h
)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${PLPSLAM_INCLUDE_INSTALL_DIR}/solve/GCRANSAC/solver)