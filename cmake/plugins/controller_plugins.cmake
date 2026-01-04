# Copyright 2024 The Openbot Authors (duyongquan)
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


################################ controller plugins #################################

# # dwb_controller
# add_library(${PROJECT_NAME}_controller_dwb_controller SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/controller/dwb_controller/dwb_local_controller.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_controller_dwb_controller)

# graceful_controller 
add_library(${PROJECT_NAME}_control_graceful_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/graceful_controller/graceful_controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_control_graceful_controller)

# # mppi_controller
# add_library(${PROJECT_NAME}_control_mppi_controller SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/controller.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller)

# # pure_pursuit_controller
# add_library(${PROJECT_NAME}_control_pure_pursuit_controller SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/controller/pure_pursuit_controller/regulated_pure_pursuit_controller.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_control_pure_pursuit_controller)

# # teb_controller
# add_library(${PROJECT_NAME}_control_teb_controller SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/controller/teb_controller/teb_controller.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_control_teb_controller)

################################ checker plugins ################################
# add_library(${PROJECT_NAME}_control_simple_goal_checker SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/checker/simple_goal_checker.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_control_simple_goal_checker)

# add_library(${PROJECT_NAME}_control_position_goal_checker SHARED 
#   "${PROJECT_SOURCE_DIR}/autonomy/control/checker/position_goal_checker.cpp"
# )
# list(APPEND plugin_libs ${PROJECT_NAME}_control_position_goal_checker)