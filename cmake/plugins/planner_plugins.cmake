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

################################ planner plugins #################################
# navfn_planner
add_library(${PROJECT_NAME}_planning_navfn_planner SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/navfn/navfn.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/navfn/navfn_planner.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_planning_navfn_planner)

# theta_star_planner
add_library(${PROJECT_NAME}_planning_theta_star_planner SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/theta_star/theta_star.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/planning/planner/theta_star/theta_star_planner.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_planning_theta_star_planner)
