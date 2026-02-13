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

# 公共函数：为 autonomy 示例应用添加可执行文件
function(add_autonomy_demo target_name source_file)
  add_executable(${target_name} ${source_file})
  target_include_directories(${target_name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../..
  )
  target_link_libraries(${target_name} PRIVATE
    ${PROJECT_NAME}
    autolink
    Eigen3::Eigen
  )
endfunction()

# MPPI Controller Application with OpenCV visualization
add_autonomy_demo(autonomy.examples.apps.demo_mppi_controller demo_mppi_controller.cpp)
if(TARGET ${PROJECT_NAME}_control_mppi_controller)
  target_link_libraries(autonomy.examples.apps.demo_mppi_controller PRIVATE 
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_control_mppi_controller
  )
  add_dependencies(autonomy.examples.apps.demo_mppi_controller ${PROJECT_NAME}_control_mppi_controller)
else()
  message(FATAL_ERROR "Plugin library ${PROJECT_NAME}_control_mppi_controller not found")
endif()

# Graceful Controller demo (console simulation)
add_autonomy_demo(autonomy.examples.apps.demo_graceful_controller demo_graceful_controller.cpp)
if(TARGET ${PROJECT_NAME}_control_graceful_controller)
  target_link_libraries(autonomy.examples.apps.demo_graceful_controller PRIVATE 
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_control_graceful_controller
  )
  add_dependencies(autonomy.examples.apps.demo_graceful_controller ${PROJECT_NAME}_control_graceful_controller)
else()
  message(FATAL_ERROR "Plugin library ${PROJECT_NAME}_control_graceful_controller not found")
endif()

# Regulated Pure Pursuit Controller demo (console simulation with moving obstacle)
add_autonomy_demo(autonomy.examples.apps.demo_pure_pursuit_controller demo_pure_pursuit_controller.cpp)
if(TARGET ${PROJECT_NAME}_control_regulated_pure_pursuit_controller)
  target_link_libraries(autonomy.examples.apps.demo_pure_pursuit_controller PRIVATE 
    ${PROJECT_NAME}_control_regulated_pure_pursuit_controller
  )
  add_dependencies(autonomy.examples.apps.demo_pure_pursuit_controller ${PROJECT_NAME}_control_regulated_pure_pursuit_controller)
else()
  message(FATAL_ERROR "Plugin library ${PROJECT_NAME}_control_regulated_pure_pursuit_controller not found")
endif()

# NavFn planner demo 
add_autonomy_demo(autonomy.examples.apps.demo_navfn_planner demo_navfn_planner.cpp)
if(TARGET ${PROJECT_NAME}_planning_navfn_planner)
  target_link_libraries(autonomy.examples.apps.demo_navfn_planner PRIVATE 
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_planning_navfn_planner
  )
  add_dependencies(autonomy.examples.apps.demo_navfn_planner ${PROJECT_NAME}_planning_navfn_planner)
else()
  message(FATAL_ERROR "Plugin library ${PROJECT_NAME}_planning_navfn_planner not found")
endif()

# Navigation-to-pose demo (NavFn + pure pursuit)
add_autonomy_demo(autonomy.examples.apps.demo_navigation_to_pose demo_navigation_to_pose.cpp)
if(TARGET ${PROJECT_NAME}_planning_navfn_planner AND TARGET ${PROJECT_NAME}_control_regulated_pure_pursuit_controller)
  target_link_libraries(autonomy.examples.apps.demo_navigation_to_pose PRIVATE
    ${PROJECT_NAME}_planning_navfn_planner
    ${PROJECT_NAME}_control_regulated_pure_pursuit_controller
  )
  add_dependencies(autonomy.examples.apps.demo_navigation_to_pose
    ${PROJECT_NAME}_planning_navfn_planner
    ${PROJECT_NAME}_control_regulated_pure_pursuit_controller
  )
else()
  message(FATAL_ERROR "Plugin libraries for demo_navigation_to_pose not found")
endif()

# YOLO26 人体识别 demo（依赖 BUILD_ONNXRUNTIME + OpenCV）
if(BUILD_ONNXRUNTIME AND OnnxRuntime_FOUND)
  add_autonomy_demo(autonomy.examples.apps.demo_yolo26_onnixruntime demo_yolo26_onnixruntime.cpp)
  target_link_libraries(autonomy.examples.apps.demo_yolo26_onnixruntime PRIVATE ${OpenCV_LIBS})
endif()

# Install the executables
set(EXAMPLE_APP_TARGETS
    autonomy.examples.apps.demo_mppi_controller
    autonomy.examples.apps.demo_graceful_controller
    autonomy.examples.apps.demo_pure_pursuit_controller
    autonomy.examples.apps.demo_navfn_planner
    autonomy.examples.apps.demo_navigation_to_pose
)
if(TARGET autonomy.examples.apps.demo_yolo26_onnixruntime)
  list(APPEND EXAMPLE_APP_TARGETS autonomy.examples.apps.demo_yolo26_onnixruntime)
endif()
install(TARGETS ${EXAMPLE_APP_TARGETS} RUNTIME DESTINATION bin)
