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

# ============================================================================
# MPPI Controller Critics 批量导出示例
# ============================================================================
# 此文件展示了如何使用 autolink_export_plugin 批量导出所有 critics
# 可以将其内容集成到 controller_plugins.cmake 中

# 包含 autolink_export_plugin 函数
include("${PROJECT_SOURCE_DIR}/../autolink/cmake/autolink_export_plugin.cmake")

# 定义所有 critics
set(MPPI_CRITICS
  constraint_critic
  cost_critic
  goal_angle_critic
  goal_critic
  obstacles_critic
  path_align_critic
  path_angle_critic
  path_follow_critic
  prefer_forward_critic
  twirling_critic
  velocity_deadband_critic
)

# 批量创建和导出 critics
foreach(critic ${MPPI_CRITICS})
  # 创建库目标名称
  set(lib_name "${PROJECT_NAME}_control_mppi_controller_${critic}")
  
  # 创建插件库
  add_library(${lib_name} SHARED 
    "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/${critic}.cpp"
  )
  
  # 设置包含目录
  target_include_directories(${lib_name} PUBLIC
    "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics"
    "${PROJECT_SOURCE_DIR}"
  )
  
  # 链接依赖库
  target_link_libraries(${lib_name} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  # 导出插件（使用统一的 XML 文件，因为所有 critics 在同一基类下）
  # 注意：如果每个 critic 需要单独的 XML，需要创建对应的 XML 文件
  autolink_export_plugin(
    LIBRARY ${lib_name}
    DESCRIPTION_FILE autonomy/control/controller/mppi_controller/critics/mppi_critics.xml
    INDEX_NAME mppi_${critic}
    INSTALL_SUBDIR control/controller/mppi_controller/critics
  )
  
  # 添加到插件列表
  list(APPEND plugin_libs ${lib_name})
endforeach()

# ============================================================================
# 方案 2: 合并所有 critics 到一个库（可选）
# ============================================================================
# 如果希望将所有 critics 合并到一个库中，可以使用以下代码：

# set(MPPI_CRITICS_SOURCES "")
# foreach(critic ${MPPI_CRITICS})
#   list(APPEND MPPI_CRITICS_SOURCES
#     "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/${critic}.cpp"
#   )
# endforeach()
# 
# add_library(${PROJECT_NAME}_control_mppi_controller_critics SHARED 
#   ${MPPI_CRITICS_SOURCES}
# )
# 
# target_include_directories(${PROJECT_NAME}_control_mppi_controller_critics PUBLIC
#   "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics"
#   "${PROJECT_SOURCE_DIR}"
# )
# 
# target_link_libraries(${PROJECT_NAME}_control_mppi_controller_critics PUBLIC
#   ${PROJECT_NAME}
#   autolink
# )
# 
# autolink_export_plugin(
#   LIBRARY ${PROJECT_NAME}_control_mppi_controller_critics
#   DESCRIPTION_FILE autonomy/control/controller/mppi_controller/critics/mppi_critics.xml
#   INDEX_NAME mppi_critics
#   INSTALL_SUBDIR control/controller/mppi_controller/critics
# )
# 
# list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller_critics)
