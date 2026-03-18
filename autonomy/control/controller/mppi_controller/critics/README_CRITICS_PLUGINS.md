# MPPI Controller Critics 插件导出指南

## 概述

MPPI Controller 的所有 critics 都可以使用 `autolink_export_plugin` CMake 函数模块来导出。目前有 11 个 critic 插件，每个都继承自 `CriticFunction` 基类。

## Critics 列表

1. **ConstraintCritic** - 约束评估器
2. **CostCritic** - 成本评估器
3. **GoalAngleCritic** - 目标角度评估器
4. **GoalCritic** - 目标距离评估器
5. **ObstaclesCritic** - 障碍物评估器
6. **PathAlignCritic** - 路径对齐评估器
7. **PathAngleCritic** - 路径角度评估器
8. **PathFollowCritic** - 路径跟随评估器
9. **PreferForwardCritic** - 偏好前进评估器
10. **TwirlingCritic** - 旋转惩罚评估器
11. **VelocityDeadbandCritic** - 速度死区评估器

## 当前状态

- ✅ 所有 critics 已使用 `CLASS_LOADER_REGISTER_CLASS` 注册
- ✅ 所有 critics 已在 `controller_plugins.cmake` 中构建为单独的插件库
- ❌ 缺少 XML 描述文件
- ❌ 未使用 `autolink_export_plugin` 函数

## 使用方案

### 方案 1: 为每个 Critic 创建单独的 XML 文件（推荐用于单独库）

如果每个 critic 都是单独的库（当前情况），可以为每个创建单独的 XML 文件：

```cmake
# 在 controller_plugins.cmake 中
include("${PROJECT_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# Constraint Critic
add_library(${PROJECT_NAME}_control_mppi_controller_constraint_critic SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/constraint_critic.cpp"
)
autolink_export_plugin(
  LIBRARY ${PROJECT_NAME}_control_mppi_controller_constraint_critic
  DESCRIPTION_FILE autonomy/control/controller/mppi_controller/critics/constraint_critic.xml
  INDEX_NAME mppi_constraint_critic
  INSTALL_SUBDIR control/controller/mppi_controller/critics
)
list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller_constraint_critic)

# 对其他 critics 重复相同模式...
```

### 方案 2: 合并所有 Critics 到一个库（推荐用于统一管理）

如果将所有 critics 合并到一个库中，可以使用单个 XML 文件：

```cmake
# 在 controller_plugins.cmake 中
include("${PROJECT_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# 合并所有 critics 到一个库
add_library(${PROJECT_NAME}_control_mppi_controller_critics SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/constraint_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/cost_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/goal_angle_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/goal_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/obstacles_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/path_align_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/path_angle_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/path_follow_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/prefer_forward_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/twirling_critic.cpp"
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/velocity_deadband_critic.cpp"
)

target_link_libraries(${PROJECT_NAME}_control_mppi_controller_critics PUBLIC
  ${PROJECT_NAME}
  autolink
)

autolink_export_plugin(
  LIBRARY ${PROJECT_NAME}_control_mppi_controller_critics
  DESCRIPTION_FILE autonomy/control/controller/mppi_controller/critics/mppi_critics.xml
  INDEX_NAME mppi_critics
  INSTALL_SUBDIR control/controller/mppi_controller/critics
)

list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller_critics)
```

## XML 描述文件

已创建 `mppi_critics.xml` 文件，包含所有 11 个 critics 的定义。如果使用方案 1（单独库），需要为每个 critic 创建单独的 XML 文件。

## 批量处理脚本

可以使用以下 CMake 代码批量处理所有 critics：

```cmake
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

# 批量创建和导出
foreach(critic ${MPPI_CRITICS})
  add_library(${PROJECT_NAME}_control_mppi_controller_${critic} SHARED 
    "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/critics/${critic}.cpp"
  )
  
  target_link_libraries(${PROJECT_NAME}_control_mppi_controller_${critic} PUBLIC
    ${PROJECT_NAME}
    autolink
  )
  
  autolink_export_plugin(
    LIBRARY ${PROJECT_NAME}_control_mppi_controller_${critic}
    DESCRIPTION_FILE autonomy/control/controller/mppi_controller/critics/${critic}.xml
    INDEX_NAME mppi_${critic}
    INSTALL_SUBDIR control/controller/mppi_controller/critics
  )
  
  list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller_${critic})
endforeach()
```

## 优势

使用 `autolink_export_plugin` 的优势：

1. **统一管理**：所有 critics 使用相同的导出方式
2. **自动化**：自动生成索引文件和配置 XML
3. **易于维护**：减少重复代码
4. **测试支持**：自动设置测试环境变量

## 注意事项

1. 如果使用方案 1，需要为每个 critic 创建单独的 XML 文件
2. 如果使用方案 2，所有 critics 必须在同一个库中
3. XML 文件中的 `@PLUGIN_LIBRARY_PATH@` 会被自动替换
4. 确保所有依赖的头文件都正确包含
