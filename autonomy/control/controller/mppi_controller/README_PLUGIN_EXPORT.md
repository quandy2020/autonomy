# MPPI Controller 插件导出指南

## 概述

`mppi_controller` 可以使用 `autolink_export_plugin` CMake 函数模块来优雅地导出插件库和描述文件。

## 当前状态

- ✅ 已使用 `CLASS_LOADER_REGISTER_CLASS` 注册插件
- ✅ 已在 `controller_plugins.cmake` 中构建为插件库
- ❌ 缺少 XML 描述文件
- ❌ 未使用 `autolink_export_plugin` 函数

## 使用方法

### 方案 1: 修改 controller_plugins.cmake（推荐）

在 `src/autonomy/cmake/plugins/controller_plugins.cmake` 中，将：

```cmake
# mppi_controller
add_library(${PROJECT_NAME}_control_mppi_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/controller.cpp"
)
list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller)
```

改为：

```cmake
# 包含 autolink_export_plugin 函数
include("${PROJECT_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# mppi_controller
add_library(${PROJECT_NAME}_control_mppi_controller SHARED 
  "${PROJECT_SOURCE_DIR}/autonomy/control/controller/mppi_controller/controller.cpp"
)

# 链接依赖库（如果需要）
target_link_libraries(${PROJECT_NAME}_control_mppi_controller PUBLIC
  ${PROJECT_NAME}
  autolink
)

# 导出插件
autolink_export_plugin(
  LIBRARY ${PROJECT_NAME}_control_mppi_controller
  DESCRIPTION_FILE autonomy/control/controller/mppi_controller/mppi_controller.xml
  INDEX_NAME mppi_controller
  INSTALL_SUBDIR control/controller
)

list(APPEND plugin_libs ${PROJECT_NAME}_control_mppi_controller)
```

### 方案 2: 创建独立的 CMakeLists.txt

在 `mppi_controller` 目录下创建 `CMakeLists.txt`：

```cmake
# 包含 autolink_export_plugin 函数
include("${CMAKE_SOURCE_DIR}/autolink/cmake/autolink_export_plugin.cmake")

# 创建插件库
add_library(autonomy_control_mppi_controller SHARED
  controller.cpp
  # 添加其他需要的源文件
)

target_include_directories(autonomy_control_mppi_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}
)

target_link_libraries(autonomy_control_mppi_controller PUBLIC
  autonomy
  autolink
)

# 导出插件
autolink_export_plugin(
  LIBRARY autonomy_control_mppi_controller
  DESCRIPTION_FILE mppi_controller.xml
  INDEX_NAME mppi_controller
  INSTALL_SUBDIR control/controller
)
```

## XML 描述文件

已创建 `mppi_controller.xml` 文件，内容如下：

```xml
<?xml version="1.0"?>
<library path="@PLUGIN_LIBRARY_PATH@">
  <class name="autonomy_control/mppi_controller" 
         type="autonomy::control::controller::mppi_controller::MPPIController" 
         base_class="autonomy::control::common::ControllerInterface">
    <description>MPPI (Model Predictive Path Integral) Controller plugin for path following</description>
  </class>
</library>
```

## 插件注册宏

当前代码使用 `CLASS_LOADER_REGISTER_CLASS`，这是正确的。如果需要，也可以使用 `PLUGINLIB_EXPORT_CLASS`（它内部调用 `CLASS_LOADER_REGISTER_CLASS`）：

```cpp
// 在 controller.cpp 末尾
#include "autolink/pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  autonomy::control::controller::mppi_controller::MPPIController,
  autonomy::control::common::ControllerInterface
)
```

## 优势

使用 `autolink_export_plugin` 的优势：

1. **自动化处理**：自动配置 XML 文件、生成索引文件、设置安装规则
2. **统一管理**：所有插件使用相同的导出方式
3. **易于维护**：代码更简洁，减少重复配置
4. **测试支持**：自动设置测试环境变量

## 注意事项

1. XML 文件中的 `@PLUGIN_LIBRARY_PATH@` 会被自动替换
2. 确保库路径在 `AUTOLINK_PLUGIN_LIB_PATH` 环境变量中
3. 插件索引文件会自动生成到 `share/autolink_plugin_index/` 目录
