# autolink_export_plugin CMake Module

简洁易用的 CMake 函数模块，用于导出和加载 autolink 插件库及其描述文件。

## 功能特性

- ✅ 自动配置插件描述 XML 文件
- ✅ 自动生成插件索引文件
- ✅ 自动设置安装规则
- ✅ 自动配置测试环境变量
- ✅ 最小化代码，易于维护

## 使用方法

### 1. 基本用法

```cmake
# 包含模块
include("${CMAKE_SOURCE_DIR}/cmake/autolink_export_plugin.cmake")

# 创建插件库
add_library(my_plugin SHARED
  my_plugin.cpp
)

target_link_libraries(my_plugin PUBLIC
  autolink
)

# 导出插件
autolink_export_plugin(
  LIBRARY my_plugin
  DESCRIPTION_FILE my_plugin.xml
)
```

### 2. 完整示例

```cmake
include("${CMAKE_SOURCE_DIR}/cmake/autolink_export_plugin.cmake")

# 创建插件库
add_library(my_plugin SHARED
  my_plugin.cpp
)

target_include_directories(my_plugin PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(my_plugin PUBLIC
  autolink
)

# 设置输出目录（可选）
set_target_properties(my_plugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/autolink/my_plugins
  OUTPUT_NAME my_plugin
)

# 导出插件
autolink_export_plugin(
  LIBRARY my_plugin
  DESCRIPTION_FILE my_plugin.xml
  INDEX_NAME my_plugin_index
  INSTALL_SUBDIR my_plugins
)

# 如果需要在测试中使用插件
if(BUILD_TEST)
  add_executable(my_plugin_test test.cpp)
  target_link_libraries(my_plugin_test PUBLIC
    autolink
    my_plugin
  )
  
  # 自动设置测试环境变量
  autolink_setup_plugin_test_environment(my_plugin_test)
endif()
```

## API 参考

### autolink_export_plugin

导出插件库和描述文件。

**参数：**

- `LIBRARY <target_name>` (必需)
  - CMake 目标名称（插件库）

- `DESCRIPTION_FILE <xml_file>` (必需)
  - 插件描述 XML 文件路径（相对于当前源目录）

- `INDEX_NAME <index_name>` (可选)
  - 插件索引文件名，默认为库目标名称

- `INSTALL_SUBDIR <subdir>` (可选)
  - 安装子目录，默认为 `pluginlib`

**示例：**

```cmake
autolink_export_plugin(
  LIBRARY my_plugin
  DESCRIPTION_FILE my_plugin.xml
  INDEX_NAME my_plugin_index
  INSTALL_SUBDIR my_plugins
)
```

### autolink_setup_plugin_test_environment

为测试设置环境变量，使其能够找到导出的插件。

**参数：**

- `test_target` (必需)
  - 测试可执行文件的 CMake 目标名称

**示例：**

```cmake
autolink_setup_plugin_test_environment(my_test)
```

## XML 描述文件格式

插件描述 XML 文件应使用 `@PLUGIN_LIBRARY_PATH@` 占位符，函数会自动替换为正确的库路径。

**示例 XML 文件：**

```xml
<?xml version="1.0"?>
<library path="@PLUGIN_LIBRARY_PATH@">
  <class name="my_plugin/foo" type="my_plugins::Foo" base_class="my_base::Base">
    <description>This is a foo plugin.</description>
  </class>
</library>
```

## 安装位置

插件文件会被安装到以下位置：

- **库文件**: `${CMAKE_INSTALL_PREFIX}/lib/autolink/<INSTALL_SUBDIR>/`
- **描述文件**: `${CMAKE_INSTALL_PREFIX}/share/autolink/<INSTALL_SUBDIR>/`
- **索引文件**: `${CMAKE_INSTALL_PREFIX}/share/autolink_plugin_index/`

## 环境变量

测试时，函数会自动设置以下环境变量：

- `AUTOLINK_PLUGIN_LIB_PATH`: 插件库搜索路径
- `AUTOLINK_PLUGIN_DESCRIPTION_PATH`: 插件描述文件搜索路径
- `AUTOLINK_PLUGIN_INDEX_PATH`: 插件索引文件搜索路径

## 注意事项

1. `autolink_export_plugin()` 必须在创建库目标之后调用
2. XML 文件中的 `@PLUGIN_LIBRARY_PATH@` 会被自动替换
3. 库路径在 XML 中是相对于 `AUTOLINK_PLUGIN_LIB_PATH` 环境变量的
4. 多个插件可以共享同一个 `INSTALL_SUBDIR`
