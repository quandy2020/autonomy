# 版本管理系统说明

## 概述

项目使用统一的 `version.json` 文件和 `cmake/version.cmake` CMake 函数来管理版本信息。

## 文件结构

```
autonomy/
├── version.json                    # 版本信息文件（项目根目录）
├── cmake/
│   └── version.cmake              # 版本读取函数
├── CMakeLists.txt                 # 主项目配置（调用 read_version_from_json）
└── autolink/
    └── CMakeLists.txt             # 子项目配置（调用 read_version_from_json）
```

## version.json 格式

```json
{
  "version": "0.0.3",
  "major": 0,
  "minor": 0,
  "patch": 3,
  "name": "Autonomy",
  "description": "Autonomous Robot Development Framework",
  "build_date": "2025-11-20"
}
```

### 字段说明

- `version`: 完整版本号字符串（必填）
- `major`: 主版本号（必填）
- `minor`: 次版本号（必填）
- `patch`: 修订版本号（必填）
- `name`: 项目名称（可选）
- `description`: 项目描述（可选）
- `build_date`: 构建日期（可选）

## CMake 函数：read_version_from_json

### 函数签名

```cmake
read_version_from_json(<prefix> <version_file>)
```

### 参数

- `<prefix>`: 变量前缀（如 `AUTONOMY` 或 `AUTOLINK`）
- `<version_file>`: version.json 文件的完整路径

### 设置的变量

函数会在父作用域设置以下变量：

- `<PREFIX>_MAJOR_VERSION` - 主版本号
- `<PREFIX>_MINOR_VERSION` - 次版本号
- `<PREFIX>_PATCH_VERSION` - 修订版本号
- `<PREFIX>_VERSION` - 完整版本号（`major.minor.patch`）
- `<PREFIX>_SOVERSION` - 共享库版本号（`major.minor`）
- `<PREFIX>_NAME` - 项目名称
- `<PREFIX>_DESCRIPTION` - 项目描述（如果存在）
- `<PREFIX>_BUILD_DATE` - 构建日期（如果存在）

### 使用示例

#### 在主项目中使用

```cmake
# CMakeLists.txt
include("${PROJECT_SOURCE_DIR}/cmake/version.cmake")
read_version_from_json(AUTONOMY "${PROJECT_SOURCE_DIR}/version.json")

# 使用版本变量
message(STATUS "Building ${AUTONOMY_NAME} version ${AUTONOMY_VERSION}")

# 设置库版本
set_target_properties(mylib PROPERTIES
  VERSION ${AUTONOMY_VERSION}
  SOVERSION ${AUTONOMY_SOVERSION}
)
```

#### 在子项目中使用

```cmake
# autolink/CMakeLists.txt
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/version.cmake")
read_version_from_json(AUTOLINK "${CMAKE_CURRENT_SOURCE_DIR}/../version.json")

# 使用版本变量
message(STATUS "Building Autolink version ${AUTOLINK_VERSION}")
```

## 版本更新方法

### 方法 1: 手动编辑

直接编辑 `version.json` 文件：

```bash
vim version.json
# 修改版本号
{
  "version": "1.0.0",
  "major": 1,
  "minor": 0,
  "patch": 0,
  ...
}
```

### 方法 2: 使用更新脚本

使用提供的 `scripts/update_version.sh` 脚本：

```bash
# 更新到版本 1.0.0
./scripts/update_version.sh 1 0 0
```

脚本会自动：
- 更新 `version.json` 中的所有版本字段
- 更新构建日期
- 提示后续操作步骤

### 重新配置 CMake

版本更新后，需要重新配置 CMake：

```bash
cd build
cmake ..
make -j$(nproc)
```

## 版本号规范

遵循[语义化版本](https://semver.org/lang/zh-CN/)规范：

- **主版本号 (MAJOR)**: 不兼容的 API 变更
- **次版本号 (MINOR)**: 向下兼容的功能性新增
- **修订号 (PATCH)**: 向下兼容的问题修正

## 配置输出示例

```
-- Autonomy version: 0.0.3
-- Autonomy version: 0.0.3
-- Configuring done (63.4s)
-- Generating done (0.7s)
```

## 常见问题

### Q: 修改 version.json 后版本号没有更新？

A: 清除 CMake 缓存并重新配置：

```bash
cd build
rm -rf CMakeCache.txt CMakeFiles
cmake ..
```

### Q: 可以为不同的子项目设置不同的版本吗？

A: 可以。在子项目目录创建独立的 `version.json`，并在子项目的 CMakeLists.txt 中指向该文件。

### Q: version.json 中的字段顺序重要吗？

A: 不重要。CMake 函数使用正则表达式解析，字段顺序不影响结果。

## 技术细节

- 正则表达式支持 JSON 中的换行和空格
- 使用 `DEFINED` 检查匹配结果，避免数字 `0` 被判断为 FALSE
- 所有变量使用 `PARENT_SCOPE` 设置，确保在调用作用域可用
- 如果项目名称未在 JSON 中定义，使用前缀作为默认值

