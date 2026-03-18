# Autonomy Bazel 构建系统

## 概述

本项目使用 Bazel 作为构建系统，所有模块的 BUILD.bazel 文件都使用统一的宏来简化配置。

## 核心文件

### 1. `tools/autonomy_module.bzl`
这是核心的 Bazel 宏文件，提供了以下可重用宏：

- **`autonomy_module()`** - 创建完整的模块（包含 proto 和 C++ 库）
- **`autonomy_cc_library()`** - 创建 C++ 库
- **`autonomy_proto_library()`** - 创建 proto 库
- **`autonomy_cc_proto_library()`** - 创建 C++ proto 绑定
- **`autonomy_binary()`** - 创建可执行文件

### 2. `tools/autonomy_config.bzl`
提供通用配置常量：
- `AUTONOMY_COPTS` - 编译器选项
- `AUTONOMY_DEFAULT_EXCLUDES` - 默认排除模式
- `AUTONOMY_BASE_DEPS` - 基础依赖

## 使用方法

### 基本模块（带 proto）

```python
load("//autonomy/tools:autonomy_module.bzl", "autonomy_module")

autonomy_module(
    name = "my_module",
    has_proto = True,
    proto_glob = ["proto/*.proto"],
    deps = [
        "//autonomy/common:common",
        "//autonomy/commsgs:commsgs_cc",
    ],
)
```

### 基本模块（无 proto）

```python
load("//autonomy/tools:autonomy_module.bzl", "autonomy_module")

autonomy_module(
    name = "my_module",
    has_proto = False,
    deps = [
        "//autonomy/common:common",
        "//autonomy/commsgs:commsgs_cc",
    ],
)
```

### 带可执行文件的模块

```python
load("//autonomy/tools:autonomy_module.bzl", "autonomy_module", "autonomy_binary")

autonomy_module(
    name = "my_module",
    has_proto = True,
    exclude_patterns = ["main.cpp"],  # 排除可执行文件
    deps = [...],
)

autonomy_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [":my_module"],
)
```

## 配置说明

- **根目录配置**：`src/autonomy/MODULE.bazel` 和 `.bazelrc` 管理整个项目的依赖和配置
- **子模块配置**：各子模块的 `.bazelrc` 和 `MODULE.bazel` 可以删除，因为它们不会被使用
- **版本**：所有模块使用 Bazel 8.4.2（在根目录的 `.bazelversion` 中指定）

## 模块列表

所有模块都已使用简化的宏配置：
- ✅ commsgs
- ✅ common
- ✅ control
- ✅ driver
- ✅ map
- ✅ perception
- ✅ planning
- ✅ prediction
- ✅ sensor
- ✅ simulation
- ✅ system
- ✅ tasks
- ✅ tools
- ✅ transform
- ✅ vehicle
- ✅ visualization
- ✅ bridge
- ✅ localization
