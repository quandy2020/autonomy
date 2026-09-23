# autocmake

可安装的 CMake 模块包，用来管理任意 C/C++ 工程。用法对齐 [gz-cmake](https://github.com/gazebosim/gz-cmake) 的工程流程，依赖导出和测试对齐 [ament_cmake](https://github.com/ament/ament_cmake)。

本仓库的 `cmake/` 仍是 autonomy 专用（模块图、proto、特性依赖）。`autocmake` 提供其中可复用的那一层，后续工程改为 `find_package(autocmake)`，不再复制一套 `*_configure_project`。

完整用法见 [docs/manual.md](docs/manual.md)。

## 安装

```bash
cmake -S autocmake -B autocmake/build
cmake --build autocmake/build
cmake --install autocmake/build --prefix "$HOME/autocmake"
```

下游配置时加上 `-DCMAKE_PREFIX_PATH=$HOME/autocmake`。未安装时也可以 `-Dautocmake_DIR=<autocmake 的 build 目录>`。

## 下游工程

一个包一份 `CMakeLists.txt`。`include/`、`src/`、`test/` 只放源文件。

```cmake
cmake_minimum_required(VERSION 3.20)
project(hello VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_find(Eigen3 REQUIRED PRIVATE)   # 缺依赖先记下来，不立刻中断
autocmake_build(QUIT)
autocmake_library(hello SOURCES src/hello.cpp)
autocmake_library(greet SOURCES greet/src/greet.cpp)
autocmake_binary(hello_main SOURCES src/hello_main.cpp DEPENDENCIES hello)
autocmake_test(hello_TEST SOURCES test/hello_TEST.cpp DEPENDENCIES hello greet)
autocmake_package()
```

`autocmake_project()` 读 `package.xml`。`autocmake_build()` 按清单找依赖，并打开测试。库、可执行文件和测试写在同一个文件里。

安装后：

```cmake
find_package(hello CONFIG REQUIRED)
target_link_libraries(app PRIVATE hello::core hello::greet)
```

## 和现有接口的对应

| `cmake/` / gz-cmake / ament | autocmake |
|---|---|
| `autonomy_configure_project` / `gz_configure_project` | `autocmake_project` |
| `gz_find_package` | `autocmake_find` |
| `gz_configure_build` | `autocmake_build` |
| `autonomy_library` / `gz_create_core_library` / `ament_auto_add_library` | `autocmake_library` |
| `autonomy_binary` / `gz_build_executables` | `autocmake_binary` |
| `autonomy_test` / `gz_build_tests` / `ament_add_gtest` | `autocmake_test` |
| `ament_target_dependencies` | `autocmake_link` |
| `autonomy_create_packages` / `gz_create_packages` / `ament_package` | `autocmake_install` / `autocmake_package` |

`autocmake_package()` 每个包调用一次，放在末尾：安装 `package.xml`、注册 `share/autocmake_index`、导出 CMake config。`autocmake_find` 的非 `PRIVATE` 依赖和 `autocmake_export()` 会写成下游的 `find_dependency`。`REQUIRED` 失败先进入错误列表，由 `autocmake_build(QUIT)` 统一退出。

GTest 已安装则直接用；否则在打开测试时拉取 1.15.2。默认 C++17，`BUILD_SHARED_LIBS` 默认 ON。符号默认可见，`AUTOCMAKE_HIDE_SYMBOLS=ON` 时改为隐藏。

## 多包工作空间

构建系统和构建工具分开，对应 ament_cmake 与 colcon：

| ament | autocmake |
|---|---|
| `package.xml` | 同名清单。`<depend>` 同时编译并导出；`<build_depend>` 只在本包使用；`<exec_depend>` / `<build_export_depend>` 只导出 |
| `ament_package()` | `autocmake_package()` |
| `ament_export_dependencies()` | `autocmake_export()` |
| `ament_index` | `share/autocmake_index/resource_index/packages/<包名>` |
| `colcon` | `autocmake build` / `autocmake test` / `autocmake list` / `autocmake index` |
| `ament_lint_auto` | `autocmake_lint()`。`test_depend` 点名的检查（`clang-format`、`cppcheck`、`cpplint`、`flake8`、`pycodestyle`、`pyflakes`、`pep257`、`mypy`、`xmllint`、`lint_cmake`、`uncrustify`、`clang-tidy`、`copyright`）会变成 CTest |

`project()` 可以不写 `VERSION`，`autocmake_project()` 会用 `package.xml` 里的版本。包名必须和 `project()` 一致。清单依赖由 `autocmake_build()` 查找；要提前检查某个名字是否写在清单里，用 `autocmake_dependencies(REQUIRED <名>)`。

```bash
autocmake list --base-path example/workspace
autocmake build --base-path example/workspace --install-base install
autocmake test --base-path example/workspace
autocmake index --prefix install
```

`build` 按依赖拓扑安装到同一个前缀，并把该前缀放进后续包的 `CMAKE_PREFIX_PATH`。`AUTOCMAKE_IGNORE` 或 `COLCON_IGNORE` 会跳过目录。命令安装到 `bin/autocmake`，源码树入口是 `autocmake/scripts/autocmake`。`autocmake_package()` 还会安装 `share/<包>/environment/library_path.sh`，把 `bin`、`lib` 和 `lib/python` 加进 `PATH`、库路径和 `PYTHONPATH`。

`example/workspace` 里 `app` 依赖 `base`。`app` 的目录名排在 `base` 前面，用来确认顺序来自 `package.xml`，不是目录名。

## 目录

```text
autocmake/
  Autocmake.cmake                 # find_package 之后的总入口
  cmake/                      # 工程、目标、依赖、清单、检查，以及 Config
  codecheck/                  # cppcheck 规则，以及 lint / codecheck 脚本
  tools/                      # 清单版本、测试结果、Doxygen 警告
  example/                    # hello、两包工作空间、proto3
  tests/                      # ctest 脚本
```

## 示例

`example/hello` 与 `example/workspace` 都是一个包一份 `CMakeLists.txt`。hello 覆盖两个库、可执行文件、gtest、清单和索引。workspace 覆盖两个包的依赖顺序。`example/messages` 覆盖 proto3。在 autocmake 的 build 目录执行：

```bash
ctest --output-on-failure
```

## 这一版不包含

gz-cmake 的 Gazebo 专用 `Find*`、pkg-config。`package.xml` 的 `condition` 按环境变量求值；`<group_depend>` 会让解析失败。有 `Doxyfile` 时用 `autocmake_documentation()` 调 Doxygen。
