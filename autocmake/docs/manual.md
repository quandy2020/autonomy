# autocmake 使用手册

一套可安装的 CMake 模块，用来管理任意 C/C++ 工程。一个包的配置、编译、安装和导出由宏完成。多个包的依赖顺序由命令 `autocmake` 完成。

要求 CMake 3.20。未另行指定时，C++ 标准是 17。

| 章节 | 内容 |
|---|---|
| 1 安装 | 装上工具包 |
| 2 写一个包 | 一份 `CMakeLists.txt` 的顺序 |
| 3 package.xml | 依赖怎么声明 |
| 4 宏 | 按调用顺序查参数 |
| 5 配置开关 | `-D` 传入的选项 |
| 6 装好之后 | 前缀里有什么，下游怎么链接 |
| 7 多包 | 工作空间命令 |
| 8 示例 | 仓库里的三个例子 |
| 9 不包含 | 这一版不做的事 |

## 1. 安装

在 autonomy 仓库里：

```bash
cmake -S autocmake -B autocmake/build
cmake --build autocmake/build
cmake --install autocmake/build --prefix "$HOME/autocmake"
```

下游配置时加上 `-DCMAKE_PREFIX_PATH=$HOME/autocmake`。还没安装时，用 `-Dautocmake_DIR=<autocmake 的 build 目录>`。

装好之后，前缀里有：

```text
bin/autocmake
lib/cmake/autocmake/autocmake-config.cmake
lib/cmake/autocmake/autocmake-config-version.cmake
lib/cmake/autocmake/Autocmake.cmake
lib/cmake/autocmake/cmake/
lib/cmake/autocmake/codecheck/
lib/cmake/autocmake/tools/
```

`find_package(autocmake CONFIG)` 会包含 `Autocmake.cmake`，后面的宏才可用。版本按主版本兼容：`find_package(autocmake 0.1 CONFIG REQUIRED)` 接受 0.1.x，不接受 1.x。

## 2. 写一个包

一个包一份 `CMakeLists.txt`。源文件放在 `src/`、`include/`、`test/` 或其他目录都行，路径写在这一份文件里。

### 调用顺序

| 顺序 | 调用 | 作用 |
|---|---|---|
| 1 | `project()` | 包名。版本可省略，改由清单提供 |
| 2 | `find_package(autocmake CONFIG REQUIRED)` | 载入宏 |
| 3 | `autocmake_project()` | 布局、C++ 默认、读清单 |
| 4 | `autocmake_find()` | 可选，要在 `autocmake_build()` 之前 |
| 5 | `autocmake_build()` | 按清单找依赖，打开测试 |
| 6 | `autocmake_library()` / `autocmake_binary()` / `autocmake_test()` / `autocmake_protobuf()` | 目标写在同一份文件里 |
| 7 | `autocmake_export()` | 可选，要在收尾之前 |
| 8 | `autocmake_package()` 或 `autocmake_install()` | 收尾，只能一次 |

有清单时，`autocmake_build()` 发现还没跑过 `autocmake_dependencies()`，会自己调用。普通包不必手写这一步。

### 有清单

`example/hello` 的形状：

```text
hello/
  package.xml
  CMakeLists.txt
  include/hello/hello.hpp
  include/hello/greet/greet.hpp
  src/hello.cpp
  src/hello_main.cpp
  greet/src/greet.cpp
  test/hello_TEST.cpp
```

```cmake
cmake_minimum_required(VERSION 3.20)
project(hello VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_build(QUIT)
autocmake_library(hello SOURCES src/hello.cpp)
autocmake_library(greet SOURCES greet/src/greet.cpp)
autocmake_binary(hello_main SOURCES src/hello_main.cpp DEPENDENCIES hello)
autocmake_test(hello_TEST SOURCES test/hello_TEST.cpp DEPENDENCIES hello greet)
autocmake_package()
```

目标名与项目名相同时，导出 `${PROJECT_NAME}::hello`，并另有别名 `${PROJECT_NAME}::core`。第二个库导出为 `${PROJECT_NAME}::greet`。

同一个 CMake 进程里用 `add_subdirectory` 再放一个包时，清单、导出依赖和配置错误记在各自目录上，不会跟父目录串。

### 没有清单

`project()` 必须写 `VERSION`。最后一步用 `autocmake_install()`。`autocmake_package()` 会去读 `package.xml`，文件不存在就配置失败。

```cmake
cmake_minimum_required(VERSION 3.20)
project(hello VERSION 0.1.0)
find_package(autocmake CONFIG REQUIRED)

autocmake_project()
autocmake_build(QUIT)
autocmake_install()
```

## 3. package.xml

格式是 ament package format 2 或 3。

- `<name>` 必须等于 `project()` 的名字。
- `<version>` 必须是 `MAJOR.MINOR.PATCH`。
- `project()` 也写了 `VERSION` 时，两边必须相同。
- `project()` 不写 `VERSION` 时，采用清单里的版本。

```xml
<?xml version="1.0"?>
<package format="3">
  <name>app</name>
  <version>0.1.0</version>
  <description>Application package.</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>autocmake</buildtool_depend>
  <depend>base</depend>
  <build_depend>Eigen3</build_depend>
  <build_export_depend>Eigen3</build_export_depend>
  <exec_depend>runtime_only</exec_depend>
  <test_depend>clang-format</test_depend>

  <export>
    <build_type>autocmake</build_type>
  </export>
</package>
```

### 依赖标签

| 标签 | 本包查找 | 写入下游 `find_dependency` |
|---|---|---|
| `<depend>` | 是，且导出 | 是 |
| `<build_depend>` | 是，`PRIVATE` | 否 |
| `<buildtool_depend>` | `autocmake` 跳过；其他与 `<build_depend>` 相同 | 仅当同时是导出依赖 |
| `<exec_depend>` | 否 | 是 |
| `<build_export_depend>` | 否 | 是 |
| `<buildtool_export_depend>` | 否 | 是 |
| `<test_depend>` | 否 | 否。已知的检查工具交给 `autocmake_lint()` |

`<export><build_type>` 默认按 `autocmake` 处理。写成别的值时，仍然注册 packages 索引，但不再注册 `autocmake` 资源。

### 条件和版本

`condition` 用环境变量求值。变量不存在时按空字符串比较。结果为假的依赖会丢掉。

| 写法 | 含义 |
|---|---|
| `$NAME` | 环境变量 |
| `'...'` 或 `"..."` | 字符串 |
| `==` `!=` | 比较 |
| `and` `or` `not` | 逻辑，可以用括号 |

版本属性加在依赖标签上。同一个依赖写出两个不同的约束，或版本不是 `MAJOR.MINOR` / `MAJOR.MINOR.PATCH`，解析失败。

| 属性 | 效果 |
|---|---|
| `version_eq` | `find_package` 的 `EXACT` |
| `version_gte` | 最低版本 |
| `version_gt` `version_lt` `version_lte` | 找到之后再比较 |

语法不对、包版本不是 `MAJOR.MINOR.PATCH`、或出现 `<group_depend>` 时，解析失败。

### 解析结果

- `${PROJECT_NAME}_VERSION`
- `${PROJECT_NAME}_DESCRIPTION`
- `${PROJECT_NAME}_LICENSE`
- `${PROJECT_NAME}_MAINTAINER`
- `${PROJECT_NAME}_BUILD_DEPENDS`
- `${PROJECT_NAME}_BUILDTOOL_DEPENDS`
- `${PROJECT_NAME}_EXPORT_DEPENDS`
- `${PROJECT_NAME}_TEST_DEPENDS`
- `${PROJECT_NAME}_VERSION_SPECS`：有版本约束的依赖，按「名字、种类、版本」三个一组。种类是 `eq`、`gte`、`gt`、`lte`、`lt`
- `${PROJECT_NAME}_BUILD_TYPE`

要在 `autocmake_build()` 之前确认某个名字确实写在清单里，用 `autocmake_dependencies(REQUIRED <名>)`。这个名字必须出现在 build、buildtool 或 export 依赖中，否则配置失败。`autocmake_dependencies()` 只能调用一次。

## 4. 宏

按第 2 节的顺序排列。

### 配置

**`autocmake_project()`**

在 `project()` 之后调用。未指定 `CMAKE_CXX_STANDARD` 时设为 17，关闭编译器扩展，打开 `CMAKE_POSITION_INDEPENDENT_CODE`。

| 结果 | 值 |
|---|---|
| `PROJECT_LIBRARY_TARGET_NAME` | `${PROJECT_NAME}` |
| `AUTOCMAKE_EXPORT_NAME` | `${PROJECT_NAME}Targets` |
| 库、可执行文件、头文件 | `lib`、`bin`、`include`（跟随 GNUInstallDirs） |
| CMake config | `lib/cmake/${PROJECT_NAME}` |

| 参数 | 含义 |
|---|---|
| `INCLUDE <前缀>` | 记录公开头文件前缀，默认是项目名 |
| `SUFFIX <标签>` | 预发布标记，只进入 `PROJECT_VERSION_FULL`（形如 `0.1.0~rc1`），不进入 CMake 包版本 |
| `EXTRAS <文件>...` | 额外 `.cmake` 或 `.cmake.in`，安装到 config 旁边并在 config 里 `include`。`.cmake.in` 会先做 `@VAR@` 替换 |

**`autocmake_xml([DIRECTORY <目录>])`**

手动解析清单。源码根有 `package.xml` 时，`autocmake_project()` 会自己调用。一个目录只能解析一次。`DIRECTORY` 指向不在源码根上的清单。

**`autocmake_build([QUIT])`**

按清单查找依赖，打印此前记下的警告和错误，然后 `include(CTest)`。找到 `cppcheck` 时添加 codecheck 目标。

| 参数 | 含义 |
|---|---|
| `QUIT` | `REQUIRED` 依赖没找到时 `FATAL_ERROR`。不加则用 `SEND_ERROR` |

**`autocmake_error(<文本>)`** 与 **`autocmake_warning(<文本>)`**

把消息留到 `autocmake_build()` 再打印。

### 依赖

**`autocmake_find(<包名> [REQUIRED] [PRIVATE] [QUIET] [CONFIG] [EXACT] [VERSION <版本>] [COMPONENTS <组件>...] [EXTRA_ARGUMENTS <参数>...])`**

转发给 `find_package`。这里的 `REQUIRED` 不会传给 `find_package`：没找到时先记一条错误，到 `autocmake_build(QUIT)` 再退出。非 `PRIVATE` 且找到了的包，会写成下游的 `find_dependency`。可选包没找到时打印一行状态；加 `QUIET` 则不打印。

```cmake
autocmake_find(Eigen3 REQUIRED PRIVATE)
autocmake_find(Boost VERSION 1.74 COMPONENTS filesystem)
```

**`autocmake_export(<包名>...)`**

在收尾之前，把这些名字追加为下游的 `find_dependency(<包名>)`。清单里已经导出的依赖不必再写一遍。

**`autocmake_link(<目标> [PRIVATE|INTERFACE] <包名>...)`**

把已经找到的包链接到目标上。查找顺序：`${包}::${包}`，然后同名目标，然后 `${包}_LIBRARIES` / `${包}_INCLUDE_DIRS`。默认 `PUBLIC`。

```cmake
autocmake_link(hello PRIVATE Eigen3)
```

### 目标

**`autocmake_library(<目标>)`**

导出为 `${PROJECT_NAME}::<目标>`。目标名与项目名相同时，同时导出 `${PROJECT_NAME}::core`。当前目录有 `include/` 时使用它。库带上 `cxx_std_${CMAKE_CXX_STANDARD}`。

| 参数 | 含义 |
|---|---|
| `SOURCES <文件>...` | 源文件 |
| `DIRECTORY <目录>` | 递归收集该目录下的 C/C++ 源文件 |
| `DEPENDENCIES <目标>...` | `PUBLIC` 链接 |
| `STATIC` / `SHARED` / `INTERFACE` | 库类型。都不写时跟随 `BUILD_SHARED_LIBS` |

`SOURCES` 与 `DIRECTORY` 至少要有一个，除非是 `INTERFACE`。非接口库的 `VERSION` 是项目版本，`SOVERSION` 是主版本号。

**`autocmake_binary(<目标> SOURCES <文件>... [DEPENDENCIES <目标>...])`**

可执行文件。`DEPENDENCIES` 以 `PRIVATE` 链接，并安装到 `bin/`。

共享库和可执行文件会设置 `INSTALL_RPATH`。可执行文件指向安装后的库目录，默认是 `@loader_path/../lib`（其他平台是 `$ORIGIN/../lib`）。`CMAKE_INSTALL_LIBDIR` 不是 `lib` 时，用同样的相对路径。共享库的 RPATH 是 `@loader_path` 或 `$ORIGIN`，用来找同一目录里的库。

**`autocmake_protobuf(<目标> SOURCES <文件>... [IMPORTS <目录>...] [DEPENDENCIES <目标>...])`**

编译 proto3，同一次 `protoc` 生成 C++ 和 Python。C++ 放进这个库；目标不存在时会创建。要在 `autocmake_project()` 之后、收尾之前调用。文件开头必须是 `syntax = "proto3"`。库会 `PUBLIC` 链接 `protobuf::libprotobuf`，并写成 `find_dependency(Protobuf)`。`protoc` 低于 3.15 时自动加上 `--experimental_allow_proto3_optional`。

`IMPORTS` 的第一项是布局根，默认是当前源码目录。后面的目录只作为 `protoc` 的搜索路径，用来找 `import "sibling.proto"`，或另一个包安装出来的 `${那个包}_PROTO_PATH`。

以 `proto/a.proto` 为例：

| 产物 | 位置 |
|---|---|
| C++ 头文件 | `include/proto/a.pb.h`，源码里写 `#include "proto/a.pb.h"` |
| `.proto` | `share/<包>/proto/a.proto` |
| Python | `<lib>/python/proto/a_pb2.py`，每一层目录有 `__init__.py` |
| `<包>_PROTO_PATH` | 安装后指向 `share/<包>`；构建时是布局根 |
| `<包>_PYTHON_PATH` | 安装后指向 `<lib>/python`；构建时是当前二进制目录 |

`library_path.sh` 会把 `<lib>/python` 加进 `PYTHONPATH`。Python 包 `protobuf` 的版本不能低于生成文件开头写的 `Protobuf Python Version`。

```cmake
autocmake_protobuf(messages SOURCES proto/greeting.proto proto/note.proto)
autocmake_binary(messages_main SOURCES src/main.cpp DEPENDENCIES messages)
```

```python
from proto import greeting_pb2

greeting = greeting_pb2.Greeting()
greeting.text = "ok"
greeting.value = 3
```

另一个包引用这些描述文件：

```cmake
autocmake_protobuf(app_messages
  SOURCES proto/app.proto
  IMPORTS ${CMAKE_CURRENT_SOURCE_DIR} ${messages_PROTO_PATH}
  DEPENDENCIES messages::messages)
```

要让头文件落成 `autolink/proto/simple.pb.h`，布局根取 `autolink/` 的上一级，并把放 `.proto` 的目录加进搜索路径，这样 `import "qos_profile.proto"` 仍然能找到：

```cmake
autocmake_protobuf(autolink
  SOURCES autolink/proto/simple.proto
  IMPORTS ${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_SOURCE_DIR}/proto)
```

### 测试

**`autocmake_test(<目标> SOURCES <文件>... [DEPENDENCIES <目标>...] [NO_MAIN])`**

注册一个 gtest 可执行文件，名字同时是 CTest 名字。测试不安装。`BUILD_TESTING` 关闭时什么都不做。`NO_MAIN` 链接 `GTest::gtest`，否则链接带 main 的目标。

结果写到构建目录的 `test_results/<目标>.xml`。随后的 `check_<目标>` 跑 `tools/check_test_ran.py`：没有这份结果文件时，检查失败。

本机已经装了 GTest 就直接用。否则在打开测试时拉取 googletest 1.15.2，并关闭 GTest 自己的安装和 gmock。

**`autocmake_lint()`**

`BUILD_TESTING` 打开时，按 `test_depend` 注册 CTest。名字可以是工具本身，也可以是以该工具结尾的 ament 包名（如 `ament_cmake_cppcheck`）。程序没安装时，这项测试失败。清单里没有点名的 `test_depend` 不会变成测试。

| `test_depend` | 命令 |
|---|---|
| `clang-format` | `clang-format --dry-run --Werror` |
| `cppcheck` | `cppcheck`，C++17 |
| `cpplint` | 系统里的 `cpplint` |
| `flake8` / `pycodestyle` / `pyflakes` / `pep257` / `mypy` | 对应的 Python 检查。`pep257` 也接受 `pydocstyle` |
| `xmllint` | `xmllint --noout` |
| `lint_cmake` | `cmakelint` |
| `uncrustify` | 包根要有 `uncrustify.cfg` |
| `clang-tidy` | 构建目录里要有 `compile_commands.json` |
| `copyright` | 源文件开头要有 `Copyright` |

`autocmake_build()` 在找到 `cppcheck` 时添加实际可用的目标：`cppcheck`、`codecheck`，以及 PATH 上有 `cpplint` 时的 `cpplint`。`codecheck` 跑两遍 cppcheck（风格检查，以及 `missingInclude`），并使用 `codecheck/*.rule`。当前 cppcheck 若不认识 `--rule-file`，规则文件会被跳过。源码根或构建目录里的 `cppcheck.suppress` 会传给 cppcheck。机器上没有 cppcheck 时不添加这些目标，配置仍然成功。

```bash
cmake --build <包的 build 目录> --target codecheck
```

### 收尾

**`autocmake_package()`**

安装 `share/${PROJECT_NAME}/package.xml`，注册索引，安装环境钩子，然后调用 `autocmake_install()`。每个包只能调用一次，放在末尾。安装结果见第 6 节。

**`autocmake_install()`**

安装 `include/` 下的 `.h` `.hh` `.hpp` `.hxx` `.inl` `.ipp` `.tpp`，生成 `${PROJECT_NAME}-config.cmake` 与版本文件，安装导出集，并添加 `uninstall` 目标。版本兼容策略是同一主版本。没有编译目标时不安装 targets 文件。没有清单、只要 CMake 导出时，用它代替 `autocmake_package()`。

### 可选项

这些宏不会自动跑。

**`autocmake_documentation([DOXYFILE <路径>])`**

找到 Doxygen，且存在 Doxyfile 时，添加 `docs` 和 `doc_check`。默认使用源码根的 `Doxyfile`。两者缺一就打印状态并返回，不让配置失败。`docs` 把警告写到构建目录的 `autocmake-doxygen.warn`。`doc_check` 依赖 `docs`，警告文件非空则失败；文件不存在则跳过。

**`autocmake_package_version(<package.xml> <前缀>)`**

只读版本号，得到 `<前缀>_VERSION` 以及 `_MAJOR`、`_MINOR`、`_PATCH`。

**`autocmake_sanitize([ADDRESS] [UNDEFINED] [THREAD])`**

给当前目录及子目录加上 `-fsanitize=`。参数都省略时等于 `ADDRESS` 加 `UNDEFINED`。`THREAD` 不能和前两者一起用。MSVC 上直接跳过。

## 5. 配置开关

在 `autocmake_build()` 里生效，用 `-D` 传入：

| 变量 | 默认 | 含义 |
|---|---|---|
| `BUILD_SHARED_LIBS` | `ON` | 库默认动态链接 |
| `AUTOCMAKE_HIDE_SYMBOLS` | `OFF` | `ON` 时隐藏 C 和 C++ 符号，除非源码显式导出 |
| `AUTOCMAKE_SANITIZER` | `OFF` | `ON` 时打开 AddressSanitizer 和 UndefinedBehaviorSanitizer |
| `BUILD_TESTING` | `ON` | 关掉之后不注册 gtest |

GCC、Clang、AppleClang 加 `-Wall -Wextra -Wpedantic`。MSVC 加 `/W4`。没有启用任何语言时，不添加这些选项。

## 6. 装好之后

`hello` 再加一个库 `greet`，`autocmake_package()` 之后，默认前缀里是：

```text
include/hello/...
lib/libhello.dylib
lib/libgreet.dylib
bin/hello_main
lib/cmake/hello/hello-config.cmake
lib/cmake/hello/hello-config-version.cmake
lib/cmake/hello/helloTargets.cmake
share/hello/package.xml
share/hello/environment/library_path.sh
share/hello/environment/library_path.dsv
share/autocmake_index/resource_index/packages/hello
share/autocmake_index/resource_index/autocmake/hello
```

库目录和可执行文件目录跟随 `CMAKE_INSTALL_LIBDIR`、`CMAKE_INSTALL_BINDIR`。上面按默认的 `lib` 和 `bin` 来写。动态库扩展名随平台变化。

只用 `autocmake_install()`、没有清单时，没有 `share/hello/package.xml`、环境钩子和索引。

下游：

```cmake
find_package(hello CONFIG REQUIRED)
target_link_libraries(app PRIVATE hello::core hello::greet)
```

`hello-config.cmake` 先 `find_dependency` 那些被导出的依赖，再包含 targets 文件。没有编译目标时不包含 targets 文件。库目标名与项目名相同时，再补上 `hello::core`。`greet` 导出为 `hello::greet`。

`library_path.sh` 从 `share/<包>/environment` 向上三级得到前缀，把安装时的 bindir、libdir 和 `libdir/python` 加进 `PATH`、库路径和 `PYTHONPATH`。macOS 用 `DYLD_LIBRARY_PATH`，其他系统用 `LD_LIBRARY_PATH`。`library_path.dsv` 是同样四条路径的 ament dsv 记录。

```bash
source "$PREFIX/share/hello/environment/library_path.sh"
```

删除已安装文件：

```bash
cmake --build <包的 build 目录> --target uninstall
```

它按这次配置的顶层构建目录里的 `install_manifest.txt` 删除。先安装过才能卸载。

## 7. 多包

`autocmake` 命令只做排序，然后逐包调用 CMake。每个包自己的 `CMakeLists.txt` 仍然使用上面的宏。源码树里的入口是 `autocmake/scripts/autocmake`，安装后是 `bin/autocmake`。Python 只用标准库。

### 发现和排序

工作空间里每个包一份 `package.xml`。

- 从 `--base-path` 往下找 `package.xml`。默认是当前目录。可以重复 `--base-path`
- 遇到包之后不再进入该包的子目录
- 跳过 `build`、`install`、`log`、`Testing`、`CMakeFiles`、`.git`，以及点开头的目录
- 目录里有 `AUTOCMAKE_IGNORE` 或 `COLCON_IGNORE` 时，整棵子树跳过
- 同名包出现两次则失败

参与排序的依赖，是清单里的 `buildtool_depend` 和 `build_depend`（`<depend>` 会计入 `build_depend`）中、并且也在这个工作空间里的包。`autocmake` 不当作工作空间包。工作空间以外的依赖留给 `find_package`。有环则失败。

`example/workspace` 里目录名是 `app` 在前、`base` 在后，但 `app` 的清单依赖 `base`。`autocmake list` 先打印 `base`。

### 命令

```bash
autocmake list --base-path example/workspace
autocmake build --base-path example/workspace --install-base install
autocmake test --base-path example/workspace
autocmake index --prefix install
```

**`autocmake list [--base-path <目录>]...`**

按构建顺序打印包名。

**`autocmake build [范围] [--generator <生成器>] [--cmake-args <参数>...]`**

对每个选中的包依次配置、编译、安装到同一个 `--install-base`（默认 `install`）。构建目录是 `--build-base/<包名>`（默认 `build/<包名>`）。默认构建类型是 Release。

前缀会放到后续包的 `CMAKE_PREFIX_PATH` 最前面，然后是环境变量 `CMAKE_PREFIX_PATH`，再然后是 `--cmake-args` 里的 `-DCMAKE_PREFIX_PATH=`。用户写的 `-DCMAKE_INSTALL_PREFIX=` 会被丢掉，安装位置只由 `--install-base` 决定。`--cmake-args` 要放在命令最后；如果第一个参数是 `--`，会被去掉。

```bash
autocmake build --base-path ws --cmake-args -DCMAKE_BUILD_TYPE=Debug -DAUTOCMAKE_SANITIZER=ON
```

**`autocmake test [范围]`**

对已经构建的包跑 `ctest`。还没构建则失败。某个包没有测试、CTest 打印 `No tests were found!!!` 并返回 0 时，算通过。

**`autocmake index --prefix <安装前缀>`**

列出 `share/autocmake_index/resource_index/packages/` 下的包名。索引不存在或为空则失败。

`build` 和 `test` 都能用范围参数。两者不能同时给。

| 参数 | 含义 |
|---|---|
| `--packages-select <名>...` | 只构建这些包。若某个包的工作空间依赖没被选中，直接失败 |
| `--packages-up-to <名>...` | 构建这些包以及它们的工作空间祖先 |

## 8. 示例

仓库里有三个例子，由 autocmake 自己的 CTest 调用。在 autocmake 的 build 目录执行 `ctest --output-on-failure`。

| 例子 | 覆盖 |
|---|---|
| `example/hello` | 两个库、可执行文件、gtest、清单和索引。消费者链接 `hello::core` 与 `hello::greet` |
| `example/workspace` | `app` 依赖 `base`。`base` 的 `project()` 不写版本，版本来自清单。构建顺序来自清单，不来自目录名 |
| `example/messages` | `autocmake_protobuf()` 同时生成 C++ 和 Python。`messages_main` 打印 `ok 3` |

## 9. 不包含

- gz-cmake 里 Gazebo 专用的 `Find*` 模块
- pkg-config 封装
- `<group_depend>`。写了会在解析清单时失败
- ament 那套自动 source 环境钩子的工具。这里只安装 `library_path.sh` 和 `library_path.dsv`
- gRPC 插件、按仓库模块扫描 `.proto`、把 Protobuf 固定在 3.19。这些仍在仓库根的 `cmake/`

单包 proto3 用 `autocmake_protobuf()`。模块图和特性依赖仍在仓库根的 `cmake/`。
