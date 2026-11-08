# 07 · 构建与测试

## 1. 依赖

- CMake ≥ 3.22
- C++17
- Protobuf ≥ 3.15（CONFIG）
- automsgs（proto 生成/库）
- Threads
- BehaviorTree.CPP 4.x（behavior nodes）
- Eigen3、glog（autonomy/common math/logging 头）
- 可选：autolink（Component，Linux）
- 可选：autonomy::common + ONNX Runtime / TensorRT（Engine）

## 2. CMake 组织

navigation 是 autonomy 的子模块，`CMakeLists.txt` 只保留 134 行，根工程提供
`autocmake_project()`、基础依赖和安装框架；navigation 自己使用：

- `autocmake_find(behaviortree_cpp)`；
- `autocmake_library()`：导航库、proto、组件；
- `autocmake_binary()`：`navigation_goal_sender`；
- `autocmake_install_share()`：conf / dag / launch。

- `autocmake_library(navigation ...)`：一个导航库 + 兼容别名
- `autocmake_library(navigation_proto ...)`：生成的 proto
- autolink 组件与 goal sender
- 测试、安装

源码和测试都用 `file(GLOB_RECURSE ... CONFIGURE_DEPENDS)` 收集；新增 `.cpp`
不需要改 `CMakeLists.txt`，测试文件自动注册为 CTest。

## 3. 开发构建（macOS/Linux）

```sh
cmake -S /path/to/autonomy -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DAUTONOMY_BUILD_NAVIGATION=ON \
  -DBUILD_TEST=ON

cmake --build build --target navigation
ctest --test-dir build --output-on-failure
```

- automsgs/autolink 由根工程内嵌，不需要单独构建目录。
- proto import 名 `automsgs/msgs/...` 由 navigation CMake 建符号链接。

## 4. Linux + autolink + Engine

```sh
tools/build_linux.sh /path/to/automsgs/build build-linux
```

或：

```sh
cmake -S . -B build -G Ninja \
  -DAUTONOMY_BUILD_NAVIGATION=ON
cmake --build build --target navigation
ctest --test-dir build --output-on-failure
```

部署入口需在 `Runner::Create` 前注册：

```cpp
navigation::planning::RegisterEnginePlugins();
```

仿真默认使用 `RegisterSimulationPlugins()`，不需要 Engine。

## 5. 安装

```sh
cmake --install build --prefix /opt/navigation
```

安装头文件、库、`conf/`、`dag/`、`launch/`、`behavior/trees/`。

## 6. Sanitizer

```sh
cmake -S . -B build-asan -G Ninja -DNAVIGATION_SANITIZER=address ...
cmake --build build-asan
ASAN_OPTIONS=detect_leaks=0 ctest --test-dir build-asan
```

## 7. Benchmark

```sh
./build/navigation_benchmark
```

输出 VoxelGrid / KeyframeFilter / CBF / ChunkBlender P50/P99。

## 8. 配置校验

```sh
./build/navigation_configuration_validator conf
```

## 9. 模型

DeFM：

```sh
python tools/export_defm_onnx.py \
  --checkpoint conf/models/defm_vit_s14.pth \
  --defm-source /path/to/VNT-PA/third_party/defm
```

详见 [06-models.md](06-models.md)。
