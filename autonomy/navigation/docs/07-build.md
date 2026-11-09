# 07 · 构建与测试

## 1. 当前构建环境

navigation 是 autonomy umbrella 工程的子模块，使用根工程提供的 autocmake、
automsgs、autolink 与 autonomy_common。不能把 navigation 目录当作独立 CMake 工程。

本轮验证环境为 Linux `SpaceHero` 容器，workspace 为 `/workspace/autonomy`，
源码根目录为 `/workspace/autonomy/src/autonomy`。依赖包括 CMake ≥ 3.22、C++20、
工程固定的 Protobuf 3.19.4、Threads、Eigen3、glog、OpenSSL Crypto 和 BehaviorTree.CPP 4.x。
autolink 组件依赖 Linux；macOS 上的源码检查不能替代组件运行验证。

## 2. colcon 构建与导航测试

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy \
  --cmake-args -DNAVIGATION_BUILD_TESTS=ON
ctest --test-dir build/autonomy/autonomy/navigation \
  --output-on-failure --no-tests=error -j 8
```

`NAVIGATION_BUILD_TESTS=ON` 独立开启导航测试，不要求开启其他模块的 `BUILD_TEST`。
新增 `*_test.cpp` 会自动注册；配置校验和组件集成测试由 CMake 显式注册。

也可从 navigation 目录执行：

```sh
tools/build_linux.sh /workspace/autonomy 8
```

脚本默认从自己的位置推导 colcon workspace，第二个参数为 CTest 并发数。
构建会执行 colcon 的安装步骤。当前测试结果与环境证据见
[16-linux-validation.md](16-linux-validation.md)。

## 3. ONNX Runtime 后端

DeFM C++ 验证需要真正编入推理后端。仅 `BUILD_ONNXRUNTIME=ON` 不足以保证后端可用，
还需 CMake 找到 C++ SDK。本轮使用 CPU SDK 1.23.2：

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy --cmake-args \
  -DNAVIGATION_BUILD_TESTS=ON -DBUILD_ONNXRUNTIME=ON \
  -DOnnxRuntime_ROOT=/workspace/autonomy/.deps/onnxruntime-linux-x64-1.23.2
```

检查 `build/autonomy/CMakeCache.txt` 中的 `OnnxRuntime_INCLUDE_DIR` 与
`OnnxRuntime_LIBRARY`，并运行模型验证工具确认。部署机器仍需提供相应共享库；
本轮未制作包含全部依赖的独立发布包。

## 4. 配置与组件联调

```sh
build/autonomy/bin/navigation_configuration_validator \
  src/autonomy/autonomy/navigation/conf
ctest --test-dir build/autonomy/autonomy/navigation \
  -R '^component_integration$' --output-on-failure
```

集成测试解析 navigation/safety/guardian 三个 DAG，实例化五个具体组件。
测试使用合成传感器和差速积分模型，在独立 IPC/挂载命名空间及独立 `/dev/shm`、`/tmp` 中
启用 Autolink `MODE_REALITY` 的真实定时器与传输。包装脚本需要 Linux `unshare`、
`mount` 及创建命名空间的权限；隔离失败会直接报错退出。
它验证组件输入输出、Action、断流和心跳，不覆盖 mainboard 动态加载、独立进程通信、
真实 autosim 碰撞或独立进程 kill。完整仿真流程见 [05-simulation.md](05-simulation.md)。

`NavigationComponent` 当前注册并使用仿真插件。真实 VNTPA 部署还需权重和
`RegisterEnginePlugins()` 入口接入，不能仅修改 policy 名称就视为部署成功。

## 5. 安装与开发工具

colcon 安装位置为 `install/autonomy`，包含组件库和 navigation 的 conf、dag、launch、
behavior trees、docs。模型放在 `conf/models` 后随配置目录安装。
`navigation_benchmark`、`navigation_configuration_validator`、
`navigation_component_integration`、`navigation_defm_validator` 使用开发构建目录中的二进制。

```sh
build/autonomy/bin/navigation_benchmark
```

模型导出与数值验证命令见 [06-models.md](06-models.md)。

## 6. Sanitizer

本轮未运行 ASan/TSan，不将旧环境记录计入当前验收。
`NAVIGATION_SANITIZER` 不是当前 CMake 提供的选项。后续需在独立构建目录设置编译和
链接 sanitizer flags，并验证所有相关共享库，避免污染正常构建缓存。
