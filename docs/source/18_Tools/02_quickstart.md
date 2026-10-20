# 2. 快速开始

新 clone 后需要一条最短路径完成依赖安装、编译、launch，并（可选）验证 Bridge 或 autoviz 是否编入。

默认 **`BUILD_GRPC=ON`**、桌面常 **`BUILD_AUTOVIZ=ON`**；板端或 CI 可关 AUTOVIZ 省依赖。BT 插件路径与 launch 目录必须在 shell 里 export，否则 TaskServer 找不到树。

### 编译与 launch

```bash
python3 -m install_deps
python3 tools/python/clang_format_sources.py --check
cmake -G Ninja -B build && ninja -C build
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

### Docker（可选）

```bash
python3 docker/run_autonomy.py -p x86_64
```

### Bridge 冒烟

另开终端，需 bridge 进程已起且 **`BUILD_GRPC=ON`**：

```bash
grpcurl -plaintext -import-path $PWD -proto automsgs/proto/rpcs/system.proto \
  -d '{}' 127.0.0.1:5005 automsgs.rpcs.system.SystemService/GetCapabilities
```

### 可视化

`BUILD_AUTOVIZ=ON` 时编 autoviz，见 [13 · autoviz](../13_Visualization/09_autoviz.md)。无 `VisualizationServer` 进程属预期。

### Bazel（可选）

`./autonomy.sh build -m planning,control`（需已有 CMake 前缀）。
