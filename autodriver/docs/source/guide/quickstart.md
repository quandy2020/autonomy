# 快速开始

在 autonomy **仓库根目录**（含顶层 `CMakeLists.txt`）构建并运行 Autodriver。

## 1. 环境

```bash
cd /path/to/autonomy
export AUTODRIVER_PATH=$PWD/autodriver   # 须含 config/autodriver_hardware.yaml
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH
```

与 Autolink 联调时通常还需：

```bash
export AUTOLINK_PATH=$PWD/autolink
```

调试日志：`export GLOG_logtostderr=1`。

## 2. 构建

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j8 --target autodriver autodriver_main autodriver_demo
```

| 开关 | 默认 | 作用 |
|---|---|---|
| `BUILD_AUTODRIVER` | ON | 顶层是否编译 autodriver |
| `AUTODRIVER_BUILD_TEST` | ON | 单元测试（`ctest`） |
| `AUTODRIVER_BUILD_EXAMPLES` | ON | `autodriver_demo` |
| `AUTODRIVER_BUILD_DOCS` | ON | MkDocs 目标 `docs` |
| `AUTODRIVER_WITH_REALSENSE` | ON | RealSense 驱动（需 librealsense2） |
| `AUTODRIVER_WITH_ORBBEC` | ON | Orbbec 驱动（需 OrbbecSDK；未找到则 stub） |

Linux 上若找到 `libudev`，会自动启用热插拔（宏 `AUTODRIVER_HAVE_UDEV`），无单独 CMake 开关。

产物：

- `build/lib/libautodriver.so` — 核心库（CMake target：`autodriver`）
- `build/bin/autodriver` — 读 YAML 的进程入口（CMake target：`autodriver_main`，与库目标同名冲突故加 `_main`）
- `build/bin/autodriver_demo` — 不读 YAML 的最小示例

## 3. 运行

### 直接运行

默认读取 `config/autodriver_hardware.yaml`：

```bash
autodriver
```

编辑 YAML：将要用的传感器设为 `enable: true`。当前示例默认打开 RealSense D455 相关条目。

参数：

1. **configuration_directory**（可选）：配置根目录（含 `config/`）
2. **configuration_file**（可选）：默认 `autodriver_hardware.yaml`

```bash
autodriver /path/to/autodriver autodriver_hardware.yaml
```

### 使用 autolink launch（推荐）

```bash
export AUTODRIVER_PATH=$PWD/autodriver
export AUTOLINK_LAUNCH_PATH=$PWD/autodriver/launch
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH

autolink launch start autodriver.launch
```

停止 / 列表：

```bash
autolink launch stop autodriver.launch
autolink launch list
```

launch 文件：`launch/autodriver.launch`。  
默认 `exception_handler: respawn`，崩溃后最多重启 3 次。自定义配置目录可在 `<process_name>` 中传参：

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

## 4. Demo

不加载 YAML，手写两条 attach-only lidar 配置：

```bash
./build/bin/autodriver_demo
```

## 5. 测试

```bash
ctest --test-dir build -R 'test_(config|config_loader|manager|hub|realsense|nmea|wit)' --output-on-failure
```

## 6. 下一步

| 需求 | 文档 |
|---|---|
| 配置字段 | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| serial / CAN / RealSense | [后端](backends.md) |
| C++ API | [API 概览](../api/overview.md) |
| 路径与权限 | [FAQ](../faq.md) |
