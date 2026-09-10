# 快速开始

在 autonomy **仓库根目录**（含顶层 `CMakeLists.txt`）构建并运行。包路径：`src/autonomy/autodriver`。

## 1. 环境

```bash
cd /path/to/autonomy
export AUTODRIVER_PATH=$PWD/src/autonomy/autodriver   # 须含 config/
export LD_LIBRARY_PATH=$PWD/build/lib:$LD_LIBRARY_PATH
export PATH=$PWD/build/bin:$PATH
# 联调 Autolink：
export AUTOLINK_PATH=$PWD/autolink   # 按仓库布局调整
export GLOG_logtostderr=1
```

## 2. 构建

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
cmake --build build -j"$(nproc)" --target autodriver autodriver_main autodriver_demo
```

| 开关 | 默认 | 作用 |
|---|---|---|
| `BUILD_AUTODRIVER` | ON | 顶层是否编译 |
| `AUTODRIVER_BUILD_TEST` | ON | 单元测试 |
| `AUTODRIVER_BUILD_EXAMPLES` | ON | `autodriver_demo` |
| `AUTODRIVER_BUILD_DOCS` | ON | MkDocs `docs` |
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK；未找到→stub |
| `AUTODRIVER_WITH_RPLIDAR` | ON | rplidar_sdk |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox SDK1/2；未找到→stub |

Linux 找到 `libudev` → `AUTODRIVER_HAVE_UDEV`（无单独开关）。

```bash
cd "$AUTODRIVER_PATH"
./scripts/install_rplidar_sdk.sh
./scripts/install_livox_sdk2.sh   # Mid-360 / HAP …
./scripts/install_livox_sdk.sh    # Mid-40 / Horizon …
```

| 产物 | 说明 |
|---|---|
| `build/lib/libautodriver.so` | 库（target `autodriver`） |
| `build/bin/autodriver` | 进程（target `autodriver_main`） |
| `build/bin/autodriver_demo` | 不读 YAML 的最小例 |

## 3. 运行

默认 `config/autodriver_hardware.yaml`；厂商细项用 `params_file`。

```bash
autodriver
autodriver "$AUTODRIVER_PATH" autodriver_hardware.yaml
```

参数：`[configuration_directory] [configuration_file]`。只用的设备设 `enable: true`。

### Launch

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

`exception_handler: respawn`（最多 3 次）。自定义：

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

## 4. Demo / 测试

```bash
./build/bin/autodriver_demo
ctest --test-dir build -R test_ --output-on-failure
```

## 5. 下一步

| 需求 | 文档 |
|---|---|
| 使用 / 嵌入 | [使用方式](usage.md) |
| 架构 | [架构](architecture.md) |
| 数据流 | [数据流](dataflow.md) |
| 厂商 | [传感器](../sensor/index.md) |
| YAML | [配置](configuration.md) |
| 测试 | [测试](testing.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| backend | [后端](backends.md) |
| C++ | [API](../api/overview.md) |
| 排障 | [FAQ](../faq.md) |
