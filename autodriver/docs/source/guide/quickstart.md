# 快速开始

在 autonomy **仓库根**（含顶层 `CMakeLists.txt`）构建并运行 autodriver：传感采集 + 可选底盘 stub，经 Autolink 发布。

本仓库布局下包根为 `$PWD/autodriver`（其下有 `config/`）。若你把仓库嵌在 `src/autonomy/`，把下文路径改成 `$PWD/src/autonomy/autodriver`。

| 相关 | 链接 |
|---|---|
| 环境 / 嵌入 / 故障 | [使用方式](usage.md) |
| YAML | [配置](configuration.md) |
| 排障 | [FAQ](../faq.md) |

## 1. 环境

```bash
cd /path/to/autonomy          # 仓库根
export AUTODRIVER_PATH=$PWD/autodriver   # 必须是「含 config/ 的包根」，不是 config 本身
export LD_LIBRARY_PATH=$PWD/build/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PATH=$PWD/build/bin${PATH:+:$PATH}
export AUTOLINK_PATH=$PWD/autolink       # 按布局调整
export GLOG_logtostderr=1
```

| 变量 | 是否必须 | 含义 |
|---|---|---|
| `AUTODRIVER_PATH` | 推荐 | 配置根；未设则回退编译期默认 / `AUTODRIVER_DISTRIBUTION_HOME` |
| `LD_LIBRARY_PATH` | 是 | 须含 `libautodriver` 与 Autolink / automsgs |
| `PATH` | 是 | 须含 `autodriver` 二进制 |
| `AUTOLINK_PATH` | 联调时 | Autolink 资源 |
| `AUTOLINK_LAUNCH_PATH` | launch 时 | 设为 `$AUTODRIVER_PATH/launch` |
| `GLOG_logtostderr` | 建议 | `1` 日志打终端 |

依赖概览：CMake ≥ 3.20、C++17、yaml-cpp、Eigen3、GTest（开测试时）、Autolink、automsgs。厂商 SDK 见下表，**未找到则对应 Create→nullptr，不挡编译**。

## 2. 构建

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
# 可选：关掉用不到的 SDK，加快配置
#   -DAUTODRIVER_WITH_REALSENSE=OFF -DAUTODRIVER_WITH_ORBBEC=OFF \
#   -DAUTODRIVER_WITH_RPLIDAR=OFF -DAUTODRIVER_WITH_LIVOX=OFF \
#   -DAUTODRIVER_BUILD_DOCS=OFF
cmake --build build -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" \
  --target autodriver autodriver_main autodriver_demo
```

| 开关 | 默认 | 作用 |
|---|---|---|
| `BUILD_AUTODRIVER` | ON | 顶层是否 `add_subdirectory(autodriver)` |
| `AUTODRIVER_BUILD_TEST` | ON | 单元测试目标 |
| `AUTODRIVER_BUILD_EXAMPLES` | ON | `autodriver_demo` |
| `AUTODRIVER_BUILD_DOCS` | ON | MkDocs `docs` 目标 |
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 → `AUTODRIVER_HAVE_REALSENSE` |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK；未找到→stub |
| `AUTODRIVER_WITH_RPLIDAR` | ON | rplidar_sdk |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox SDK1/2；未找到→stub |

Linux 找到 `libudev` → 自动定义 `AUTODRIVER_HAVE_UDEV`（无单独开关）。

### 厂商 SDK（可选）

在包根执行：

```bash
cd "$AUTODRIVER_PATH"
./scripts/install_rplidar_sdk.sh
./scripts/install_livox_sdk2.sh    # Mid-360 / HAP …
./scripts/install_livox_sdk.sh     # Mid-40 / Horizon …
./scripts/create_udev_rules.sh     # /dev/rplidar（可选）
```

装完后**重新 cmake / build**，确认配置日志出现 `… enabled`。

| 产物 | CMake target | 说明 |
|---|---|---|
| `build/lib/libautodriver.so`（或 `.dylib`） | `autodriver` | 传感 + chassis + 内置 Module |
| `build/bin/autodriver` | `autodriver_main` | 进程入口 |
| `build/bin/autodriver_demo` | `autodriver_demo` | 手写 Config，不读 YAML |

## 3. 配置（跑前必看）

默认文件：`$AUTODRIVER_PATH/config/autodriver_hardware.yaml`。

- 只用的设备设 **`enable: true`**；全 false → `no enabled sensors`。  
- 厂商细项用 `params_file`（相对 `config/`），条目内 `params` 覆盖文件。  
- 相机折叠：`streams` / `point_clouds` / `imu`（见 [配置](configuration.md)）。  
- 底盘联调：`chassis.enable: true`，`backend: stub`（无硬件差分积分）。

最小思路：先开一路串口 IMU 或 stub chassis，确认进程起来再加激光/相机。

## 4. 运行

```bash
autodriver
# 显式指定配置根与文件名：
autodriver "$AUTODRIVER_PATH" autodriver_hardware.yaml
# 厂商专用 YAML 例：
# autodriver "$AUTODRIVER_PATH" camera/orbbec/gemini_330.yaml
```

参数：`[configuration_directory] [configuration_file]`。省略时用 `AUTODRIVER_PATH` + 默认 basename。

启动序（与 `main.cpp` 一致）：

```text
LoadConfig → Publisher → SensorManager → PoseFeeder → ChassisManager → 等信号
Stop：Chassis → PoseFeeder → SensorManager
```

正常时可看到类似 `autodriver running (Ctrl+C to stop)`；`Ctrl+C` 优雅退出。

### Launch

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

`exception_handler: respawn`，`respawn_limit: 3`。硬编码路径时编辑 launch：

```xml
<process_name>autodriver /path/to/autodriver autodriver_hardware.yaml</process_name>
```

## 5. Demo / 测试

```bash
./build/bin/autodriver_demo
ctest --test-dir build -R 'test_' --output-on-failure
# 单测例：
./build/bin/test_config_loader
```

用例表见 [测试](testing.md)。

## 6. 常见问题（速查）

| 现象 | 处理 |
|---|---|
| 找不到 YAML | `AUTODRIVER_PATH` 是否指向含 `config/` 的包根 |
| `no enabled sensors` | 打开 `enable: true` |
| Create / driver nullptr | 装 SDK；看 cmake STATUS；`WITH_*=OFF` 则无该 backend |
| Publisher / Autolink 失败 | `LD_LIBRARY_PATH`、`AUTOLINK_PATH` |
| 串口权限 | 用户进 `dialout`；设备节点路径与 YAML `port` 一致 |
| RPLidar A3 无数据 | `params_file: lidar/slamtec/a3.yaml`（256000） |

更多：[FAQ](../faq.md)。

## 7. 下一步

| 需求 | 文档 |
|---|---|
| 使用 / 嵌入库 | [使用方式](usage.md) |
| 架构 / Registry | [架构](architecture.md) |
| 字节 → 话题 | [数据流](dataflow.md) |
| YAML 全字段 | [配置](configuration.md) |
| Attach / udev | [生命周期](lifecycle.md) |
| backend / Stream | [后端](backends.md) |
| 厂商手册 | [传感器](../sensor/index.md) |
| 测试矩阵 | [测试](testing.md) |
| C++ API | [API](../api/overview.md) |

本地预览文档：

```bash
pip install -r autodriver/docs/requirements.txt
cd autodriver/docs && mkdocs serve
```
