# 快速开始

目标：在 autonomy **仓库根**构建并运行传感进程；可选经 launch / DAG 拉起 **JetAuto 或 L1-W** 底盘（二选一）。

| 本机布局 | 包根（须含 `config/` `dag/`） |
|---|---|
| 仓库即 autonomy 根 | `$PWD/autodriver` |
| 嵌套于 `src/autonomy/` | `$PWD/src/autonomy/autodriver` |

下文以第一种为准；第二种请替换路径。

> 命令按 `CMakeLists.txt` / launch 整理；本机是否可跑取决于 SDK 与构建结果。

| 相关 | 链接 |
|---|---|
| 环境 / 嵌入 / 排障 | [使用方式](usage.md) |
| 底盘 / DAG | [本体 chassis](chassis.md) |
| YAML | [配置](configuration.md) |
| FAQ | [FAQ](../faq.md) |

---

## 1. 环境变量

```bash
cd /path/to/autonomy          # 仓库根（含顶层 CMakeLists.txt）
export AUTODRIVER_PATH=$PWD/autodriver   # 含 config/ dag/ 的包根
export LD_LIBRARY_PATH=$PWD/build/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
export PATH=$PWD/build/bin${PATH:+:$PATH}
export AUTOLINK_PATH=$PWD/autolink
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=$PWD/build/lib   # libautodriver_jetauto.so / _l1w.so
export GLOG_logtostderr=1
```

| 变量 | 是否必须 | 含义 |
|---|---|---|
| `AUTODRIVER_PATH` | 推荐 | 包根（含 `config/`）；未设则回退编译默认 |
| `LD_LIBRARY_PATH` | 是 | `libautodriver` + Autolink / automsgs |
| `PATH` | 是 | `autodriver` / `mainboard` |
| `AUTOLINK_LAUNCH_PATH` | launch | `$AUTODRIVER_PATH/launch` |
| `AUTOLINK_DAG_PATH` | 底盘 DAG | `$AUTODRIVER_PATH/dag` |
| `AUTOLINK_LIB_PATH` | 底盘 Component | 含 `libautodriver_*.so` 的 lib 目录 |
| `GLOG_logtostderr` | 建议 | `1` → 日志到终端 |

**依赖**：CMake ≥ 3.20、C++17、yaml-cpp、Eigen3、Autolink、automsgs；启用测试时需 GTest。厂商 SDK 见 §2；**未找到时对应 Create 返回 `nullptr`，不阻碍编译与链接**。

---

## 2. 构建

```bash
cmake -S . -B build -DBUILD_AUTODRIVER=ON
# 可选：关闭未用 SDK，加快配置
#   -DAUTODRIVER_WITH_REALSENSE=OFF -DAUTODRIVER_WITH_ORBBEC=OFF \
#   -DAUTODRIVER_WITH_RPLIDAR=OFF -DAUTODRIVER_WITH_LIVOX=OFF \
#   -DAUTODRIVER_BUILD_DOCS=OFF
cmake --build build -j"$(nproc 2>/dev/null || sysctl -n hw.ncpu)" \
  --target autodriver autodriver_main autodriver_demo
```

### 2.1 CMake 开关

| 开关 | 默认 | 作用 |
|---|---|---|
| `BUILD_AUTODRIVER` | ON | 顶层是否 `add_subdirectory(autodriver)` |
| `AUTODRIVER_BUILD_TEST` | ON | 单元测试目标 |
| `AUTODRIVER_BUILD_EXAMPLES` | ON | `autodriver_demo` |
| `AUTODRIVER_BUILD_DOCS` | ON | MkDocs `docs` 目标 |
| `AUTODRIVER_WITH_REALSENSE` | ON | librealsense2 → `AUTODRIVER_HAVE_REALSENSE` |
| `AUTODRIVER_WITH_ORBBEC` | ON | OrbbecSDK；未找到时为 stub |
| `AUTODRIVER_WITH_RPLIDAR` | ON | rplidar_sdk |
| `AUTODRIVER_WITH_LIVOX` | ON | Livox SDK1/2；未找到时为 stub |
| `GenisomL1w_ROOT` | — | 可选；智身 ZSL-1W HighLevel SDK。未设时 `l1w` 仍可编（simulate） |

Linux 找到 `libudev` 时自动定义 `AUTODRIVER_HAVE_UDEV`（无单独开关）。

### 2.2 厂商 SDK（按需）

在包根执行后，须**重新 cmake / build**，并确认配置日志出现对应 `… enabled`：

```bash
cd "$AUTODRIVER_PATH"
./scripts/install_rplidar_sdk.sh
./scripts/install_livox_sdk2.sh    # Mid-360 / HAP …
./scripts/install_livox_sdk.sh     # Mid-40 / Horizon …
./scripts/create_udev_rules.sh     # 可选：/dev/rplidar
```

### 2.3 产物

| 产物 | CMake target | 说明 |
|---|---|---|
| `libautodriver.so` | `autodriver` | 传感 + stub chassis + Module |
| `libautodriver_jetauto.so` | `autodriver_jetauto` | JetAuto Component |
| `libautodriver_l1w.so` | `autodriver_l1w` | L1-W Component |
| `bin/autodriver` | `autodriver_main` | 传感进程 |
| `bin/autodriver_demo` | `autodriver_demo` | 代码构造 Config |

---

## 3. 运行前配置

默认文件：`$AUTODRIVER_PATH/config/autodriver_hardware.yaml`。

| 要点 | 说明 |
|---|---|
| `enable` | 仅将实际使用的设备设为 `true`；全部为 false 时日志出现 `no enabled sensors` |
| `params_file` | 相对 `config/`；条目内 `params` 覆盖文件同名键 |
| 相机 | 推荐折叠 `streams` / `point_clouds` / `imu`（见 [配置](configuration.md)） |
| 底盘 stub | 进程内：`chassis.enable: true`，`backend: stub` |
| 实机底盘 | **推荐** launch / DAG（JetAuto XOR L1-W）；主 YAML 保持 `chassis.enable: false` |

建议：先启用一路串口 IMU 或 stub，确认进程正常后再加激光/相机/实机底盘。详见 [本体](chassis.md)。

---

## 4. 运行进程

```bash
autodriver
# 显式指定配置根与文件名：
autodriver "$AUTODRIVER_PATH" autodriver_hardware.yaml
# 厂商专用 YAML 例：
# autodriver "$AUTODRIVER_PATH" camera/orbbec/gemini_330.yaml
```

| 参数 | 含义 |
|---|---|
| `argv[1]` | `configuration_directory`（包根）；省略则用 `AUTODRIVER_PATH` / 编译默认 |
| `argv[2]` | 配置文件名（相对 `config/` 或绝对路径）；省略则默认 basename |

启动顺序（与 `main.cpp` 一致）：

```text
LoadConfig → Publisher → SensorManager → PoseFeeder → ChassisManager → 等待信号
Stop：Chassis → PoseFeeder → SensorManager
```

正常运行时可看到类似 `autodriver running (Ctrl+C to stop)` 的日志；`Ctrl+C` 优雅退出。

### 4.1 Launch + 底盘 DAG（二选一）

```bash
export AUTOLINK_LAUNCH_PATH=$AUTODRIVER_PATH/launch
export AUTOLINK_DAG_PATH=$AUTODRIVER_PATH/dag
export AUTOLINK_LIB_PATH=$PWD/build/lib
autolink launch start autodriver.launch
autolink launch list
autolink launch stop autodriver.launch
```

`autodriver.launch` 含：

| Module | 类型 | 说明 |
|---|---|---|
| `autodriver` | binary | 传感 / joy |
| `chassis_l1w` **或** `chassis_jetauto` | `dag_conf` | **只启用一个**（文件内注释切换） |

单独起底盘：`mainboard -d $AUTOLINK_DAG_PATH/chassis_l1w.dag`。详见 [本体](chassis.md)。

---

## 5. Demo 与测试

```bash
./build/bin/autodriver_demo
ctest --test-dir build -R 'test_' --output-on-failure
./build/bin/test_config_loader   # 单测例
```

用例表见 [测试](testing.md)。

---

## 6. 常见问题（简明）

| 现象 | 处理 |
|---|---|
| 找不到 YAML | 确认 `AUTODRIVER_PATH` 指向含 `config/` 的包根 |
| `no enabled sensors` | 将所需设备设为 `enable: true` |
| Create / driver 为 `nullptr` | 安装对应 SDK；查看 CMake STATUS；`WITH_*=OFF` 时无该 backend |
| Publisher / Autolink 失败 | 检查 `LD_LIBRARY_PATH`、`AUTOLINK_PATH` |
| 串口权限不足 | 将用户加入 `dialout`；确认设备节点与 YAML `port` 一致 |
| RPLidar A3 无数据 | 使用 `params_file: lidar/slamtec/a3.yaml`（波特率 256000） |
| mainboard 找不到 `.dag` / `.so` | 设 `AUTOLINK_DAG_PATH`、`AUTOLINK_LIB_PATH` |
| L1-W 无 SDK | `-DGenisomL1w_ROOT=` 或 YAML `simulate: true` |

更多见 [FAQ](../faq.md)。

---

## 7. 后续阅读

| 需求 | 文档 |
|---|---|
| 使用 / 嵌入 | [使用方式](usage.md) |
| 底盘 / DAG | [本体](chassis.md) |
| YAML | [配置](configuration.md) |
| 架构 / backend | [架构](architecture.md) · [后端](backends.md) |
| 传感器 | [传感器](../sensor/index.md) |
| FAQ / API | [FAQ](../faq.md) · [API](../api/overview.md) |

本地预览文档：

```bash
pip install -r autodriver/docs/requirements.txt
cd autodriver/docs && mkdocs serve
```
