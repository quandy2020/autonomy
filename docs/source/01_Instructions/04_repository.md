# 4. 仓库与生态

**源码根**：含顶层 `CMakeLists.txt` 的目录。路径均相对源码根。版本见 `version.json`（**0.2.0**）。分层见 [§3](03_system_architecture.md)；安装见 [02 Installation](../02_Installation/00_guide.md)。

## 4.1 顶层分区

| 分区 | 目录 | 作用 |
|------|------|------|
| 平台 | `autolink/`、`automsgs/` | IPC 与消息 / RPC |
| 业务域 | `autonomy/*` | SLAM、感知、地图/规划/控制、任务、管理、Bridge |
| 硬件 | `autodriver/` | 传感器 / 底盘（`BUILD_AUTODRIVER`） |
| 工具 | `autoviz/`、`autosim/`、`autonomy/orbisview/` | 可视化、仿真、Web HMI |
| 基建 | `cmake/`、`docker/`、`scripts/`、`docs/`、`tools/` | 依赖、容器、环境、文档 |
| 部署 | `ansible/` | 装机（可选） |

## 4.2 目录树

```text
autonomy/ # 源码根
├── CMakeLists.txt / CMakePresets.json / version.json
├── autolink/ # 通信运行时
├── automsgs/ # 消息与 RPC schema
├── autonomy/ # 业务域（AUTONOMY_BUILD_*）
│ ├── common/ transform/ map/ planning/ control/ task/
│ ├── localization/ perception/ prediction/
│ ├── sensor/ vehicle/ audio/ system/ bridge/
│ ├── orbisview/ visualization/ tools/
├── autodriver/ # HAL
├── automanip/ # 机械臂 + bolza
├── autoviz/ autosim/
├── config/ data/ cmake/ docker/ docs/ scripts/ tools/ ansible/
```

| 要改… | 先看 |
|---------|------|
| 通信 / launch | `autolink/` · [03](../03_Communication/00_guide.md) |
| 消息 / RPC | `automsgs/` |
| 导航 / BT | `autonomy/planning|control|task/` |
| SLAM / 感知 | `autonomy/localization|perception/` |
| 监控 / OTA | `autonomy/system/` |
| 远程发令 | `autonomy/bridge/` |
| 相机雷达底盘 | `autodriver/` |

## 4.3 构建产物

默认 `源码根/build/`（板端 build **必须本地盘**）。无单一 `libautonomy.so`。

| 类型 | 路径 | 说明 |
|------|------|------|
| 域库 | `build/lib/libautonomy_*.so` | 如 `libautonomy_common.so` |
| 平台库 | `build/lib/libautolink.so`、`libautomsgs*.so` | — |
| 可执行 | `build/bin/` | `autolink`、`autonomy.planning`、`autonomy.task`… |
| 插件 | `build/lib/` | `*_planner*.so`、`autonomy_behavior_tree_*.so` |

```bash
source scripts/setup_environment.bash
# install 后：source /usr/local/share/autonomy/setup.bash
# build 非默认路径：export AUTONOMY_BUILD_DIR=/你的/build
```

## 4.4 配置与 launch

| 用途 | 位置 |
|------|------|
| 系统 / 监控 / OTA | `autonomy/system/conf/*.pb.txt` |
| 全栈启动 | `autonomy/system/launch/autonomy.launch` |
| 仅导航 | `autonomy/task/launch/task.launch` |
| BT XML | `autonomy/task/conf/behavior_tree/` |
| 各域 conf | `autonomy/<域>/conf/` |
| 驱动 | `autodriver/config/`、`launch/` |

跨进程须一致：`global_frame` / `robot_base_frame`、默认 `planner_id` / `controller_id`。运行见 [04 Running](../04_Running/00_guide.md)。

## 4.5 编译开关

| 层级 | 前缀 | 例子 |
|------|------|------|
| 业务域 | `AUTONOMY_BUILD_*` | `AUTONOMY_BUILD_PLANNING` |
| 产品 | `BUILD_*` | `BUILD_AUTODRIVER`、`BUILD_GRPC` |

| 常用 | 默认约 | 说明 |
|------|--------|------|
| `AUTONOMY_BUILD_<域>` | 多数 ON | 关后不进构建；依赖不满足则 FATAL |
| `BUILD_AUTODRIVER` | ON | 传感器 / 底盘 |
| `BUILD_GRPC` | ON | Bridge |
| `BUILD_AUTOVIZ` / `BUILD_AUTOSIM` / `BUILD_ORBISVIEW` | ON / OFF / ON | — |
| `BUILD_ONNXRUNTIME` | ON | 感知推理 |
| `BUILD_TEST` / `BUILD_DOCS` | OFF / ON | — |

完整 cmake 示例：[Installation §6.2](../02_Installation/06_build.md)。

## 4.6 生态

| 方式 | 组件 | 文档 |
|------|------|------|
| 多进程入口 | `autolink_launch` | [04](../04_Running/00_guide.md) |
| IPC | Channel / Service / Action | [03](../03_Communication/00_guide.md) |
| 消息 | `automsgs` | [14](../14_Commsgs/00_guide.md) |
| 远程 API | `autonomy.bridge` | [15](../15_Bridge/00_guide.md) |
| ROS 2 | 可选包装 | [Running · ROS 2](../04_Running/06_ros2_integration.md) |
| 可视化 / 仿真 | Autoviz、OrbisView、Autosim | [13](../13_Visualization/index.rst) · [12](../12_Simulation/index.rst) |

## 4.7 日常命令

```bash
source scripts/setup_environment.bash
python3 scripts/install_dependencies.py --skip-installed
python3 scripts/install_dependencies.py --profile board --skip-installed
python3 tools/python/clang_format_sources.py autonomy/planning
cmake --build build -j$(nproc) --target autonomy.planning
```
