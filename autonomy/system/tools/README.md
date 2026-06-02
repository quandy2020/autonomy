# system 离线导航工具

本目录提供 **`autonomy_nav_test`**：在**不依赖 ROS** 的环境下，跑通 `system::Autonomy` 完整导航链路（Map / Planner / Controller / Task / BT），并用差速模型仿真里程计与 TF 反馈。

对应源码：`nav_test.cpp`。

## 与其它工具的区别

| 工具 | 路径 | 覆盖范围 |
|------|------|----------|
| **`autonomy_nav_test`** | `system/tools/` | **完整栈**：`Autonomy` → BT 或直驱 → 规划 + 控制 + 恢复行为 |
| `autonomy_planning_test` | `planning/tools/` | 仅全局规划，输出 PNG/MP4 |
| `autonomy_controller_test` | `control/tools/` | 规划 + 控制器闭环仿真，**不跑 BT** |
| `autonomy.system.main` | `system/main.cpp` | 仅 `Start()` + `Configure()`，**不发导航目标** |

需要验证行为树、`NavigateToPose`、恢复节点时，请用本工具；仅需对比控制器跟踪性能时用 `autonomy_controller_test`。

## 功能概览

1. 通过 `CreateOptions` + `CreateAutonomy` 加载 `config/autonomy.lua`（及子配置）。
2. `Start()` 启动 MapServer、PlannerServer、ControllerServer、TransformServer。
3. `Configure()` 附着 BT 导航引擎（需 BT 插件 `.so`）。
4. 设置初始位姿：向 `ControllerServer` 注入里程计，并向 `transform::Buffer` 发布 `global_frame → base_frame` TF。
5. 后台线程按 `cmd_vel` 积分仿真位姿，持续刷新里程计 / TF。
6. 主线程调用 `Autonomy::NavigateToPose()`，直至成功、失败或超时。

### 导航模式

| `--use_bt` | 行为 |
|------------|------|
| `true`（默认） | 行为树导航，与 ROS 侧 `autonomy.use_bt_navigation:=true` 一致 |
| `false` | 直驱：`ComputePathToPose` + `TickFollowPath` 循环 |

## 构建

在工程 `src/autonomy` 目录下，需开启 `BUILD_TOOLS` 与 `BUILD_TASKS`：

```bash
mkdir -p build && cd build
cmake .. -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
ninja autonomy_nav_test
```

可执行文件：`build/bin/autonomy_nav_test`。

BT 行为树插件需已编译（默认随 `libautonomy` 一同构建，位于 `build/lib/`）。

## 环境变量

| 变量 | 说明 |
|------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索路径（冒号分隔）。未设置时尝试 build/install 的 `lib` 目录 |
| `GLOG_logtostderr=1` | 建议开启，便于查看规划 / BT / 控制日志 |

示例：

```bash
export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
export GLOG_logtostderr=1
```

## 运行示例

在 **Docker / Linux** 中使用容器内路径（不要用 macOS 的 `/Users/...`）。

### BT 单点导航（推荐）

```bash
./bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --start_yaw=0 \
  --goal_x=5 --goal_y=5 --goal_yaw=0 \
  --use_bt=true \
  --timeout_sec=120
```

成功时终端输出 `Navigation succeeded.` 及 `Last path poses: N`。

### 直驱模式（不走 BT）

```bash
./bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 \
  --goal_x=5 --goal_y=5 \
  --use_bt=false
```

### 与 `config/data/map.yaml` 配合

默认地图 `config/data/map.pgm` 为世界坐标系下的静态栅格。起终点应选在**自由空间**内（例如 `--start_x=1 --start_y=1 --goal_x=5 --goal_y=5`）。

若规划失败，可先用 `autonomy_planning_test` 在同一地图上确认起终点可达：

```bash
./bin/autonomy_planning_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/planning_vis \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5
```

### 查看全部命令行参数

```bash
./bin/autonomy_nav_test --help
```

## 命令行参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `configuration_directory` | string | （必填） | 配置根目录，见 `common/gflags` |
| `configuration_basename` | string | `autonomy.lua` | 主配置 Lua 文件名 |
| `start_x` / `start_y` / `start_yaw` | double | `1.0` / `1.0` / `0.0` | 仿真初始位姿 [m, rad] |
| `goal_x` / `goal_y` / `goal_yaw` | double | `5.0` / `5.0` / `0.0` | 目标位姿 [m, rad] |
| `use_bt` | bool | `true` | 是否走行为树导航 |
| `timeout_sec` | double | `120.0` | 导航超时 [s] |
| `sim_dt` | double | `0.1` | 里程计仿真步长 [s] |
| `global_frame` | string | `map` | 与 `config/common.lua` 中 `AUTONOMY_COMMON.global_frame` 一致 |
| `base_frame` | string | `base_link` | 与 `robot_base_frame` 一致 |

## 数据流

```text
autonomy_nav_test
  ├─ CreateAutonomy → Start → Configure
  ├─ [sim thread]  cmd_vel → 积分 → UpdateOdometry + map→base_link TF
  └─ NavigateToPose
        ├─ use_bt=true  → Task::StartNavigateToPose → BT (Plan/Follow/Recovery…)
        └─ use_bt=false → NavigateDirectToPose (Plan + Follow loop)
```

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `Autonomy not ready` | Navigator/BT 配置失败 | 检查 `AUTONOMY_BT_PLUGIN_PATH`；确认 `config/navigator/navigator.lua` 与 BT XML 存在 |
| BT 插件 load 失败 | `.so` 不在搜索路径 | `export AUTONOMY_BT_PLUGIN_PATH=<build或install>/lib` |
| `NavigateDirectToPose: no robot pose` | 直驱模式无里程计 | 本工具已注入初始 odom；若仍失败，检查 Controller 是否 `Start()` |
| 规划失败 / 路径过短 | 目标在障碍内或越界 | 调整起终点；参考 `autonomy_planning_test` 可视化 |
| 超时 | 控制器未收敛或目标过远 | 增大 `--timeout_sec`；减小目标距离或换控制器 |
| `TransformAvailable` 失败 | TF 未发布 | 确认 `--global_frame` / `--base_frame` 与 `tasks.lua` 一致 |

## Docker 快速上手

```bash
# 宿主机
python3 src/autonomy/docker/run_autonomy.py -p x86_64

# 容器内
cd /workspace/autonomy
cmake -S src/autonomy -B build -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
cmake --build build -j$(nproc)

export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
export GLOG_logtostderr=1

./build/bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5
```

## 相关文件

| 文件 | 说明 |
|------|------|
| `nav_test.cpp` | 本工具实现 |
| `../autonomy.hpp` / `../autonomy.cpp` | `system::Autonomy` 核心 |
| `../../config/autonomy.lua` | 主配置入口 |
| `../../config/navigator/README.md` | navigator 配置说明 |
| `../../config/navigator/navigator.lua` | Navigator / BT 配置 |
| `../../config/navigator/behavior_tree/*.xml` | 行为树定义 |
| `../control/tools/README.md` | 控制器离线评测文档 |
| `../planning/tools/README.md` | 规划离线可视化文档 |
