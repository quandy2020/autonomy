# control 离线可视化工具

本目录提供 **`autonomy_controller_test`**：在不启动完整导航栈的情况下，按 `config/control/controller.lua` 与 `config/planner/planner.lua` 规划全局路径，对每个已配置控制器做闭环仿真，将**实际跟踪轨迹**渲染为 PNG，并生成 MP4/GIF 动画。

对应源码：`controller_server_test.cpp`。

## 功能概览

1. 加载 `planner/planner.lua` 的 `AUTONOMY_PLANNER`，用 `PlannerServer` 规划 `start → goal` 全局路径。
2. 加载 `control/controller.lua` 的 `AUTONOMY_CONTROLLER`（各控制器参数、`goal_checker` / `progress_checker`）。
3. 复用规划器 **global costmap**（与 `TaskScheduler` 单进程模式一致：`controller.lua` 中 `costmap.enabled = false` 时由 `SetSharedCostmap` 注入）。
4. 对每个控制器：`BeginFollowPath` → 循环 `TickFollowPath`，用差速模型积分 `cmd_vel` 并写入 `OdomSmoother` + `map→base_link` TF。
5. 输出全局参考路径与实时执行轨迹**同图叠加**（PNG / MP4 / GIF）、`run_summary.txt`。

### 自动检测的控制器

`controller.lua` 中存在对应配置块时，默认依次测试（可用 `--controllers` 覆盖）：

| 控制器 ID | 配置块 | 说明 |
|-----------|--------|------|
| `graceful_controller` | `graceful_controller` | 平滑跟踪 / Pure Pursuit 风格 |
| `nmpc_controller` | `nmpc_controller` | 非线性 MPC（Ipopt） |
| `tdmpc_controller` | `tdmpc_controller` | 拓扑驱动 MPCC |
| `mppi_controller` | `mppi_controller` | MPPI 采样控制（计算量大，可用 `--skip_mppi` 跳过） |

`ControllerServer` 内置插件 ID 亦包含 `FollowPath`（映射到 `graceful_controller`），本工具默认不单独跑该别名。

## 构建

在工程 `src/autonomy` 目录下：

```bash
mkdir -p build && cd build
cmake ..
ninja autonomy_controller_test
```

可执行文件：`build/bin/autonomy_controller_test`。

## 运行示例

在 **Docker / Linux** 中请使用容器内路径（不要用 macOS 的 `/Users/...`）：

```bash
./bin/autonomy_controller_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/controller_vis \
  --map_yaml=/workspace/autonomy/src/autonomy/config/data/map.yaml \
  --max_sim_steps=50000 \
  --start_x=5 --start_y=6 \
  --goal_x=10 --goal_y=10 \
  --fps=10 \
  --controllers=graceful_controller \
  --output_format=both
```

未写 `--map_yaml` 时同样会加载 `config/data/map.yaml`。`--configuration_directory` 可省略。

MPPI 较慢，批量评测可加 `--skip_mppi=true`。

### 默认地图（`config/data/map.pgm`）

未指定 `--map_yaml` 且未加 `--use_synthetic_map` 时，自动加载 `config/data/map.yaml`（及同目录 `map.pgm`）。`map.pgm` 为 400×400 格、`resolution: 1.0` 时世界范围约 0–400 m，默认起终点 `(30,30)→(370,370)` 有效。若 `resolution` 误为 `0.1`（仅约 40×40 m），会报目标越界；日志中会打印 `Map world bounds`。

### 仅测试部分控制器

```bash
./build/bin/autonomy_controller_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/controller_vis \
  --controllers=graceful_controller,nmpc_controller \
  --skip_mppi=true
```

### 合成小地图（快速调试）

```bash
./build/bin/autonomy_controller_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/controller_vis \
  --use_synthetic_map=true \
  --add_demo_obstacles=false \
  --start_x=0.5 --start_y=0.5 \
  --goal_x=2.0 --goal_y=2.0 \
  --controllers=graceful_controller \
  --max_sim_steps=400
```

`--add_demo_obstacles=true` 时中央有障碍块，小地图上终点易落在障碍内导致**规划失败**。

### 跳过 MPPI（离线批量较慢）

```bash
./build/bin/autonomy_controller_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/controller_vis \
  --skip_mppi=true
```

### 查看全部命令行参数

```bash
./build/bin/autonomy_controller_test --help
```

## 命令行测试

`controller_test_cli.sh` 对工具做端到端冒烟（help、合成地图 + graceful、自动检测控制器）：

```bash
cd build && ninja autonomy_controller_test

./autonomy/control/tools/controller_test_cli.sh

# 或指定构建/配置目录
AUTONOMY_BUILD_DIR=/workspace/autonomy/src/autonomy/build \
AUTONOMY_CONFIG_DIR=/workspace/autonomy/src/autonomy/config \
./autonomy/control/tools/controller_test_cli.sh
```

启用 `BUILD_TEST` 时纳入 CTest：

```bash
cd build
ctest -R autonomy_controller_test_cli --output-on-failure
```

## 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `configuration_directory` | string | `""` | 配置根目录（`common/gflags`） |
| `output_dir` | string | `/tmp/autonomy_controller_vis` | PNG / MP4 / GIF / 摘要输出目录 |
| `output_format` | string | `mp4` | `mp4` / `gif` / `both`（不区分大小写） |
| `map_yaml` | string | `""` | 地图 YAML；空则 `config/data/map.yaml` |
| `use_synthetic_map` | bool | `false` | 不加载 PGM，使用合成 costmap |
| `planner_id` | string | `""` | 规划器 ID；空则用 `planner.lua` 默认 |
| `controllers` | string | `""` | 逗号分隔控制器 ID；空则按 lua 块自动检测 |
| `start_x` / `start_y` | double | `30.0` / `30.0` | 起点 [m] |
| `goal_x` / `goal_y` | double | `370.0` / `370.0` | 终点 [m] |
| `fps` | double | `10.0` | 视频/GIF 帧率 |
| `hold_frames` | int | `15` | 每个控制器片段末尾停留帧数 |
| `animate_poses_per_frame` | int | `3` | 动画每帧新增轨迹点数（≥1） |
| `max_sim_steps` | int | `2500` | 每个控制器最大控制 tick 数 |
| `per_controller_video` | bool | `true` | 是否为每个控制器单独写 MP4/GIF |
| `combined_video_name` | string | `controller_all.mp4` | 合并 MP4 文件名 |
| `combined_gif_name` | string | `controller_all.gif` | 合并 GIF 文件名 |
| `save_reference_png` | bool | `true` | 是否保存规划参考路径 PNG |
| `add_demo_obstacles` | bool | `true` | 合成地图中央障碍块 |
| `skip_mppi` | bool | `false` | 跳过 `mppi_controller` |
| `write_run_summary` | bool | `true` | 写入 `run_summary.txt` |
| `path_pose_spacing` | double | `0.05` | 参考路径重采样最大间距 [m]（0 关闭） |
| `min_reference_path_poses` | int | `10` | 过少路径点时打印 WARN |

## 输出文件

在 `--output_dir` 下生成：

| 路径 | 说明 |
|------|------|
| `reference_path.png` | 规划器原始全局路径（`ComputePathToPose` 输出，未加密） |
| `{controller}_tracking.png` | **规划全局路径 + 执行轨迹** 同图（青=规划路径，红=已走，黄点=当前位姿） |
| `{controller}.mp4` | 单控制器动画：规划路径全程可见，执行轨迹逐帧增长（`per_controller_video=true`） |
| `{controller}.gif` | 单控制器 GIF（`output_format` 含 `gif` 且系统有 `ffmpeg`） |
| `controller_all.mp4` | 所有控制器片段顺序拼接 |
| `controller_all.gif` | 合并 GIF |
| `run_summary.txt` | 各控制器成功/失败与轨迹点数摘要 |
| `frames/` | 渲染中间帧 PNG |

视频编码为 OpenCV `mp4v`。帧上叠加控制器 ID、状态行（`OK:` / `FAIL:`）及图例（planner global path / executed trajectory / robot pose）。青线为**规划器全局路径**（非控制器用的加密参考线）；闭环控制实际跟踪的是加密后的 `reference_path`（见日志 `controller reference (densified)`）。

GIF 依赖 **`ffmpeg`**（palette 优化）；未安装时 MP4 仍可用，GIF 会打 WARN 并跳过。

## 退出码

| 码 | 含义 |
|----|------|
| `0` | 至少一个控制器到达目标（`IsGoalReached`） |
| `2` | 全部控制器未到达目标 |
| `1` | 致命错误（costmap 为空、地图加载失败、全局规划失败等） |

## 常见问题

### 日志显示 `Planned path with 2 poses` 但 `Found valid path of size 84`

`planner.lua` 中 `path_simplify_epsilon = 0.02` 会在 `PostProcessPath` 里做 Douglas-Peucker 简化，短距离直线常被收成**仅起点+终点**两点。`graceful_controller` 的 `initial_rotation` 会长时间原地旋转，且 `Finished: 0/1 controllers reached goal`。

本工具已强制 `path_simplify_epsilon = 0`，并默认按 `--path_pose_spacing=0.05` 重采样参考路径（点数不足时自动减半间距）。

**跟踪参考路径的前提**：`ControllerServer::BeginFollowPath` 会把全局参考路径交给控制器的 `SetPlan`；`graceful_controller` 通过 `PathHandler::TransformGlobalPlan` 在 `base_link` 下裁剪并跟踪该路径。**切勿**在每个控制周期再次 `SetPlan`（旧版 `UpdateGlobalPath` 曾每 tick 调用），否则会每帧重置 `do_initial_rotation_`，表现为 `linear=0`、`angular=±v_angular_min_in_place`（约 ±0.1）原地摆头而不沿路径前进。该问题已在 `controller_server.cpp` 中修复。

重新编译后日志应类似：

```text
Reference path: planner returned 84 poses, using 84 poses for controllers
BeginFollowPath with 84 poses using controller 'graceful_controller'.
```

若仍只有个位数路径点，请确认已 `ninja autonomy_controller_test` 且日志含 `Reference path:` 行。可减小 `--path_pose_spacing` 或增大 `--min_reference_path_poses`。

### 末端 `linear=0` 且 `angular` 在 ±0.1 之间来回抖

机器人已走完全程（`executed` 轨迹几百个点）但未判到达时，多为 **末端对准** 阶段：`graceful_controller` 曾把目标航向的绝对 yaw 直接交给 `RotateToTarget`，在目标附近会正负交替输出约 ±0.1 rad/s。已改为使用 **当前航向与目标航向之差**，并在误差小于 `initial_rotation_tolerance` 时输出零速度。

若日志出现 `Resulting plan has 0 poses in it.` 后 `controller failed`：多为路径剪枝后 `PathHandler` 在机器人略偏离路径时裁出空局部计划。已在 `path_handler.cpp` 中保证至少变换最近点/终点，并保留全局路径最后一个目标点直至控制结束。

短路径建议 `--max_sim_steps=800`（工具默认已改为 800）。

### 合成地图规划失败 `Goal is occupied`

中央障碍块 (`--add_demo_obstacles=true`) 可能挡住目标，请使用 `--add_demo_obstacles=false` 或调整起终点。

## 仿真说明

- 控制周期：`1 / controller_frequency`（`controller.lua` 默认 20 Hz）。
- 位姿来源：优先 `OdomSmoother` 中积分后的 `Odometry`（`header.frame_id = map`）。
- 同时发布 `map → base_link` TF，供依赖 TF 的控制器使用。
- 与 `autonomy_tasks_main` 的 mock 仿真类似，但**不**跑行为树，仅评测跟踪性能。

## 与在线导航的关系

- 不订阅真实 `/odom`；离线灌图 + 仿真里程计。
- 动态障碍层（`obstacle_layer`）未启用；与 `planning_test` 相同，适合静态地图对比。
- 完整栈（BT、恢复行为、动态障碍）请用 `autonomy_tasks_main`。

## 相关配置与代码

| 路径 | 说明 |
|------|------|
| `config/control/controller.lua` | 控制器与 checker 参数 |
| `config/planner/planner.lua` | 规划器与 global costmap |
| `autonomy/control/controller_server.hpp` | 控制服务与 `TickFollowPath` |
| `autonomy/control/controller_factory.hpp` | 控制器插件注册 |
| `autonomy/map/utils/pgm_converter.hpp` | 路径渲染为图像 |
| `autonomy/planning/tools/planning_test.cpp` | 同类离线规划可视化工具 |
| `autonomy/control/tools/controller_server_test.cpp` | 本工具实现 |
