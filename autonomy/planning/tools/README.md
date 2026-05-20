# planning 离线可视化工具

本目录提供 **`autonomy_planning_test`**：在不启动完整导航栈的情况下，按 `config/planner/planner.lua` 批量运行所有已配置的规划器，将路径渲染为 PNG，并生成逐帧动画 MP4。

对应源码：`planning_test.cpp`。

## 功能概览

1. 加载 `planner/planner.lua` 中的 `AUTONOMY_PLANNER`（规划器列表、各 planner 参数、costmap 尺寸等）。
2. 启动 `PlannerServer`，对 `planner_plugins` 中的每个 ID 调用 `ComputePathToPose`。
3. 默认使用 `planner.lua` 中的 costmap 层（`static_layer` → `denoise_layer` → `inflation_layer`），PGM 经 `Costmap2DWrapper::loadMap` 注入；仅 `--use_synthetic_map` 时改为 `plugins=none`。
4. 输出静态快照、单规划器视频、以及所有规划器串联的合并视频。

默认会测试的配置（见 `config/planner/planner.lua`）：

- `navfn_planner`
- `dijkstra_planner`
- `theta_star_planner`

在 `planner_plugins` 中增删规划器后，无需改本工具代码即可一并评测。

## 构建

在工程 `src/autonomy` 目录下：

```bash
mkdir -p build && cd build
cmake ..
ninja autonomy_planning_test
```

可执行文件：`build/bin/autonomy_planning_test`。

## 运行示例

在 **Docker / Linux** 中请使用容器内路径（不要用 macOS 的 `/Users/...`）：

```bash
./build/bin/autonomy_planning_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/planning_vis \
  --start_x=30 --start_y=30 \
  --goal_x=370 --goal_y=370 \
  --fps=10 \
  --hold_frames=20
```

`--configuration_directory` 可省略：为空时会走 `ConfigurationSearchDirectories`（源码 `config`、安装目录等），与 `autonomy_tasks_main` 行为一致。

### 默认地图（`config/data/map.pgm`）

未指定 `--map_yaml` 且未加 `--use_synthetic_map` 时，自动加载：

| 文件 | 说明 |
|------|------|
| `config/data/map.yaml` | ROS 风格地图元数据 |
| `config/data/map.pgm` | 栅格图（默认 400×400，分辨率 1 m/格） |

地图经 **StaticLayer** 写入，再经 **DenoiseLayer** / **InflationLayer** 处理（参数见 `planner.lua` 第 117–139 行）。默认起终点 `(30,30)→(370,370)` 可按实际障碍调整。

### 指定其它地图 YAML

```bash
./build/bin/autonomy_planning_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/planning_vis \
  --map_yaml=/path/to/other_map.yaml \
  --start_x=10 --start_y=10 \
  --goal_x=100 --goal_y=100
```

`map.yaml` 中的 `image` 字段为相对 YAML 所在目录的 PGM 路径。

### 合成空地图（不用 PGM）

```bash
./build/bin/autonomy_planning_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --output_dir=/tmp/planning_vis \
  --use_synthetic_map=true \
  --add_demo_obstacles=false \
  --start_x=0.5 --start_y=0.5 \
  --goal_x=19.0 --goal_y=19.0
```

`--use_synthetic_map` 使用 `planner.lua` 中的 costmap 尺寸（默认 20 m × 20 m）；`--add_demo_obstacles=true` 时会在中央添加障碍块。

### 查看全部命令行参数

```bash
./build/bin/autonomy_planning_test --help
```

## 命令行测试

`planning_test_cli.sh` 对工具做端到端冒烟测试（help、默认 `map.pgm`、显式 `map_yaml`、合成地图、单规划器 MP4）：

```bash
# 先编译
cd build && ninja autonomy_planning_test

# 手动跑脚本（仓库根 = src/autonomy）
./autonomy/planning/tools/planning_test_cli.sh

# 或指定构建/配置目录
AUTONOMY_BUILD_DIR=/workspace/autonomy/src/autonomy/build \
AUTONOMY_CONFIG_DIR=/workspace/autonomy/src/autonomy/config \
./autonomy/planning/tools/planning_test_cli.sh
```

启用 `BUILD_TEST` 时纳入 CTest：

```bash
cd build
ctest -R autonomy_planning_test_cli --output-on-failure
```

## 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `configuration_directory` | string | `""` | 配置根目录（与 `autonomy_tasks_main` 共用，定义于 `common/gflags`） |
| `output_dir` | string | `/tmp/autonomy_planning_vis` | PNG / MP4 输出目录 |
| `map_yaml` | string | `""` | 地图 YAML；空则加载 `config/data/map.yaml` |
| `use_synthetic_map` | bool | `false` | 为 true 时不加载 PGM，使用合成 costmap |
| `start_x` / `start_y` | double | `30.0` / `30.0` | 起点 [m]（默认适配 `map.pgm`） |
| `goal_x` / `goal_y` | double | `370.0` / `370.0` | 终点 [m] |
| `fps` | double | `10.0` | 视频帧率 |
| `hold_frames` | int | `20` | 每个规划器片段末尾停留帧数 |
| `animate_poses_per_frame` | int | `2` | 动画每帧新增的路径点数（≥1） |
| `per_planner_video` | bool | `true` | 是否为每个规划器单独写 MP4 |
| `combined_video_name` | string | `planning_all_planners.mp4` | 合并视频文件名（位于 `output_dir`） |
| `add_demo_obstacles` | bool | `true` | 合成地图时是否在中央添加障碍块 |

## 输出文件

在 `--output_dir` 下生成：

| 路径 | 说明 |
|------|------|
| `{planner_id}.png` | 该规划器最终路径快照（规划失败时可能无此文件） |
| `{planner_id}.mp4` | 单规划器路径生长动画（`per_planner_video=true`） |
| `planning_all_planners.mp4` | 所有规划器片段按配置顺序拼接（可通过 `combined_video_name` 改名） |
| `frames/` | 渲染中间帧 PNG（按规划器 ID 命名） |

视频编码为 OpenCV `mp4v`；帧上叠加规划器 ID 标签，失败时显示 `PLAN FAILED`。

## 退出码

| 码 | 含义 |
|----|------|
| `0` | 至少一个规划器成功得到非空路径 |
| `2` | 全部规划器失败或路径为空 |
| `1` | 致命错误（如 costmap 为空、地图 YAML 加载失败） |

## 与在线导航的关系

- 本工具 **不** 订阅 `/map` 话题；`static_layer.subscribe_to_updates=false` 时由 `loadMap(yaml)` 离线灌图。
- 默认保留 `planner.lua` 的 `plugins = {static_layer, denoise_layer, inflation_layer}`；`libautonomy_map_layers_*.so` 配置项仅作文档，进程内直接加载对应 C++ 层。
- `--use_synthetic_map` 时才会设 `plugins=none` 并手写栅格。
- 灌图后会 `updateMap()` 跑一遍 layer 管线，再 `Stop()` 后台线程。

动态障碍层（`obstacle_layer`）请使用 `autonomy_tasks_main` 在线栈。
- 若要在真实环境验证，请使用 `autonomy_tasks_main` 或完整 autonomy 栈；本工具用于快速对比各 grid planner 的路径形态与可视化。

## 相关配置与代码

| 路径 | 说明 |
|------|------|
| `config/planner/planner.lua` | 规划器列表与参数 |
| `autonomy/planning/planner_server.hpp` | 规划服务入口 |
| `autonomy/map/utils/pgm_converter.hpp` | 路径渲染为图像 |
| `autonomy/planning/tools/planning_test.cpp` | 本工具实现 |
