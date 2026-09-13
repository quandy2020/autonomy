# OrbisView

本工程原生 Web 可视化 / 运行观测平台（CivetWeb + Autolink/automsgs 适配）。

- **不**引入 Cyber RT / Bazel
- **不**替换 `autonomy/visualization`（Foxglove）或 `autoviz`
- **不** vendoring Dreamview 源码
- 传输栈：**CivetWeb**（`thirdparty/civetweb`，MIT）

设计：[`docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md`](../../docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md)

## 目录结构

```shell
autonomy/orbisview
├── backend               # C++ 服务（WS / mock / HMI / recorder / plugins）
├── CMakeLists.txt
├── conf
├── frontend
├── launch
├── main.cc
├── proto                 # 仅 *.proto
├── thirdparty/           # civetweb
└── README.md
```

C++ envelope / schema 常量在 `backend/common/`（`stream_envelope.*`、`render_schemas.h`），不放在 `proto/`。

## 阶段状态

| 阶段 | 状态 |
|------|------|
| 一 骨架 + 上游归档 | ✅ |
| 二 最小数据链路 | ✅（CivetWeb `/ws`；WS 断线指数退避重连） |
| 三 基础可视化 | ✅（含 PointCloud2 / Depth、图层、stale） |
| 四 工程分析 | ✅（统计 / 录制回放 / 插件 / PNC Monitor / 点选 goal） |
| 五 WorldModel + Chassis + Obstacles | ✅（Dashboard、障碍物、`cmd_vel` Autolink 钩子） |
| 六 Mosaic 布局 | ✅（`react-mosaic-component` + 地面车 preset） |
| 七 Routing / VectorMap / HMI | ✅（软 HMI 模式，无 cyber_launch） |
| 八 Prediction / Charts / 深 PNC | ✅ |

首期主链路按设计 §14：**默认 mock** 可验证地图、TF、位姿、路径、激光、点云、图像/Depth；Autolink 为可选编译（见下）。DV+ 能力对等走原生栈，**不**拷贝 Cyber。

## Autolink（可选）

在**整仓**构建中开启 OrbisView 时，若已有 `autolink` 目标会自动打开适配：

```bash
cmake -S . -B build -DBUILD_ORBISVIEW=ON -DCMAKE_PREFIX_PATH=/opt/homebrew
cmake --build build --target autonomy.orbisview -j
./build/bin/autonomy.orbisview --mock=false --autolink=true --port=8766
```

macOS Homebrew 需能找到 `osqp` / `eigen` 等（例如已 `brew install osqp`，并设 `CMAKE_PREFIX_PATH=/opt/homebrew`）。整仓还依赖 Lua≥5.2 等，与 OrbisView 本身无关。

独立 `autonomy/orbisview` 目录构建默认 **mock-only**（无 Autolink / automsgs 链接）。

已转换的 `msg_type` → `orbisview.render.*`：

| msg_type | schema |
|----------|--------|
| `geometry_msgs.Pose2D` / `Pose2DStamped` / `PoseStamped` | Pose2D |
| `nav_msgs.Odometry` | Pose2D |
| `nav_msgs.Path` | Path2D |
| `sensor_msgs.LaserScan` | LaserScan |
| `map_msgs.OccupancyGrid` | OccupancyGrid |
| `geometry_msgs.TransformStampeds` / `tf2_msgs.TFMessage` | TfTree |
| `sensor_msgs.PointCloud2` | PointCloud2（最多约 500 点） |
| `sensor_msgs.Image` | Image / DepthImage（通道名含 depth 时） |
| `geometry_msgs.Twist` / `Twist2D` / `TwistStamped` | Twist2D |
| `vehicle_msgs.RobotState` | ChassisState |

**缺口（双轨文档）**：尚无稳定 `automsgs` 障碍物/预测/规划 debug 消息时，对应 schema 仅有 mock；converter 返回 false，通道以 `unsupported` 透传。

Autolink 开启时每 2s 刷新拓扑；通道集合变化会 `Broadcast` 新的 `channels` 列表。未识别类型带 `unsupported=true`。

Teleop：`cmd_vel` 在 mock 下驱动位姿积分；`--autolink=true` 且整仓链接 automsgs 时，额外向 `--cmd_vel_channel`（默认 `/cmd_vel`）发布 `geometry_msgs.Twist2D`。

## 运行

一键（mock 后端 + Vite）：

```bash
./autonomy/orbisview/launch/dev.sh
# 或：cd autonomy/orbisview/frontend && npm run dev:all
# 可选：--flagfile=autonomy/orbisview/conf/orbisview.conf
```

浏览器打开 http://127.0.0.1:5173 → **Connect**（`ws://127.0.0.1:8766/ws`）。Ctrl+C 同时停前后端。

分步启动：

```bash
cmake -S autonomy/orbisview -B build-orbisview -DBUILD_TEST=ON
cmake --build build-orbisview -j
./build-orbisview/bin/autonomy.orbisview --mock=true --port=8766

cd autonomy/orbisview/frontend && npm install && npm run dev
# Connect → ws://127.0.0.1:8766/ws
```

可选静态托管：`--document_root=autonomy/orbisview/frontend/dist`

## Mock 通道

| 通道 | schema |
|------|--------|
| `/orbisview/mock/pose` | `orbisview.render.Pose2D` |
| `/orbisview/mock/path` | `orbisview.render.Path2D` |
| `/orbisview/mock/map` | `orbisview.render.OccupancyGrid` |
| `/orbisview/mock/tf` | `orbisview.render.TfTree` |
| `/orbisview/mock/laser` | `orbisview.render.LaserScan` |
| `/orbisview/mock/image` | `orbisview.render.Image` |
| `/orbisview/mock/pointcloud` | `orbisview.render.PointCloud2` |
| `/orbisview/mock/depth` | `orbisview.render.DepthImage` |
| `/orbisview/mock/exploration` | `orbisview.render.Exploration` |
| `/orbisview/mock/navigation` | `orbisview.render.Navigation` |
| `/orbisview/mock/mapping` | `orbisview.render.Mapping` |
| `/orbisview/mock/twist` | `orbisview.render.Twist2D` |
| `/orbisview/mock/chassis` | `orbisview.render.ChassisState` |
| `/orbisview/mock/obstacles` | `orbisview.render.ObstacleArray` |
| `/orbisview/mock/world` | `orbisview.render.WorldState` |
| `/orbisview/mock/route` | `orbisview.render.RoutePath` |
| `/orbisview/mock/vector_map` | `orbisview.render.VectorMap` |
| `/orbisview/mock/prediction` | `orbisview.render.PredictionObstacles` |
| `/orbisview/mock/planning` | `orbisview.render.PlanningDebug` |
| `/orbisview/mock/hmi` | `orbisview.render.HmiStatus` |
| `/orbisview/mock/components` | `orbisview.render.ComponentsStatus` |

前端：连接状态 `online` / `reconnecting` / `offline`；envelope 超过 2s 标 `stale`。
默认布局为 **Mosaic 地面车 preset**（Map2D 主视口 + Dashboard/Status）；Catalog 可拖入更多面板。底栏 Teleop / PNC。

## 协议要点

控制：`list_channels` / `subscribe` / `unsubscribe` / `status` /
`channel_stats` / `list_plugins` /
`record_start|stop|set_filter` / `playback_start|stop|pause|seek` / `bag_index` /
`plugins_scan|load|unload|reload` / `plugin_host_status` /
`set_goal` / `clear_goal` /
`cmd_vel`（+ `cmd_vel_ack`）/
`set_route` / `clear_route` /
`hmi_set_mode` / `hmi_module_action` / `hmi_status` /
`dump_snapshot` / `clear_sim` / `list_local_bags`

录制 bag：JSONL，首行 `orbisview_bag_header`，其后为 `StreamEnvelope`。

插件：C ABI（`plugins/orbisview_plugin_abi.h`）+ `dlopen`；示例
`orbisview_example_hello` 输出到 `build-*/lib/orbisview/plugins/`。
启动可加 `--plugin_dir=...`。前端 manifest：`frontend/public/plugins/manifest.json`。

数据：`StreamEnvelope`（稳定 `orbisview.render.*` schema）

## 测试

```bash
# C++（需 GTest；独立构建会 FetchContent googletest）
ctest --test-dir build-orbisview --output-on-failure

# 前端
cd autonomy/orbisview/frontend && npm test && npm run build
```

## 验收记录（首期收口）

| 类型 | 结果 |
|------|------|
| 静态 | 目录分层；CivetWeb 许可证见 `thirdparty/civetweb/LICENSE.md` |
| 构建 | `cmake --build build-orbisview --target autonomy.orbisview` |
| 自动化 | `ctest`（envelope / throttle / recorder / plugin；Autolink 开启时含 converter）+ `npm test` |
| 浏览器（手动） | Connect → View3D 见点云；Image 见 Depth 伪彩；杀后端见 `reconnecting` |

浏览器 checklist：

1. 启动 backend + `npm run dev`，Connect
2. View3D：Follow/Free/Reset；图层 grid/robot/path/pointcloud/footprint/map/costmap/laser；cloudColor / mapOpacity / laserHeight
3. Image：同时显示 Image 与 Depth
4. 停止 `autonomy.orbisview`，顶栏应变为 `reconnecting`；重启后自动恢复订阅
