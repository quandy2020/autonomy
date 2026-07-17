# Teleop + MPPI 智能避障（RGBD）设计

日期：2026-07-17  
状态：待实现  
范围：`autonomy/task/apps/teleop`

## 1. 目标

人工遥操时，用 **RGBD 相机**感知障碍，按 CMU `local_planner` 的「手柄意图 → 候选路径打分」模式选出局部路径，再由 **`MPPIController`** 跟踪并输出安全 `/cmd_vel`。

成功标准（MVP）：

- 空旷处跟手（方向与手柄意图一致）
- 正前方障碍可减速/绕行，不持续顶障
- `assist_enabled=false` 时行为与现有纯透传 teleop 一致
- watchdog 超时后零速

## 2. 已确认决策

| 项 | 选择 |
|----|------|
| 控制关系 | 手柄给期望方向/速度；MPPI 尽量跟随并避障 |
| 感知链路 | RGBD depth → 点云 → 局部 costmap → MPPI |
| 代码位置 | `task/apps/teleop` 内闭环（不依赖完整 ControllerServer） |
| 意图→路径 | 路径簇 + costmap/方向打分（对标 CMU localPlanner）；MVP 程序生成扇形弧，不强制 PLY |
| 地图表示 | 局部 `costmap_2d`（**不用** grid_map） |

## 3. 架构

```text
手柄 / TeleopGoal.velocity
        │  joySpeed, joyDir
        ▼
┌───────────────────────┐     RGBD depth
│ IntentPathSelector    │◄──── → PointCloud2 → Costmap2DWrapper.feedPointCloud2
│ (预存路径簇 + 打分)    │              │
└───────────┬───────────┘              ▼
            │ Path              local costmap (obstacle + inflation)
            ▼
┌───────────────────────┐
│ MPPIController        │◄── pose / odom
│ SetPlan + Compute…    │
└───────────┬───────────┘
            ▼
 TeleopClient（限速 + watchdog）→ /cmd_vel
```

### 组件

| 组件 | 职责 |
|------|------|
| `RgbdObstacleFeeder` | 订阅 RGBD 深度；反投影为点云（前方、高度裁剪）；`feedPointCloud2` |
| `IntentPathSelector` | 加载路径簇；按手柄方向 + costmap 碰撞/代价选最优局部 `planning_msgs::Path` |
| `TeleopMppiAssist` | 持有局部 `Costmap2DWrapper` + `MPPIController`；每周期选路 → `SetPlan` → `ComputeVelocityCommands` |
| `TeleopClient` | 限速、watchdog、发布 `/cmd_vel`；接收 assist 输出的 twist |

BT 现有 `ApplyTeleopVelocity` / watchdog 条件保留；启用 assist 时，发布的速度来自 `TeleopMppiAssist`，而非原始手柄值。

## 4. 周期数据流（约 10–20 Hz）

1. 读取当前指令 → `joySpeed`（限幅线速）、`joyDir`（由角速度或摇杆映射为期望航向偏角）
2. RGBD 回调异步更新 costmap
3. `|joySpeed|≈0`：不选路，发零线速（原地小角速度可选、可配置）
4. 否则 `IntentPathSelector` 打分选路
5. `MPPIController::SetPlan` + `ComputeVelocityCommands`
6. `TeleopClient` 限速 + watchdog 检查后发布

## 5. 配置

建议新增 `config/task/teleop_assist.lua`（或挂到现有 teleop 配置块）：

- `assist_enabled`：总开关；关闭则纯透传
- RGBD：topic、帧、深度范围、体素/高度阈值
- costmap：分辨率、尺寸、obstacle/inflation 参数
- 路径簇：文件路径（可由 CMU `paths.ply` 转换，仓库内提供一份精简集）
- MPPI：引用/复制 `config/control/controller.lua` 中 mppi 相关参数；进程内 `Configure`，并显式注册 MVP 所需 critics（至少 `CostCritic`、`PathFollowCritic`；可选 `PreferForwardCritic`）
- `stale_cloud_timeout`：无点云/过期时的策略（默认停车）

## 6. 安全与错误

| 情况 | 行为 |
|------|------|
| watchdog 超时 | 零速，任务失败（保持现有语义） |
| RGBD/costmap 过期 | 默认停车（可配置为降速透传，MVP 不做） |
| 无可行路径 | 线速置 0；允许小角速度朝向手柄方向 |
| MPPI/critic 未就绪 | 启动失败并打明确日志 |

## 7. 非目标（本阶段不做）

- 不改造 `ControllerServer` 插件加载与 FollowPath 主环
- 不接入激光或多传感器融合
- 不移植 CMU terrain analysis / pathFollower 整套增益逻辑
- 不做全局规划或自动建图

## 8. 实现落点（文件级，供计划使用）

- 新增：`teleop/rgbd_obstacle_feeder.{hpp,cpp}`
- 新增：`teleop/intent_path_selector.{hpp,cpp}` + 路径数据（如 `config/task/teleop/paths/`）
- 新增：`teleop/teleop_mppi_assist.{hpp,cpp}`
- 修改：`teleop_client` / `apply_teleop_velocity_action` / `TeleopTask` 接入 assist
- 配置：`config/task/teleop_assist.lua`（及必要时 task_options 引用）
- 测试：选路单元测试（合成 costmap）；可选 assist 环 mock 测试

## 9. 参考

- CMU AED：`local_planner`（`localPlanner.cpp` 路径簇打分；`pathFollower.cpp` 跟踪）
- Autonomy：`control/controller/mppi_controller`、`map/costmap_2d`（`feedPointCloud2`）
- 现有：`task/apps/teleop/*`
