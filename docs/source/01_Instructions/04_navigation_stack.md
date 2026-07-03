# 4. 导航栈全景

移动机器人**自主导航**通常分解为感知、定位、建图、规划、控制、编排六个协作子系统。本章给出 Autonomy 导航栈的全景视图与各层职责。

## 4.1 导航子问题

| 子问题 | 英文 | Autonomy 模块 | 典型频率 |
|--------|------|---------------|----------|
| 感知 | Perception | `perception` | 10–30 Hz |
| 定位 | Localization | `localization` | 10–50 Hz |
| 建图 | Mapping | `map` | 1–10 Hz |
| 全局规划 | Global Planning | `planning` | 1–5 Hz |
| 局部控制 | Local Control | `control` | 10–50 Hz |
| 任务编排 | Orchestration | `navigator` | 事件驱动 |

## 4.2 导航栈流水线

<div class="nav-stack-pipeline">

  <div class="nav-input-row">
    <div class="nav-input-card">
      <span class="nav-input-label">传感器</span>
      <span class="nav-input-text">激光 / 相机 / IMU</span>
      <span class="nav-input-arrow">→</span>
      <span class="nav-input-target">obstacle_layer</span>
    </div>
    <div class="nav-input-card">
      <span class="nav-input-label">定位</span>
      <span class="nav-input-text">Atlas / AMCL</span>
      <span class="nav-input-arrow">→</span>
      <span class="nav-input-target">TF map→odom</span>
    </div>
    <div class="nav-input-card">
      <span class="nav-input-label">先验地图</span>
      <span class="nav-input-text">PGM / SLAM 图</span>
      <span class="nav-input-arrow">→</span>
      <span class="nav-input-target">static_layer</span>
    </div>
  </div>

  <div class="nav-costmap-banner">
    <span>融合与膨胀后生成</span>
    <strong>Costmap2D</strong>
    <span class="nav-costmap-detail">（static + obstacle + inflation）</span>
    <span class="nav-costmap-arrow">↓ 规划环境模型</span>
  </div>

  <div class="nav-layer-row nav-layer-behavior">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L1</span>
      <div class="nav-layer-title">编排层</div>
      <div class="nav-layer-sub">navigator</div>
      <span class="nav-meta-freq">事件驱动</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-body-block">
        <div class="nav-body-label">职责</div>
        <ul>
          <li>解析导航目标，维护任务状态机</li>
          <li>BT 协调规划 / 控制 / 恢复</li>
          <li>TF 丢失时局部生存模式</li>
        </ul>
      </div>
      <div class="nav-chip-list">
        <span class="nav-chip">navigate_to_pose</span>
        <span class="nav-chip">RecoveryNode</span>
        <span class="nav-chip">PipelineSequence</span>
      </div>
    </div>
  </div>

  <div class="nav-pipe"><span class="nav-pipe-label">规划请求</span></div>

  <div class="nav-layer-row nav-layer-global">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L2</span>
      <div class="nav-layer-title">全局规划</div>
      <div class="nav-layer-sub">planning</div>
      <span class="nav-meta-freq">1–5 Hz</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-chip-list">
        <span class="nav-chip">navfn_planner</span>
        <span class="nav-chip">dijkstra_planner</span>
        <span class="nav-chip">theta_star_planner</span>
        <span class="nav-chip">IsPathValid</span>
      </div>
    </div>
  </div>

  <div class="nav-pipe"><span class="nav-pipe-label">Path</span></div>

  <div class="nav-layer-row nav-layer-local">
    <div class="nav-layer-meta">
      <span class="nav-layer-badge">L3</span>
      <div class="nav-layer-title">局部控制</div>
      <div class="nav-layer-sub">control</div>
      <span class="nav-meta-freq">10–50 Hz</span>
    </div>
    <div class="nav-layer-body">
      <div class="nav-chip-list">
        <span class="nav-chip">FollowPath</span>
        <span class="nav-chip">GoalChecker</span>
        <span class="nav-chip">VelocitySmoother</span>
        <span class="nav-chip">cmd_vel</span>
      </div>
    </div>
  </div>

</div>

## 4.3 坐标系约定

| 坐标系 | 典型名称 | 来源 | 用途 |
|--------|----------|------|------|
| 全局 | `map` | 定位 / SLAM | 全局规划、目标点 |
| 里程计 | `odom` | 轮速计 / IMU 积分 | 局部平滑、短期参考 |
| 机器人 | `base_link` | 机器人本体 | 控制、传感器外参 |

TF 链：`map → odom → base_link`。配置在 `config/common.lua` 的 `global_frame` / `robot_base_frame`。

## 4.4 一次导航的数据路径

```
1. 用户下发 Goal (map 系 PoseStamped)
2. navigator: TransformAvailable 检验 TF
3. planning:  GetPlan(start, goal) → Path
4. smoother:  SmoothPath（可选）
5. planning:  IsPathValid(path)
6. control:   FollowPath(path) → cmd_vel 循环
7. GoalChecker 判定到达 → SUCCESS
```

直驱模式（当前 `Autonomy` 默认）仅执行步骤 1–3，不进入 FollowPath 循环。

## 4.5 与 Navigation2 对照

| Nav2 包 | Autonomy | 说明 |
|---------|----------|------|
| `nav2_bt_navigator` | `navigator` | 行为树编排 |
| `nav2_planner` | `planning` | 全局规划 |
| `nav2_controller` | `control` | 局部控制 |
| `nav2_costmap_2d` | `map/costmap_2d` | 代价地图 |
| `nav2_amcl` | `localization`（AMCL 待集成） | 定位 |
| `nav2_msgs` | `commsgs` | 消息 |

## 4.6 模块深入阅读

| 主题 | 文档 |
|------|------|
| 定位 | [06 Localization](../06_Localization/index.rst) |
| 地图 | [07 Mapping](../07_Map/00_guide.md) |
| 规划 | [08 Planning](../08_Planning/00_guide.md) |
| 控制 | [09 Control](../09_Control/00_guide.md) |
| 编排 | [16 Navigator](../16_Navigator/00_guide.md) |

## 4.7 相关文档

- [§3 系统架构](03_system_architecture.md)
- [08 Planning 综述](../08_Planning/06_survey.md)
