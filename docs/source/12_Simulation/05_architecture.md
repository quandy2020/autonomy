# 5. Simulation 架构设计

本文描述 Autonomy 仿真能力的逻辑架构、组件关系与数据流。

## 5.1 设计目标

1. **分层仿真**：运动学模型（§3）→ 物理仿真（Gazebo）→ 高保真（Isaac/Habitat）
2. **多进程可测**：`autolink_launch` + Bridge/Action，不依赖进程内聚合
3. **统一车辆接口**：`VehicleInterface` 抽象硬件与仿真后端
4. **与导航栈解耦**：仿真器仅提供 odom/TF/传感器，不侵入 planning/control 逻辑

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| 多进程栈 + Bridge | ✅ | `autonomy.launch` 联调 |
| `Vehicle` / `VehicleInterface` | ⏳ | 接口就绪，仿真后端未接 |
| `KinematicsControl` | ✅ | 限幅算法完整 |
| Stage 世界 / 地图资源 | ⏳ | 配置与世界文件，无 C++ 客户端 |
| Gazebo 集成 | ✅ | 外部 `autonomy_gazebo` |
| `autonomy/simulation/` 模块 | ❌ | 无独立目录 |
| `autonomy_nav_test` | ❌ | 已移除 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Task / Planning / Control</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">NavigateToPose</span>
        <span class="nav-chip">FollowPath</span>
        <span class="nav-chip">cmd_vel</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>控制指令 / 状态反馈（autolink IPC）</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-server">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">多进程</span>
        <span class="plan-arch-title">autonomy.launch</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-chip-list">
          <span class="nav-chip">planning</span>
          <span class="nav-chip">control</span>
          <span class="nav-chip">task</span>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">+</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">外部</span>
        <span class="plan-arch-title">Gazebo / Stage / Bridge</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-chip-list">
          <span class="nav-chip">physics</span>
          <span class="nav-chip">/scan</span>
          <span class="nav-chip">/odom</span>
        </div>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>传感器 / 状态</span></div>

  <div class="plan-arch-layer plan-arch-post">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">桥接层</span>
      <span class="plan-arch-title">autonomy.bridge / autonomy_ros / VehicleInterface</span>
    </div>
  </div>

</div>

## 5.4 数据流

### 5.4.1 多进程 + 外部仿真

```
ControllerServer ──cmd_vel──→ Bridge / ROS ──→ Gazebo（或其它后端）
       ↑                              │
       └──── /odom, /scan, TF ←────────┘
```

### 5.4.2 无物理仿真的联调

由 Bridge / 业务进程注入 odom 与 TF，用 Action 向 `autonomy.task` 发目标。

## 5.5 扩展：SimulationServer（规划）

未来可新建 `autonomy/simulation/`：

```cpp
class SimulationServer {
    void Step(double dt);           // 单步仿真
    void SetCommand(Twist cmd);     // 接收 cmd_vel
    Odometry GetOdometry() const;
    void LoadWorld(const SimulationOptions& opts);
};
```

接入 `Vehicle::ApplyCommand` 与仿真 conf。

## 5.6 扩展路线图

| 阶段 | 内容 |
|------|------|
| P0 | 多进程栈 + Bridge/Action 联调文档与示例 |
| P1 | `Vehicle` 接仿真后端 |
| P2 | Stage / Gazebo 客户端统一封装 |
| P3 | Habitat / Isaac 适配器（可选） |
