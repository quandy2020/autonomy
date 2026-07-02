# 5. Simulation 架构设计

本文描述 Autonomy 仿真能力的逻辑架构、组件关系与数据流。

## 5.1 设计目标

1. **分层仿真**：运动学仿真（nav_test）→ 物理仿真（Gazebo）→ 高保真（Isaac/Habitat）
2. **无 ROS 可测**：`nav_test` 支持 CI 与快速迭代
3. **统一车辆接口**：`VehicleInterface` 抽象硬件与仿真后端
4. **与导航栈解耦**：仿真器仅提供 odom/TF/传感器，不侵入 planning/control 逻辑

## 5.2 实现状态

| 组件 | 实现度 | 说明 |
|------|--------|------|
| `autonomy_nav_test` | ✅ | 差速积分 + 全栈导航 |
| `Vehicle` / `VehicleInterface` | ⏳ | 接口就绪，仿真后端未接 |
| `KinematicsControl` | ✅ | 限幅算法完整 |
| `simulation.lua` + Stage | ⏳ | 配置与世界文件，无 C++ 客户端 |
| Gazebo 集成 | ✅ | 外部 `autonomy_gazebo` |
| `autonomy/simulation/` 模块 | ❌ | 无独立目录 |

## 5.3 分层架构

<div class="plan-arch-diagram">

  <div class="plan-arch-layer plan-arch-app">
    <div class="plan-arch-header">
      <span class="plan-arch-badge">应用层</span>
      <span class="plan-arch-title">Navigator / Planning / Control</span>
    </div>
    <div class="plan-arch-body">
      <div class="nav-chip-list">
        <span class="nav-chip">NavigateToPose</span>
        <span class="nav-chip">FollowPath</span>
        <span class="nav-chip">cmd_vel</span>
      </div>
    </div>
  </div>

  <div class="plan-arch-pipe"><span>控制指令 / 状态反馈</span></div>

  <div class="plan-arch-split">
    <div class="plan-arch-layer plan-arch-server">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">进程内</span>
        <span class="plan-arch-title">autonomy_nav_test</span>
      </div>
      <div class="plan-arch-body">
        <div class="nav-chip-list">
          <span class="nav-chip">IntegrateDiffDrive</span>
          <span class="nav-chip">UpdateOdometry</span>
          <span class="nav-chip">setTransform</span>
        </div>
      </div>
    </div>

    <div class="plan-arch-link">
      <span class="plan-arch-link-text">或</span>
      <span class="plan-arch-link-arrow">↔</span>
    </div>

    <div class="plan-arch-layer plan-arch-plugin">
      <div class="plan-arch-header">
        <span class="plan-arch-badge">外部</span>
        <span class="plan-arch-title">Gazebo / Stage</span>
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
      <span class="plan-arch-title">autonomy_ros / VehicleInterface</span>
    </div>
  </div>

</div>

## 5.4 数据流对比

### 5.4.1 nav_test（闭环）

```
ControllerServer ──cmd_vel──→ nav_test 仿真线程
       ↑                              │
       └──── UpdateOdometry + TF ←────┘
```

### 5.4.2 Gazebo（开环桥接）

```
ControllerServer ──cmd_vel──→ autonomy_ros ──→ Gazebo
       ↑                              │
       └──── /odom, /scan, TF ←────────┘
```

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

接入 `Vehicle::ApplyCommand` 与 `simulation.lua`。

## 5.6 扩展路线图

| 阶段 | 内容 |
|------|------|
| P0 | 完善 nav_test CMake 接入根构建 |
| P1 | `Vehicle` 接 nav_test 仿真后端 |
| P2 | `simulation.lua` 接入 `autonomy.lua` |
| P3 | Stage 客户端或 Gazebo 插件统一封装 |
| P4 | Habitat / Isaac 适配器（可选） |
