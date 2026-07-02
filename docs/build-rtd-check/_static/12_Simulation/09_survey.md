# 9. 仿真技术综述（Survey）

本文综述移动机器人仿真技术，并明确 Autonomy 仿真能力的定位与选型。

> 运动学公式见 [03_math.md](03_math.md)；工具对比见 [06_nav_test.md](06_nav_test.md)、[07_gazebo_ros.md](07_gazebo_ros.md)。

---

## 9.1 仿真在机器人开发中的角色

| 阶段 | 仿真用途 |
|------|----------|
| 算法开发 | 快速迭代，无硬件风险 |
| 集成测试 | 全栈闭环验证 |
| CI/CD | 自动化回归（nav_test） |
| 强化学习 | 高保真环境（Isaac, Habitat） |

---

## 9.2 仿真 fidelity 光谱

```
低 fidelity ──────────────────────────────────→ 高 fidelity

运动学积分    Stage 2D    Gazebo Classic    Gazebo Ignition    Isaac Sim
(nav_test)    (规划)      (物理+传感器)      (新引擎)           (GPU 物理)
```

| 级别 | 代表 | 计算成本 | Autonomy |
|------|------|----------|----------|
| 运动学 | nav_test | 极低 | ✅ |
| 2D 物理 | Stage | 低 | 配置占位 |
| 3D 物理 | Gazebo | 中 | 外部包 ✅ |
| 具身 AI | Habitat, Isaac | 高 | Docker 可选 |

---

## 9.3 发展时间轴

<div class="planning-timeline-v2">

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2000s</div>
    <div class="planning-timeline-content">
      <strong>Player/Stage</strong> — 2D 仿真标准
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2007</div>
    <div class="planning-timeline-content">
      <strong>Gazebo</strong> — 3D 物理仿真成为 ROS 默认
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2019</div>
    <div class="planning-timeline-content">
      <strong>Ignition Gazebo</strong> — 下一代 Gazebo
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2020s</div>
    <div class="planning-timeline-content">
      <strong>Isaac Sim, Habitat-Sim</strong> — GPU 加速、RL 训练
    </div>
  </div>

</div>

---

## 9.4 仿真器对比

| 仿真器 | 维度 | 物理 | 传感器 | ROS | Autonomy |
|--------|------|------|--------|-----|----------|
| nav_test | 运动学 | 无 | 无 | 否 | ✅ 内置 |
| Stage | 2D | 简单 | 激光、相机 | 可选 | 配置 |
| Gazebo | 3D | ODE/Bullet | 丰富 | 是 | 外部包 |
| Webots | 3D | 专有 | 丰富 | 是 | — |
| Isaac Sim | 3D | GPU | 丰富 | 桥接 | Docker |
| Habitat-Sim | 3D | 网格 | RGB-D | 桥接 | Docker 脚本 |

---

## 9.5 传感器仿真要点

| 传感器 | 仿真方式 | 用途 |
|--------|----------|------|
| 激光 | 射线投射 | costmap, SLAM |
| 相机 | 渲染管线 | 视觉感知 |
| 里程计 | 真值+噪声 | 定位、控制闭环 |
| IMU | 真值+噪声 | 融合定位 |

Gazebo 插件提供标准 ROS topic；nav_test 仅仿真 odom。

---

## 9.6 仿真时钟

| 模式 | 驱动 | 场景 |
|------|------|------|
| 墙钟 | `std::chrono` | nav_test 默认 |
| 仿真时钟 | `/clock` topic | Gazebo `use_sim_time` |
| Mock 时间 | Autolink `MODE_SIMULATION` | 回放、单元测试 |

---

## 9.7 Autonomy 选型建议

| 场景 | 推荐 |
|------|------|
| 单元测试 / CI | `autonomy_nav_test` |
| 控制器调参 | `autonomy_controller_test` 或 nav_test |
| 全栈 + 传感器 | Gazebo + autonomy_ros |
| 2D 快速原型 | Stage（待接 C++ 客户端） |
| RL / 具身智能 | Habitat / Isaac（Docker） |

---

## 9.8 能力矩阵

| 能力 | 状态 |
|------|------|
| 差速运动学仿真 | ✅ nav_test |
| 全栈 BT 导航闭环 | ✅ nav_test |
| 车辆抽象接口 | ⏳ Vehicle |
| Gazebo 集成 | ✅ 外部 |
| Stage 集成 | ⏳ 配置 |
| 统一 SimulationServer | ❌ |
| 传感器噪声模型 | ❌ |
| 动态障碍仿真 | ❌ |

---

## 9.9 参考文献

| 主题 | 参考 |
|------|------|
| Gazebo | gazebosim.org 文档 |
| Stage | Player/Stage 项目 |
| ROS 2 Gazebo | ros_gz 文档 |
| Habitat-Sim | aihabitat.org |
