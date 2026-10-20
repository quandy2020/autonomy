# 5. Simulation 架构

### 5.1 实现状态

| 组件 | 状态 |
|------|------|
| 多进程 + Bridge | — |
| Vehicle / KinematicsControl | / |
| Stage 资源 / Gazebo 外部包 | / |
| `autonomy/simulation/`、`nav_test` | — |

### 5.2 分层

| 层 | 组件 |
|----|------|
| 应用 | Task / Planning / Control |
| 多进程 | `autonomy.launch`（planning、control、task） |
| 外部 | Gazebo / Stage / Bridge（physics、scan、odom） |
| 桥接 | `autonomy.bridge` / `autonomy_ros` / `VehicleInterface` |

### 5.3 数据流

```
ControllerServer ──cmd_vel──→ Bridge/ROS ──→ Gazebo
↑ │
└──── /odom, /scan, TF ←────────┘
```

无物理时：Bridge 注入 odom/TF；Action → `autonomy.task`。

### 5.4 SimulationServer（规划）

```cpp
class SimulationServer {
    void Step(double dt);
    void SetCommand(Twist cmd);
    Odometry GetOdometry() const;
    void LoadWorld(const SimulationOptions& opts);
};
```

### 5.5 路线图

| 阶段 | 内容 |
|------|------|
| P0 | 多进程 + Bridge 文档/示例 |
| P1 | Vehicle 接仿真后端 |
| P2 | Stage/Gazebo 统一封装 |
| P3 | Habitat/Isaac（可选） |

### 5.6 参见
