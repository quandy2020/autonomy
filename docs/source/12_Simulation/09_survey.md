# 9. 仿真技术综述

公式：[§3](03_math.md)；Gazebo：[§7](07_gazebo_ros.md)。

### 9.1 角色与 fidelity

| 阶段 | 用途 |
|------|------|
| 算法 / 集成 / CI | 迭代、全栈、多进程+Bridge 回归 |
| RL | Isaac、Habitat |

```
运动学积分 → Stage 2D → Gazebo → Isaac/Habitat
```

| 级别 | 代表 | Autonomy |
|------|------|----------|
| 运动学 | Vehicle / 注入 | 接口 |
| 2D / 3D 物理 | Stage / Gazebo | 配置 / 外部包 |
| 具身 AI | Habitat, Isaac | Docker 可选 |

### 9.2 仿真器对比

| 仿真器 | 物理 | 传感器 | Autonomy |
|--------|------|--------|----------|
| 多进程+Bridge | 无/外部 | 注入 | 推荐 |
| Stage | 简单 2D | 激光、相机 | 配置 |
| Gazebo | ODE/Bullet | 丰富 | 外部包 |
| Isaac / Habitat | GPU / 网格 | 丰富 | Docker |

### 9.3 传感器与时钟

| 传感器 | 方式 | 用途 |
|--------|------|------|
| 激光 / 相机 | 射线 / 渲染 | costmap、视觉 |
| odom / IMU | 真值±噪声 | 定位、控制 |

| 模式 | 驱动 |
|------|------|
| 墙钟 | `std::chrono` |
| 仿真时钟 | `/clock` |
| Mock | Autolink `MODE_SIMULATION` |

### 9.4 选型与能力

| 场景 | 推荐 |
|------|------|
| 栈联调 / CI | `autolink_launch` + Bridge |
| 全栈+传感器 | Gazebo + autonomy_ros |
| 2D 原型 | Stage（待接客户端） |
| RL | Habitat / Isaac |

| 能力 | 状态 |
|------|------|
| 差速公式、多进程联调、Gazebo | — |
| Vehicle、Stage | — |
| SimulationServer、噪声/动态障碍、`nav_test` | — |

### 9.5 参考

| 主题 | 链接 |
|------|------|
| Gazebo / ros_gz | [gazebosim.org](https://gazebosim.org/docs) |
| Stage | [Player/Stage](http://playerstage.sourceforge.net/) |
| Habitat | [aihabitat.org](https://aihabitat.org/docs/habitat-sim/) |
