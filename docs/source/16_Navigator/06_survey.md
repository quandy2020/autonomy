# 6. 导航编排综述

选型与边界。形式化 [§0](00_guide.md)；架构 [§1](01_architecture.md)；BT [§2–§5](02_bt_algorithms.md)。

### 6.1 能力边界

| 能力 | 状态 |
|------|------|
| nav2 对齐 BT XML（单点+多点+局部生存） | — |
| `BtEngine` + 52 插件 `.so` | — |
| 直驱调试 | — |
| HTN / 多机 MAPF / LLM 分解 | — |

### 6.2 范式对照

| 范式 | 代表 | Autonomy |
|------|------|----------|
| FSM | move_base | 历史对照 |
| BT | Nav2、BT.CPP | 目标 |
| BDI / HTN | PRS、SHOP2 | — |
| 脚本 | launch+服务 | 直驱近似 |

### 6.3 时间轴（摘要）

| 阶段 | 里程碑 |
|------|--------|
| 1980s–90s | BDI / HTN / PRS |
| 2010s | ROS `move_base` |
| 2018+ | Nav2 BT + Groot |
| 本仓库 | Autonomy navigator（局部生存增强） |

### 6.4 模式族

| 族 | 要点 |
|----|------|
| Pipeline | Compute → Smooth → Validate → Follow |
| Recovery | RecoveryNode 组合清图/后退/旋转 |
| 反应式 | ReactiveFallback 每 tick 重检 |

### 6.5 选型

| 场景 | 推荐 |
|------|------|
| 生产单点/多点 | BT XML |
| 调试规划 | 直驱 GetPlan |
| 弱定位 | 启用 LocalSurvival |

| 参数 | 敏感度 |
|------|--------|
| `goal_reached_tol` | 高（过早 SUCCESS / 永不到达） |
| tick / RateController Hz | 中（CPU vs 响应） |
| Recovery retries | 中（卡住时长） |

### 6.6 Nav2 对照

| 项 | Nav2 | Autonomy |
|----|------|----------|
| 引擎 | `nav2_bt_navigator` | `BtNavigator` / TaskServer |
| 插件 | `.so` | 同型，命名 `autonomy_behavior_tree_*` |
| 增强 | — | LocalSurvivalMode |

### 6.7 参考

- Nav2 Behavior Tree 文档
- BehaviorTree.CPP / Groot2
