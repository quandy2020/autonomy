# 6. 导航编排综述

形式化入口 [§0](00_guide.md)；实现 [§1](01_architecture.md)。

选型常卡在：BT 还是直驱、和 Nav2 怎么对照、要不要上 HTN/多机。

BT 在 autonomy/task/；TaskServer 编排。

### 本仓库实际有的

- Nav2 风格 BT XML：**单点**（含 LocalSurvival）、**多点**  
- TaskServer 内 BT 引擎 + 大量 `autonomy_behavior_tree_*` 插件  
- 直驱 GetPlan / NotifyPath：调规划、不跟完整树  
- Bridge gRPC → Task GoalChannel

### 刻意不做的（文档层）

HTN、多机 MAPF、LLM 任务分解等不在当前 task 栈产品路径里；需要时请另开模块。

### 范式对照

| 范式 | 代表 | Autonomy |
|------|------|----------|
| FSM | 早期 move_base | 历史参考 |
| BT | Nav2、BehaviorTree.CPP | **主路径** |
| 脚本 launch | 临时编排 | 直驱近似 |

### 实用选型

| 场景 | 建议 |
|------|------|
| 生产单点/多点 | BT + `autonomy.launch` |
| 只验规划 | 直驱 GetPlan |
| 定位差、TF 偶发丢 | 单点树 LocalSurvival（别先改 planner） |
| 场外发令 | Bridge Navigate + 确认 task 进程 |

### 敏感参数

| 参数 | 风险 |
|------|------|
| `goal_reached_tol` | 过大早停；过小永不 SUCCESS |
| RateController Hz | 低则反应慢；高则 CPU |
| Recovery 次数 | 过多原地折腾；过少易失败 |

### Nav2 对照

| 项 | Nav2 | Autonomy |
|----|------|----------|
| 入口 | `bt_navigator` | TaskServer + `task/navigation` |
| 插件 | `.so` + factory | 同型命名 |
| 增强 | （按需） | LocalSurvival、进程内直调 |

参考：Nav2 BT 文档、BehaviorTree.CPP、Groot2。
