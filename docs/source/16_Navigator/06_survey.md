# 6. 导航编排综述（Survey）

> **本文范围**：编排谱系、Autonomy 能力边界与**选型依据**。  
> 形式化见 [§0 指南](00_guide.md)；模块架构见 [§1](01_architecture.md)；BT 专题见 [§2–§5](02_bt_algorithms.md)。

---

## 6.1 综述定位

| 维度 | 本文 | 其他文档 |
|------|------|----------|
| SE(2) 任务形式化、双层 GoalChecker | 不展开 | [§0.3](00_guide.md#03-问题形式化) |
| BtNavigator 分层、tick 时序 | 不展开 | [§1](01_architecture.md)、[§3 引擎](03_bt_engine.md) |
| XML 逐层、插件端口 | 摘要 + 链接 | [§2 总览](02_bt_algorithms.md) · [§4–§5](04_navigate_to_pose.md) |
| 编排历史、分类、选型 | **本文** | — |
| 全局路径 / 局部轨迹 | 摘要 | [Planning 综述 §6](../08_Planning/06_survey.md) · [Control 综述 §6](../09_Control/06_survey.md) |

**建议阅读顺序**

| 角色 | 路径 |
|------|------|
| 选型 / 集成 | §6.6 → §6.7 排错对照 |
| BT 定制 | §6.3 → §6.5 模式族 → [§4](04_navigate_to_pose.md) |
| 背景调研 | §6.4 → §6.5 → §6.9 参考文献 |

---

## 6.2 Autonomy 能力边界

Autonomy `navigator` 是 **nav2 兼容的 L1 编排层**：将用户目标转化为对 `planning` / `control` / `map` 的 Action 调用序列，不实现路径搜索或跟踪律。

```
1995 BDI/FSM → 2010 ROS move_base → 2018 Nav2 BT → 2022 Groot2 → 2025 Autonomy navigator
                                                                              ├── BT XML（局部生存）
                                                                              ├── 52 插件清单
                                                                              └── 直驱 / BT 双模式
```

| 能力 | 状态 | 说明 |
|------|------|------|
| nav2 对齐 BT XML | ✅ | 单点 + 多点 + Groot 模型 |
| `RecoveryNode` / `PipelineSequence` | ✅ XML | 与 Nav2 默认同型 |
| 局部生存（TF 丢失） | ✅ XML | Autonomy 增强 |
| `BtEngine` + 52 插件 `.so` | ⏳ | 待迁回 |
| 直驱规划调试 | ✅ | `NavigateDirectToPose` |
| HTN / 任务规划器 | ❌ | 上层应用 |
| 多机 MAPF 编排 | ❌ | 需 Fleet 层 |
| LLM 高层任务分解 | ❌ | 研究向（见 §6.8） |

---

## 6.3 编排范式分类

### 6.3.1 四维分类法

```
导航编排
├── 表示：FSM · 行为树 BT · BDI · HTN · 脚本/DSL
├── 反应性：开环脚本 · 每 tick 重检（ReactiveFallback）· 事件驱动 FSM
├── 恢复：硬编码（move_base）· 子树组合（RecoveryNode）· 学习策略
└── 可配置性：编译期 · XML 热替换 · 运行时 Goal 指定树
```

### 6.3.2 范式—特性矩阵

| 范式 | 代表 | 可读性 | 组合性 | 反应性 | 恢复扩展 | Autonomy |
|------|------|--------|--------|--------|----------|----------|
| FSM | move_base、早期游戏 AI | 状态少时好 | 差（$N^2$ 转移） | 需显式边 | 硬编码 | 历史对照 |
| BT | Nav2、BT.CPP | XML + Groot 高 | 子树复用 | Reactive 节点 | RecoveryNode | ✅ 目标 |
| BDI | PRS、JAM | 信念/意图抽象 | 中 | 强 | 需设计 | ❌ |
| HTN | SHOP2、ROSPlan | 任务分解清晰 | 强 | 规划周期 | 重规划层 | ❌ |
| 脚本 | launch + 服务调用 | 低 | 弱 | 弱 | 手动 | 直驱近似 |

---

## 6.4 发展时间轴

按五个历史阶段分块展示里程碑；完整对照见 [§6.4.1](#641-分阶段特征表)。

<div class="planning-timeline-v2">

<div class="timeline-era-block era-foundation">
  <div class="timeline-era-header">任务表示奠基 · 1980s–1990s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1985</div>
      <div class="timeline-milestone-title">BDI 架构</div>
      <div class="timeline-milestone-desc">Belief–Desire–Intention，智能体任务推理</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1992</div>
      <div class="timeline-milestone-title">HTN 规划</div>
      <div class="timeline-milestone-desc">层次任务网络，分解复合任务</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1995</div>
      <div class="timeline-milestone-title">PRS / JAM</div>
      <div class="timeline-milestone-desc">反应式规划与 FSM 混合</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">1998</div>
      <div class="timeline-milestone-title">Gat 综述</div>
      <div class="timeline-milestone-desc">移动机器人三层架构：规划/执行/控制</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-reactive">
  <div class="timeline-era-header">ROS 反应式栈 · 2000s–2010s</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2002</div>
      <div class="timeline-milestone-title">Player/Stage</div>
      <div class="timeline-milestone-desc">开源机器人中间件先驱</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2007</div>
      <div class="timeline-milestone-title">ROS navigation</div>
      <div class="timeline-milestone-desc">move_base FSM + global/local planner</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2009</div>
      <div class="timeline-milestone-title">actionlib</div>
      <div class="timeline-milestone-desc">长时间任务 Action 抽象</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2013</div>
      <div class="timeline-milestone-title">ROSPlan</div>
      <div class="timeline-milestone-desc">PDDL 任务规划接入 ROS</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-sampling">
  <div class="timeline-era-header">行为树兴起 · 2005–2018</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2005</div>
      <div class="timeline-milestone-title">Façade / Halo</div>
      <div class="timeline-milestone-desc">游戏 AI 中 BT 替代 FSM</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2012</div>
      <div class="timeline-milestone-title">BT 期刊论文</div>
      <div class="timeline-milestone-desc">BT 形式化进入学术主流</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2014</div>
      <div class="timeline-milestone-title">BT ≡ FSM 表达力</div>
      <div class="timeline-milestone-desc">BT 与 FSM 等价性证明</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2018</div>
      <div class="timeline-milestone-title">BT 专著</div>
      <div class="timeline-milestone-desc">机器人 BT 系统化教材</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-optimization">
  <div class="timeline-era-header">Nav2 与工程化 · 2018–2023</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2019</div>
      <div class="timeline-milestone-title">Navigation2</div>
      <div class="timeline-milestone-desc">ROS 2 导航栈，BT Navigator 为核心</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2020</div>
      <div class="timeline-milestone-title">BT.CPP v3</div>
      <div class="timeline-milestone-desc">C++17、插件化、黑板</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2022</div>
      <div class="timeline-milestone-title">Groot2</div>
      <div class="timeline-milestone-desc">BT 可视化编辑与在线监控</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">ROS 2 导航综述</div>
      <div class="timeline-milestone-desc">Nav2 算法与 BT 架构权威梳理</div>
    </div>
  </div>
</div>

<div class="timeline-era-block era-learning">
  <div class="timeline-era-header">前沿 · 2023–</div>
  <div class="timeline-era-grid">
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2023</div>
      <div class="timeline-milestone-title">BT 综述 RAS</div>
      <div class="timeline-milestone-desc">机器人/AI 领域 BT 全面调查</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">LLM + BT</div>
      <div class="timeline-milestone-desc">大模型生成/修补行为树</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2024</div>
      <div class="timeline-milestone-title">Nav2 Lifecycle</div>
      <div class="timeline-milestone-desc">编排与生命周期深度集成</div>
    </div>
    <div class="timeline-milestone">
      <div class="timeline-milestone-year">2025+</div>
      <div class="timeline-milestone-title">语义导航编排</div>
      <div class="timeline-milestone-desc">VLM 目标 + 经典 BT 执行层</div>
    </div>
  </div>
</div>

</div>

### 6.4.1 分阶段特征表

| 阶段 | 年代 | 核心矛盾 | 编排解法 | 局限 |
|------|------|----------|----------|------|
| 任务表示 | 1980s–90s | 复合任务如何分解 | BDI、HTN、三层架构 | 与运动层耦合弱 |
| ROS 反应式 | 2000s–10s | 实时避障 + 全局路径 | move_base FSM、actionlib | 恢复逻辑难定制 |
| BT 理论 | 2005–18 | FSM 状态爆炸 | 层次 BT、与 FSM 等价证明 | 工程库分散 |
| Nav2 工程 | 2019–23 | ROS 2 生命周期 + 可配置 | BT Navigator、BT.CPP、Groot2 | 默认树偏保守 |
| 前沿 | 2023– | 自然语言任务 + 安全执行 | LLM 规划 + BT 护栏、语义地图 | 可靠性待验证 |

---

## 6.5 编排模式族

### 6.5.1 行为树导航理论与优势

| 维度 | FSM（move_base） | BT（Nav2 / Autonomy） |
|------|------------------|------------------------|
| 结构 | 扁平转移图 | 层次树 + 子树复用 |
| 可读性 | 状态多时难维护 | XML + Groot 可视化 |
| 反应性 | 需显式转移边 | `ReactiveFallback` 每 tick 重检 |
| 恢复 | `recovery_behaviors` 硬编码列表 | `RecoveryNode` 参数化子树 |
| 定制 | 改 C++ 重编译 | 换 XML 或 Goal 指定树 |

**黑板模式**：节点通过 $\mathcal{B} = \{g, p, q_r, \ldots\}$ 共享数据，解耦 Action 与 Condition。详见 [§2.3](02_bt_algorithms.md#23-控制节点语义)。

### 6.5.2 标准流水线（Pipeline）

$$
\mathrm{Plan} \xrightarrow{\mathrm{Smooth}} \mathrm{Validate} \xrightarrow{\mathrm{Follow}} \mathrm{Goal}
$$

`PipelineSequence` + `RateController`：Nav2 与 Autonomy 默认同型。Autonomy 在 `ReactiveFallback` 内增加 **局部生存** 分支（TF 丢失时低速运动 + 重定位），见 [§4](04_navigate_to_pose.md)。

### 6.5.3 恢复层级

| 级别 | 触发 | 动作 | Autonomy |
|------|------|------|----------|
| L0 上下文 | 单节点 FAILURE | 清局部图 + Wait | LocalSurvival 内 |
| L1 导航 | Pipeline FAILURE | 清全局 + BackUp + Spin | `NavigationRecovery` ×8 |
| L2 航点 | 多点 FAILURE | 清图 + Wait 0.5s | ThroughPoses ×6 |

### 6.5.4 ROS 导航栈演进

| 系统 | 编排 | 里程碑意义 |
|------|------|------------|
| ROS 1 `move_base` | FSM + 恢复行为列表 | 十年工业标准，恢复难扩展 |
| ROS 2 Nav2 | `nav2_bt_navigator` + XML | 可配置、社区插件生态 |
| Isaac Sim / Nav2 | 同型 BT | 仿真—真机同一编排 |
| Autonomy | nav2 对齐 + 局部生存 | C++ 栈、进程内 Action |

---

## 6.6 Autonomy 编排选型

### 6.6.1 模式决策树

```
需要 FollowPath + 恢复？
├── 否 → 直驱模式（规划调试）
└── 是 → 单目标还是多航点？
         ├── 单点 → navigate_to_pose.xml
         │         └── TF 不稳定？→ 增大 local_survival_timeout
         └── 多点 → navigate_through_poses.xml
```

### 6.6.2 场景矩阵

| 场景 | 模式 | 关键配置 | 理由 |
|------|------|----------|------|
| 规划算法调试 | 直驱 | `use_bt_navigation = false` | 跳过 BT |
| 标准室内 | BT 单点 | 默认 XML | 完整流水线 |
| 定位慢恢复 | BT 单点 | `local_survival_timeout = 180` | 局部生存 |
| 仓库巡检 | BT 多点 | 10 Hz `RateController` | 动态重规划 |
| 窄通道 | BT + Theta* | `planner_selector` | 任意角路径 |
| 动态拥挤 | BT 10 Hz 规划 | `RateController hz=10` | 快速响应 |

### 6.6.3 参数敏感度

| 参数 | 过小 | 过大 |
|------|------|------|
| `bt_loop_duration` | tick 延迟 | CPU 占用 |
| 规划频率 (Hz) | 动态障碍反应慢 | 规划负载高 |
| `number_of_retries` | 轻易放弃 | 长时间原地恢复 |
| `goal_reached_tolerance` | 难以 SUCCESS | 停车精度差 |

---

## 6.7 Nav2 对照

| Nav2 包 | Autonomy | 关系 |
|---------|----------|------|
| `nav2_bt_navigator` | `autonomy/navigator` | 编排层 |
| `nav2_behavior_tree` | `behavior_tree/plugins` | BT 插件 |
| `navigate_to_pose` Action | `NavigateToPoseAction` | 顶层接口 |

| Nav2 类 | Autonomy 类 | 状态 |
|---------|-------------|------|
| `BehaviorTreeNavigator` | `BehaviorTreeNavigator<ActionT>` | ✅ |
| `NavigatorMuxer` | `NavigatorMuxer` | ✅ |
| `BtNavigator` | `BtNavigator` | ⏳ |

Action / 插件对照详见 [§1.7](01_architecture.md#17-与系统其他模块的集成) 与历史 Nav2 文档。

---

## 6.8 前沿方向

| 方向 | 代表工作 | 与 Autonomy 关系 |
|------|----------|------------------|
| **BT 形式化验证** | 合成、模型检测 | 长期：关键安全任务 |
| **LLM → BT** | 自然语言生成 XML / 节点 | 上层接口，执行层仍用 BT.CPP |
| **VLM + 经典栈** | 语义目标 → `NavigateToPose` | Bridge 下发目标位姿 |
| **行为树 + 学习恢复** | 学习何时 Spin/BackUp | 研究向，默认仍用规则恢复 |
| **多机编排** | Fleet / MAPF 上层 | 超出单机器人 navigator |

**LLM + BT** 的常见分层：大模型负责任务分解与自然语言接口，**确定性 BT** 负责运动安全与实时反应——与 move_base→Nav2 的「可配置 + 可验证」路线一致，而非端到端替代规划/控制。

---

## 6.9 参考文献

**教材与专著**

1. [Colledanchise & Ögren, *Behavior Trees in Robotics and AI* (2018)](https://doi.org/10.1201/9780429499103)
2. [LaValle, *Planning Algorithms* (2006) — 任务规划章](https://lavalle.pl/planning/)

**里程碑论文**

| 年份 | 论文 | 贡献 |
|------|------|------|
| 1985 | [Bratman 意向理论](https://doi.org/10.2307/2026411) | BDI 哲学基础 |
| 1992 | [Erol et al., HTN](https://doi.org/10.1016/0004-3702(92)90016-N) | 层次任务网络 |
| 1995 | [Rao & Georgeff, BDI](https://doi.org/10.1016/0004-3702(95)00019-4) | 反应式 BDI 智能体 |
| 1998 | [Gat, 三层移动机器人架构](https://doi.org/10.1016/S0921-8890(97)00027-4) | 规划/执行/控制分层 |
| 2005 | [Isla, GDC BT 实践](https://www.gamedeveloper.com/programming/halo-2-behavior-tree-ai) | 游戏 BT 工程化 |
| 2007 | [Gerkey et al., ROS](https://doi.org/10.1109/MRA.2007.339606) | 机器人操作系统 |
| 2012 | [Champandard, BT 期刊](https://doi.org/10.1109/TCIAIG.2012.2189593) | BT 学术形式化 |
| 2014 | [Marzinotto et al., BT 表达力](https://doi.org/10.1109/TCIAIG.2014.2325138) | BT 与 FSM 等价 |
| 2018 | [Macenski et al., Nav2 白皮书](https://doi.org/10.1109/MRA.2018.2870364) | ROS 2 导航设计 |
| 2022 | [Colledanchise et al., BT 综述](https://doi.org/10.1016/j.robot.2022.104096) | 机器人/AI BT 全面调查 |
| 2022 | [Macenski et al., ROS 2 野外应用](https://doi.org/10.1126/scirobotics.abm6074) | ROS 2 生态与 Nav2 |
| 2023 | [Macenski et al., ROS 2 导航综述](https://arxiv.org/abs/2307.15236) | Nav2 算法与 BT 章节 |

**工程资源**

- [Navigation2 · BT Navigator](https://docs.nav2.org/configuration/packages/configuring-bt-navigator.html)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [Groot2](https://github.com/BehaviorTree/Groot2)
- [Nav2 默认 BT 详解](https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html)

---

## 6.10 相关文档

- [§0 指南](00_guide.md) · [§1 架构](01_architecture.md) · [§2 BT 总览](02_bt_algorithms.md)
- [行为树引擎](03_bt_engine.md) · [单点 BT](04_navigate_to_pose.md) · [插件](05_bt_plugins.md)
- [Planning 综述](../08_Planning/06_survey.md) · [Control 综述](../09_Control/06_survey.md)
