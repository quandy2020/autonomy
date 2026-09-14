# 8. 任务综述

### 8.1 能力矩阵

| 能力 | 状态 | 说明 |
|------|------|------|
| `TaskServer` 进程 | ✅ | `autonomy.task` |
| NavigateToPose 语义 | ✅/⏳ | BT / Action 演进中 |
| NavigatorOptions conf | ✅ | pb.txt |
| BT XML 定义 | ✅ | task/conf/behavior_tree |
| BT 引擎 + 插件 | ⏳ | 持续演进 |
| Bridge 远程发令 | ⏳ | 部分实现 |
| 进程内 `system::Autonomy` | ❌ | 已移除 |
| `autonomy_nav_test` | ❌ | 已移除 |

### 8.2 与 Navigation2 对比

| 维度 | Autonomy Tasks | Nav2 |
|------|----------------|------|
| 入口 | `autonomy.task` / TaskServer | `bt_navigator` 节点 |
| 任务类型 | NavigateToPose / ThroughPoses 等 | 同名 Action |
| 编排 | BT（演进中） | BehaviorTree.CPP |
| 配置 | navigator.pb.txt / BT XML | YAML + 参数 |

### 8.3 演进路线

| 阶段 | 目标 |
|------|------|
| 近期 | Bridge / Action 端到端发令稳定 |
| 中期 | BT 默认可用路径 |
| 远期 | 多机器人任务队列 |

### 8.4 文档索引

| 主题 | 章节 |
|------|------|
| BT 实现 | [16 Navigator](../16_Navigator/index.rst) |
| 运行验证 | [04 Running](../04_Running/02_quickstart.md) |
| 框架装配 | [05 Framework](../05_Framework/index.rst) |
| 常见问题 | [19 FAQs](../19_FAQs/07_navigation.md) |

### 8.5 源码入口

| 路径 | 说明 |
|------|------|
| `autonomy/task/task_main.cpp` | 任务进程 |
| `autonomy/task/task_server.*` | TaskServer |
| `autonomy/task/conf/` | 任务与 BT 配置 |
| `autonomy/commsgs/proto/nav_msgs.proto` | Action 消息 |
