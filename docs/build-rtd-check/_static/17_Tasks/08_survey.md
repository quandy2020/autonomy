# 8. 任务综述

### 8.1 能力矩阵

| 能力 | 状态 | 说明 |
|------|------|------|
| `NavigateToPose` API | ✅ | 接口可用 |
| `NavigateThroughPoses` API | ✅ | 接口可用 |
| `NavigatorOptions` 配置 | ✅ | Lua → Protobuf |
| BT XML 定义 | ✅ | 单点/多点 |
| BT 引擎 + 52 插件 | ⏳ | 源码待迁回 |
| 直驱 `GetPlan` | ✅ | 当前默认路径 |
| 完整 FollowPath 闭环 | ⏳ | 依赖 BT 或直驱扩展 |
| 独立 `tasks` 包 | ❌ | 已合并进 Navigator |
| `TaskServer` | ❌ | 不存在 |

### 8.2 与 Navigation2 对比

| 维度 | Autonomy Tasks | Nav2 |
|------|----------------|------|
| 入口 | `system::Autonomy` | `bt_navigator` 节点 |
| 任务类型 | NavigateToPose / ThroughPoses | 同名 Action |
| 编排 | BT（演进中） | BehaviorTree.CPP |
| 配置 | `navigator.lua` | YAML + 参数 |

### 8.3 演进路线

| 阶段 | 目标 |
|------|------|
| 近期 | 恢复 `BtEngine` + 核心 BT 插件 |
| 中期 | `use_bt_navigation` 默认可用 |
| 远期 | Bridge 远程任务下发、多机器人任务队列 |

### 8.4 文档索引

| 主题 | 章节 |
|------|------|
| BT 实现 | [16 Navigator](../16_Navigator/index.rst) |
| 离线测试 | [18 Tools](../18_Tools/04_nav_test.md) |
| 框架装配 | [05 Framework](../05_Framework/index.rst) |
| 常见问题 | [19 FAQs](../19_FAQs/07_navigation.md) |

### 8.5 源码入口

| 路径 | 说明 |
|------|------|
| `autonomy/system/autonomy.{hpp,cpp}` | 任务 API |
| `autonomy/navigator/` | 编排层 |
| `config/navigator/` | 任务配置 |
| `autonomy/commsgs/proto/nav_msgs.proto` | Action 消息 |
