# 3. 术语表

速查用；细节以各章指南为准。

### 核心

| 术语 | 说明 |
|------|------|
| Autonomy | 本仓库导航与机载软件栈 |
| Autolink | 进程间通信中间件（channel / service / action） |
| commsgs / automsgs | 消息与 RPC 的 Protobuf 定义 |
| TaskServer | `autonomy.task` 进程；编排导航等 TaskApp |
| Navigator（文档章名） | BT 导航编排；代码在 `autonomy/task/navigation/` |
| Bridge | gRPC 对外面；需 `BUILD_GRPC=ON` |
| TaskMuxer | Bridge/task 侧互斥槽，同时一个 Command |
| BT | BehaviorTree.CPP 行为树 |
| Costmap | 2D 代价地图 |
| TF / 帧 | 坐标变换；常见 `map` / `odom` / `base_link` |

### 易混

| 术语 | 不是 |
|------|------|
| 导航「任务」 | `common::Task` 线程池项 |
| CreateAutonomy | 已移除的单进程入口 |
| Dock RPC 文档页 | 现 ChargeService（charge.proto） |

### 构建开关（常遇）

| 开关 | 作用 |
|------|------|
| `BUILD_GRPC` | Bridge 域与 gRPC 依赖 |
| `BUILD_AUTOVIZ` | 桌面 autoviz 3D |
| `AUTONOMY_BUILD_BRIDGE` | 域级 Bridge 模块（与 BUILD_GRPC 配合） |

### 帧（默认习惯）

| 名 | 典型值 |
|----|--------|
| global_frame | `map` |
| robot_base_frame | `base_link` |
| odom_frame | `odom` |
