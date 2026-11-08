# Backend 模块总览

Backend 承接前端发布的稳定关键帧和地图快照，完成回环检索与验证、约束图维护、跨会话地图装载、PGO/HBA 编排以及优化结果提交。它允许异步和有界延迟，但不得持有前端地图锁执行长优化。

## 子模块职责

| 子模块 | 输入 | 输出 | 唯一状态所有者 | 不负责 |
|---|---|---|---|---|
| `loop_closure/` | 关键帧快照、当前/历史描述子索引 | 排序候选、几何验证结果、已验证相对约束 | BTC 描述子索引及待验证候选 | 修改 PoseGraph 位姿、改写前端体素地图 |
| `pose_graph/` | 节点初值、里程计边、回环边、跨会话边 | 图优化请求、版本化 pose 更新、连通关系 | 图节点、约束、会话连通图和图版本 | 持有完整点云、运行 BTC/HBA 几何算法 |
| `global_mapping/` | 会话结束/稳定地图请求、不可变关键帧/体素快照 | 多会话导入状态、PGO/HBA 任务、全局结果 | 历史会话装载、全局任务队列和任务生命周期 | 重新实现 HBA 数学、直接操作实时局部窗口 |

## 子模块文档

- [回环检测与 BTC](backend/loop-closure.md)
- [位姿图与约束管理](backend/pose-graph.md)
- [多会话与全局建图](backend/global-mapping.md)

## 跨模块数据路径

```text
TrajectoryBuilder
  -> immutable KeyframeSnapshot
  -> LoopClosure: descriptor lookup -> candidate ranking -> geometric verification
  -> Verified Constraint
  -> PoseGraph: add node/constraint -> Atlas 自有 optimization request
  -> GlobalMapping: previous-session alignment / HBA request / top-down correction
  -> versioned pose/map update
  -> frontend controlled correction boundary
```

回环模块只产出测量，不直接更新图。PoseGraph 是图状态和约束的唯一 owner；GlobalMapping 管理跨会话的输入加载和全局任务调度，并通过 PoseGraph 公共接口提交跨会话约束。全局 HBA 的几何实现留在 `mapping/`，backend 负责快照、调度、取消/收尾和结果校验。

## 命名与拆分原则

- 目录由职责命名：`loop_closure`、`pose_graph`、`global_mapping`。BTC 子模块使用 `triangle/`，避免再套传感器类型层。
- 文件采用领域名词：`pose_constraint.hpp`、`pose_graph.hpp`、`global_mapping_request.hpp`。目录已有上下文时不重复项目全名、传感器名或 `backend_` 前缀；类型和文件名不使用 `SLAM` 字样。
- `internal/` 只容纳模块实现细节。只有稳定接口会被模块外调用或需替换时，才放模块根目录；不为每个 struct 机械生成头/源文件。
- Backend 使用 `backend/pose_graph/` 中的因子图值类型和 `PoseGraphSolver`；求解器封装 ISAM2 的追加式更新。
- 下列文件名是目标设计。现有实现与目标有差异时，采用兼容迁移，不暗示文件已经完成移动。
