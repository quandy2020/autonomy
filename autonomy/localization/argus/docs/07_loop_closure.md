# 回环与跨会话

时序见 [03_sequence.md §4](03_sequence.md)；数学见 [04_math.md §6–7](04_math.md)。

## 1. 组件

| 组件 | 路径 |
|------|------|
| `LoopDetectorBase` / `ScanContextDetector` | `loop_closure/scan_context_detector.*` |
| `MultiModalConsensus` | `loop_closure/multi_modal_consensus.*` |
| `LoopClosureManager` | `loop_closure/loop_closure_manager.*` |
| `PoseGraph` | `backend/pose_graph/pose_graph.*` |
| `CrossSessionLinker` | `loop_closure/cross_session_linker.*` |
| `MapSession` | `map/map_session.*` |

## 2. 同会话回环

```text
LIO 成功 → TrySubmitKeyFrame（距离 / 航向阈值）
  → LoopClosureManager::SubmitKeyFrame（非阻塞）
  → [async] ScanContext Top-K → Consensus → AddLoopFactor + Optimize
  → LoopCorrection(ΔT)
估计线程 TryInjectLoopCorrection
  → T' = ΔT · T（协方差轻膨胀）
```

硬约束：`Detect` / `Optimize` **仅** worker；`loop_closure.async` 加载时强制 `true`。

## 3. 配置

`config/loop_closure.yaml` / `LivoOdometryConfig.loop_closure`：

| 键 | 含义 |
|----|------|
| `enable` | 默认 false |
| `async` | 必须 true |
| `enable_pose_inject` | Optimize 后注入 ESIKF（默认 true） |
| `scan_context.*` | 环扇分辨率、候选数 |
| `consensus.min_confidence` | 共识阈值 |
| `keyframe_distance_m` / `keyframe_yaw_rad` | 关键帧选取 |
| `cross_session.*` / `current_session_id` | 跨会话 |

## 4. 跨会话

| 机制 | 范围 |
|------|------|
| `MapSelfCleaning` | 单会话弱观测 |
| `MapSession` + `CrossSessionLinker` | 跨会话对齐 |

流程：

```text
RegisterPriorSession(A) → PoseGraph 节点 + SC IndexKeyFrame
Session B 关键帧 SubmitKeyFrame
  → SC 命中 A → BuildChainFactors / LinkAndOptimize
  → 刷新 B.T_world_session
```

- `LinkAndOptimize` 仅异步 worker。
- 先验 `node_id_base` 取大偏移（如 `1000`）；SC id = `base + local_id`。
- 中间会话缺失：仅对已加载会话建链，不臆造桥接。
- `cross_session.enable=false`：退化为同会话 SC。

### 4.1 MapSession 序列化

文本 `argus_map_session <version>`：

| 字段 | 含义 |
|------|------|
| `session_id` | 会话名 |
| `t_start_sec` / `t_end_sec` | 时间范围 |
| `T_world_session` | 会话锚定 |
| `mean_confidence` | 置信度统计 |
| `node_id_base` | PoseGraph id 偏移 |
| `traj[]` | `local_id`, stamp, `T_session_b` |

完整点云包见 [06_map.md](06_map.md)。加载地图包含 `keyframes/` 时，`LoadBuiltMap` 可注册 SC 库。

## 5. 测试

- `test_loop_closure`：PoseGraph / SC / 异步 Manager
- `test_cross_session`：Save/Load + 双会话链式因子
