# 回环与跨会话

时序见 [03_sequence.md §4](03_sequence.md)；数学见 [04_math.md §6–7](04_math.md)。

## 1. 目录

根路径：`argus/relocalization/loop_closure/`（命名空间仍为 `argus::loop_closure`）。

```text
relocalization/loop_closure/
├── types.hpp / manager.* / consensus.* / cross_session.*
└── scan_context/detector.*
```

| 路径 | 职责 |
|------|------|
| `manager.*` / `types.hpp` | 异步门面与检测器接口 |
| `consensus.*` / `cross_session.*` | 共识与跨会话 |
| `scan_context/` | Scan Context / SC++ |
| `PoseGraph` | `backend/pose_graph/` |
| `MapSession` | `map/map_session.*` |

## 2. 同会话回环

```text
LIO 成功 → TrySubmitKeyFrame
  → LoopClosureManager::SubmitKeyFrame（非阻塞）
  → [async] ScanContext Top-K → Consensus → AddLoopFactor + Optimize
  → LoopCorrection(ΔT)
估计线程 TryInjectLoopCorrection
```

硬约束：`Detect` / `Optimize` **仅** worker；`loop_closure.async` 加载时强制 `true`。

## 3. 配置

YAML 键仍为 `loop_closure.*`（与目录名解耦）：

| 键 | 含义 |
|----|------|
| `enable` | 默认 false |
| `async` | 必须 true |
| `enable_pose_inject` | Optimize 后注入 ESIKF |
| `scan_context.*` | 环扇 / lateral（SC++） |
| `consensus.*` / `cross_session.*` | 共识与跨会话 |

## 4. 测试

`test_loop_closure` / `test_cross_session`。
