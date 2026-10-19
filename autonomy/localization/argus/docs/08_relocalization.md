# 重定位

时序见 [03_sequence.md §5](03_sequence.md)；评分见 [04_math.md §8](04_math.md)。回环见 [07_loop_closure.md](07_loop_closure.md)。

## 1. 目录

```text
argus/relocalization/
├── types.hpp / config.* / manager.* / thread_pool.*
├── prior_map.* / candidate.hpp / pose_eval.*
├── bbs/             # 3D-BBS：engine + search
├── scan_context/    # SC++ 地点识别 / 候选裁剪
└── loop_closure/    # 回环（命名空间 loop_closure）
```

| 路径 | 职责 |
|------|------|
| `manager.*` / `thread_pool.*` / `config.*` | 异步门面、池、配置 |
| `prior_map.*` / `candidate.hpp` | 先验占据场与候选源接口 |
| `pose_eval.*` | TUM ATE / GT 误差 |
| `bbs/` | 全局搜索 + RefineSE3 |
| `scan_context/` | SC++ 候选裁剪 |
| `loop_closure/` | 回环（见 07） |

## 2. 硬约束

- 池**只**服务搜索；不进 ESIKF 链路。
- 可取消、有界队列；满时默认 `OnFull::kDropOldest`。
- `Search` 仅在池线程；估计线程只 `RequestRelocalization` / `TakeResults` + 注入。
- `use_global_thread_pool: false`（仅 `RelocalizationThreadPool`）。

## 3. 流程

```text
PriorMapLoader::LoadPathBundle(.xyz | 地图包)
  → RelocalizationManager::SetPriorMap(bundle)
  → [可选] scan_context/ 建索引
RequestRelocalization(query body cloud)
  → [pool] bbs/engine
       · place_recognition.enable:
           [可选 aux] → SC++ 门控/检索 Top-K
           → 裁剪 BnB → RefineSE3
           失败且 fallback → 全图 BnB
       · 否则: 全图 / 初值邻域 BnB → RefineSE3
  → results 队列 → TryInjectRelocalization
```

默认 `relocalization.enable: true`；无先验则无结果。  
`place_recognition.enable` 默认 **false**。

## 4. 触发

| `trigger` | 行为 |
|-----------|------|
| `tracking_lost` | 连续丢失 ≥ `min_lost_streak` 且 ≥ `cooldown_sec` |
| `manual` | 仅外部请求 |
| `cross_session_load` | 加载先验后首帧有效点云请求一次 |

## 5. BnB / Place recognition

`relocalization.branch_and_bound`：`search_space`（`se2` / `se3_yaw` / `se3`）、分辨率、阈值等。

`relocalization.place_recognition`：`enable` / `top_k` / `crop_radius_steps` / `fallback_full_search` / SC++ lateral 等。

视觉：`mgr.SetAuxCandidateSource(...)`（`PlaceCandidateSource`）。

## 6. 测试

`test_relocalization`：队列、BnB、Manager、SC++ 裁剪。  
`test_pose_eval`：TUM ATE。
