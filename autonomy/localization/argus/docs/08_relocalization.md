# 重定位

时序见 [03_sequence.md §5](03_sequence.md)；评分见 [04_math.md §8](04_math.md)。

## 1. 职责

跟踪丢失或跨会话冷启动时，在先验占据栅格上做分层分支定界搜索；结果**原子注入**实时状态。

| 文件 | 职责 |
|------|------|
| `relocalization_thread_pool.*` | 专用有界队列线程池 |
| `prior_map_loader.*` | `.xyz` / 地图包 → 占据场 |
| `branch_and_bound.*` / `bbs_search_engine.*` | 搜索 |
| `relocalization_manager.*` | 异步门面 + 结果队列 |
| `pose_trajectory_eval.*` | TUM ATE / GT 误差 |

## 2. 硬约束

- 池**只**服务搜索；不进 ESIKF 链路。
- 可取消、有界队列；满时默认 `OnFull::kDropOldest`。
- `Search` 仅在池线程；估计线程只 `RequestRelocalization` / `TakeResults` + 注入。
- `use_global_thread_pool: false`（仅 `RelocalizationThreadPool`）。

## 3. 流程

```text
PriorMapLoader::FromCloud / LoadPath(.xyz | 地图包)
  → RelocalizationManager::SetPriorMap
  （包目录优先 prior.xyz，否则 map.xyz；或 LoadBuiltMap）
RequestRelocalization(query body cloud)
  → [pool] BbsSearchEngine / BranchAndBound
  → results 队列
TryInjectRelocalization
  → 整包替换 pos/rot，位置协方差重置
```

默认 `relocalization.enable: true`；无 `prior_map_path` / 未 `LoadBuiltMap` 时不会出结果。

## 4. 触发

| `trigger` | 行为 |
|-----------|------|
| `tracking_lost` | 连续丢失 ≥ `min_lost_streak` 且 ≥ `cooldown_sec` |
| `manual` | 仅外部请求 |
| `cross_session_load` | 加载先验后首帧有效点云请求一次 |

`IsTrackingLost`：下采样空 / LiDAR 健康超时 / 门控 `lidar_valid==false` / 退化 `is_degenerated==true`。

自动请求可用当前 body 云；`seed_with_current_pose` 以 ESIKF 位姿为初值。

手动：`LoadPriorMap(path)` / `RequestRelocalization(req)`。

## 5. BnB 参数

`relocalization.branch_and_bound`：

| 项 | 说明 |
|----|------|
| `search_space` | `se2` / `se3_yaw` / **`se3`（默认）** |
| `resolution` / `z_resolution` | 粗层 xy / z |
| `yaw_step_rad` / `search_radius_*` | 航向与搜索半径 |
| `roll_pitch_refine_rad` / `roll_pitch_steps` | 局部 roll/pitch 微调 |
| `score_threshold` / `max_depth` / `max_expansions` | 接受与预算 |

真值对比：`EvaluateAgainstGroundTruth` → 平移/转角误差与 `within_tolerance`。  
离线 ATE：`eval_pose_traj est.tum gt.tum`。

## 6. 测试

`test_relocalization`：队列、SE(2)/SE3Yaw 召回、GT 误差、Manager 异步、streak / cooldown。  
`test_pose_eval`：TUM ATE + `LoadPath`。
