# 03 · 数据流

## 1. 总览

```text
autosim
  /camera/depth/image_raw ─┐
  /camera/camera_info ─────┤
  /odom ───────────────────┤ ObservationBuilderComponent
  /imu ────────────────────┤   (depth 触发)
  /map ────────────────────┘          │
                                      ▼
                         /navigation/observation (Observation)
                                      │
                                      ▼
                            NavigationComponent
                                      │
                                      ▼
                                  Runner.Tick
                    ┌─────────────────┼──────────────────┐
                    ▼                 ▼                  ▼
              Encoder            Memory              Policy worker
           DepthFeature       ContextView          NavigationIntent
                    └─────────────────┼──────────────────┘
                                      ▼
                                ChunkBlender
                                      │
                                      ▼
                             Scenario/Stage/Rules
                                      │
                                      ▼
                        /navigation/cmd_vel_raw (Twist)
                                      │
                                      ▼
                              SafetyComponent
                     Reflex → CBF → AEB → Feasibility
                                      │
                                      ▼
                             /cmd_vel (TwistStamped)
                                      │
                                      ▼
                                   autosim
```

## 2. Topic 契约

| Topic | 方向 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| `/camera/depth/image_raw` | autosim → nav | `sensor_msgs/Image` 32FC1 | ~30 Hz | 触发 Observation |
| `/camera/camera_info` | autosim → nav | `sensor_msgs/CameraInfo` | ~30 Hz | 内参 |
| `/odom` | autosim → nav | `nav_msgs/Odometry` | ~30 Hz | `map -> base_link` |
| `/imu` | autosim → nav | `sensor_msgs/Imu` | ~30 Hz | 可选 |
| `/map` | autosim → nav | `map_msgs/OccupancyGrid` | 1 Hz | 探索前沿 |
| `/navigation/observation` | nav 内部 | `navigation.observation.Observation` | depth 触发 (~30 Hz) | 融合快照 |
| `/navigation/goal` | 外部 → nav | `navigation.GoalSpec` | 事件 | 目标 |
| `/navigation/status` | nav → behavior | `navigation.NavigationStatus` | ~15 Hz | 状态 |
| `/navigation/heartbeat` | nav → guardian | `navigation.lifecycle.Heartbeat` | ~15 Hz | 心跳 |
| `/navigation/cmd_vel_raw` | nav → safety | `geometry_msgs/Twist` | 每 Observation | 未安全校验 |
| `/cmd_vel` | safety → autosim | `geometry_msgs/TwistStamped` | 50 Hz | 唯一执行出口 |
| `/safety/heartbeat` | safety → guardian | `lifecycle.Heartbeat` | 50 Hz | 心跳 |
| `/behavior/heartbeat` | behavior → guardian | `lifecycle.Heartbeat` | 10 Hz | 心跳 |
| `/guardian/degradation` | guardian → 外部 | `lifecycle.Degradation` | 事件 | 降级 |

## 3. Observation 融合

`ObservationBuilderComponent`：

1. depth 到达触发 `Proc`。
2. 读取最新 `camera_info`、`odom`、`imu`、`map`。
3. `ObservationBuilder::Build` 拷贝 depth/camera/odom/imu，写 validity。
4. 拷贝 `map`（用于探索）。
5. 发布 `Observation`；`header` 取 depth 的 header。

`validity` 位：

- `DEPTH_VALID`
- `CAMERA_INFO_VALID`
- `ODOM_VALID`
- `IMU_VALID`
- `TF_VALID`（当前由 odom 是否存在近似）

## 4. Runner 决策流程

### 4.1 特征与记忆

```text
Encode(Observation) -> DepthFeature
  tokens / pooled / uncertainty / stamp_ns / sequence
Memory.Insert(Keyframe{pose, embedding, stamp})
```

- DeFM 编码器：depth → 3×224×224 → ViT-S/14 → tokens[256,384]。
- 无模型仿真：`DepthStatisticsEncoder` 输出 8 维统计特征，不依赖 ONNX。

### 4.2 策略异步

- `Tick` 每 `decision_period` 提交一个 job，worker 覆盖旧 job。
- worker 返回 `NavigationIntent`：
  - `chunk`：动作块；
  - `velocity`：单步速度（当前 Runner 只把 chunk 推入 Blender）。
- 过期结果按 sequence 丢弃。

### 4.3 Blender 与场景

- `ChunkBlender::Push` 融合剩余动作，`Consume(dt)` 按时间插值。
- `ScenarioManager` 按 `GoalSpec.type` 选择 destination/exploration/following/coverage。
- Stage 内 trajectory/obstacle/speed 优化；Rule 链做限速、禁行区、安全链等。

## 5. 目标流

### 外部 Action：/navigate_to_pose

```text
Action client
  /navigate_to_pose (automsgs.actions.NavigateToPoseAction)
    Goal { pose: PoseStamped, behavior_tree }
      -> NavigationComponent::RunNavigateToPose
      -> GoalSpec{type=POSE, pose}
      -> Runner.SetGoal
      -> ... -> /cmd_vel

  Feedback { current_pose, navigation_time, distance_remaining }
  Result   { error_code, error_msg }
```

- 取消：Action cancel → `Runner.ClearGoal` → result `NAVIGATION_CANCELLED`。
- 抢占：新 goal pending → 旧 goal 返回 `NAVIGATION_CANCELLED`（`error_msg=preempted by newer goal`）→ `AcceptPendingGoal` → 重新 `SetGoal`。
- 目标完成：`Runner.IsGoalReached()` → result `OK`。
- 同时支持 `/navigation/goal` 话题；**最新目标胜出**：
  - 新 Action goal 抢占旧 Action goal；
  - `/navigation/goal` 抢占当前 Action goal；
  - 新 Action goal 覆盖旧 topic goal。

### 点到点

```text
/navigation/goal {type: TYPE_POSE, pose: PoseStamped}
  -> NavigationComponent.SetGoal
  -> ScenarioManager.Update(destination)
  -> SimulationPolicy/EnginePolicy -> chunk
  -> Blender -> Stage -> Rules -> cmd_vel_raw
```

### 探索

```text
initial_goal {type: TYPE_EXPLORATION}
  -> ScenarioManager.Update(exploration)
  -> SimulationPolicy
       Observation.map -> FrontierDetectorTask
       -> nearest frontier -> velocity chunk
       no frontier -> forward + turn
  -> Scenario -> cmd_vel_raw
```

## 6. 恢复流

```text
NavigationComponent 记录 last_progress
  if pose goal && no distance improvement > 0.1 m for 8 s:
      Runner.ClearGoal()
      Runner.SetGoal(last_goal)
```

- 恢复期间安全链持续生效。
- 这是仿真级基础恢复；完整 BT 恢复（ClearGoal/Relocalize/Retry/GiveUp）在
  `planning/tasks/recovery_*` 与 `behavior/` 中，尚未接入 component。

## 7. 安全链

```text
candidate (cmd_vel_raw)
  -> ValidInputs: finite / dt / observation stamp freshness
  -> ReflexPolicyArbiter: near-field stop/turn
  -> CbfFilter: half-plane projection
  -> AebVelocitySampler: CBF infeasible 时采样
  -> VelocityFeasibilityFilter: acceleration/jerk
  -> SafetyResult
  -> TwistStamped /cmd_vel
```

任何失败：`EmergencyStop` → 零速 → `/cmd_vel` 仍发布，autosim 立即停车。

## 8. 时间基准

| 数据 | 时间 |
| --- | --- |
| autosim depth/odom/map | 仿真 `Header.stamp` |
| Observation | depth header |
| Safety 新鲜度 | 与 depth stamp 比较 |
| Runner `dt` | `steady_clock`（仿真/实时均可） |
| Heartbeat | 到达时间由 guardian 记录，stamp 仅诊断 |

## 9. 失败与降级

| 事件 | 行为 |
| --- | --- |
| 无 depth | ObservationBuilder 不发布；Runner 不 tick |
| 无 odom | pose 为零；策略可能无效，安全链仍零速 |
| Encoder 失败 | Tick 返回 nullopt；cmd_vel_raw 零速 |
| 无 goal | Runner 不产生命令 |
| policy 失败 | 无新 chunk，Blender 耗尽后零速 |
| safety 超时/非法 | 零速 + emergency |
| nav/safety heartbeat 丢失 | guardian REFLEX_ONLY/SAFE_STOP |
