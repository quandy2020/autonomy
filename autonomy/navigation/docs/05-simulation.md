# 05 · autosim 仿真接入

## 1. 目标

在 autosim 中完成：

- 点到点导航
- 局部避障
- 基础恢复
- 前沿探索

不包含：语义跟随、Docking、门/电梯、完整重定位、真实 map localization。

## 2. Topic 映射

| autosim | navigation | 类型 |
| --- | --- | --- |
| `/camera/depth/image_raw` | ObservationBuilder | `Image` 32FC1 |
| `/camera/camera_info` | ObservationBuilder | `CameraInfo` |
| `/odom` | ObservationBuilder | `Odometry`（map->base） |
| `/imu` | ObservationBuilder | `Imu` |
| `/map` | ObservationBuilder | `OccupancyGrid` |
| `/cmd_vel` | SafetyComponent 输出 | **`TwistStamped`** |

autosim 的 `cmd_vel` reader 固定为 `TwistStamped`；navigation 安全链已按该类型输出。

## 3. 外部 navigate_to_pose

导航进程启动后提供 `/navigate_to_pose` Action：

```text
Goal {
  pose: geometry_msgs/PoseStamped   # map 系
  behavior_tree: ""                 # 保留，当前忽略
}
```

- 最新目标胜出：新 Action goal 或 `/navigation/goal` 都会抢占当前目标；
- Action 目标会覆盖初始目标；
- Feedback：`current_pose`、`navigation_time`、`distance_remaining`；
- 取消：当前 goal 返回 `NAVIGATION_CANCELLED`。
- 抢占：旧 goal 返回 `NAVIGATION_CANCELLED`，新 goal 立即接管并重新计时。
- 完成：返回 `OK`。
- 话题方式 `/navigation/goal` 仍保留；它也会抢占当前 Action 目标。

## 4. 启动

```sh
# 1. autosim
# 2. navigation（Linux + autolink）
navigation_launch navigation/launch/navigation.launch

# 3. 目标
navigation_goal_sender 2.0 1.0 0.0   # x y yaw
navigation_goal_sender              # 探索
```

默认配置 `conf/navigation.pb.txt`：

```textproto
policy { name: "simulation" ... }
initial_goal { type: TYPE_EXPLORATION }
```

## 5. 组件行为

### ObservationBuilderComponent

- depth 消息触发；
- 读最新 camera_info/odom/imu/map；
- 发布 `/navigation/observation`。

### NavigationComponent

- 启动前 `RegisterSimulationPlugins()`；
- 读 `/navigation/goal`，调用 `Runner::SetGoal`；
- 发布 `/navigation/cmd_vel_raw`、status、heartbeat；
- 8 s 无进度重发目标（基础恢复）。

### SafetyComponent

- 读 raw cmd、depth、odom、camera_info；
- 采样 5 列深度生成障碍点；
- 使用 depth stamp 作为仿真时间；
- 输出 `TwistStamped` 与 `/safety/heartbeat`。

### BehaviorComponent

- 当前只发布 `/behavior/heartbeat`，供 guardian 判断模块存活。

### GuardianComponent

- 聚合 nav/safety/behavior 心跳；
- 输出 `/guardian/degradation`。

## 6. 仿真策略

`SimulationPolicy`：

- `TYPE_POSE`：相对位姿比例控制，前进 + 转向；
- `TYPE_EXPLORATION`：`Observation.map` + `FrontierDetectorTask` 选最近前沿；
- 无前沿：前进 + 转向；
- 返回 4 步动作块，供 `ChunkBlender` 消费。

模型无关；不依赖 ONNX/TensorRT。

## 7. 已知限制

| 项 | 说明 |
| --- | --- |
| 全局路径 | 无；直接 goal seek，靠安全链避障 |
| 障碍 | 只采样 depth 中心行 5 列，非完整点云 |
| 恢复 | 清目标重发，非完整 BT 恢复 |
| 探索 | GT map 可能全已知，需前进+转向兜底 |
| 时钟 | Safety 用仿真 stamp；Runner 用 steady dt |
| Behavior | 无完整任务编排 |

## 8. 排障

| 现象 | 检查 |
| --- | --- |
| 不动 | `/navigation/observation` 是否发布；`Runner::Create` 是否失败 |
| 立即零速 | safety 时间戳是否与 depth 同源；`/cmd_vel` 是否 TwistStamped |
| guardian 降级 | nav/safety/behavior heartbeat 是否存在 |
| 不探索 | `/map` 是否发布；frontier 是否为空（看 fallback 前进） |
| 目标不生效 | `/navigation/goal` 的 type/pose/goal_id；sequence |
