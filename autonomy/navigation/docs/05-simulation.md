# 05 · autosim 仿真接入

## 1. 目标

在 autosim 中完成：

- 点到点导航
- 局部避障
- 基础恢复
- 前沿探索

不包含：语义跟随、Docking、门/电梯、完整重定位、真实 map localization。

点到点推荐用 Autoviz 工具栏 **2D Goal Pose**（`/goal_pose`，`PoseStamped`，`map` 系）。
脚本入口 `/navigation/goal` 与 Action `/navigate_to_pose` 仍可用。

## 2. Topic 映射


| autosim                   | navigation         | 类型                    |
| ------------------------- | ------------------ | --------------------- |
| `/camera/depth/image_raw` | ObservationBuilder | `Image` 32FC1         |
| `/camera/camera_info`     | ObservationBuilder | `CameraInfo`          |
| `/odom` + `/tf` `map→odom` | ObservationBuilder | 规划位姿变到 `map` |
| `/imu`                    | ObservationBuilder | `Imu`                 |
| `/map`                    | ObservationBuilder | `OccupancyGrid`       |
| `/cmd_vel`                | SafetyComponent 输出 | `TwistStamped`        |


autosim 的 `cmd_vel` reader 固定为 `TwistStamped`；navigation 安全链已按该类型输出。

点到点必须使用 **`mode: nav`** 的仿真配置（发布 `/map`，并由仿真维护 `map -> odom`）。
不要与 Atlas / Cartographer 的 `mode: slam` 同时抢 `map -> odom` 或 `/map`。
默认 `policy.name: simulation`：有图时 `PathTo` 绕障，前方深度 &lt; 0.8 m 时停车并转向更开阔一侧。Safety 仍做近场裁剪。`vntpa` 需要 `vntpa.onnx`，未提供前不要改策略名。

## 3. 外部目标入口

导航进程同时接受三种点目标，后到者抢占：

| 来源 | 通道 | 类型 | 用途 |
| --- | --- | --- | --- |
| Autoviz 2D Goal Pose | `/goal_pose` | `geometry_msgs/PoseStamped` | 视图里点选/拖拽朝向 |
| 脚本 | `/navigation/goal` | `navigation.GoalSpec` | `navigation_goal_sender` |
| Action | `/navigate_to_pose` | `NavigateToPoseAction` | 程序化客户端 |

`/navigate_to_pose` Goal：

```text
Goal {
  pose: geometry_msgs/PoseStamped   # map 系
  behavior_tree: ""                 # 保留，当前忽略
}
```

- 最新目标胜出：新 Action、`/navigation/goal` 或 `/goal_pose` 都会抢占当前目标；
- 都会覆盖 `initial_goal`（默认 `TYPE_EXPLORATION`）；
- Action Feedback：`current_pose`、`navigation_time`、`distance_remaining`；
- 取消：当前 Action 返回 `NAVIGATION_CANCELLED`；
- 抢占：旧 Action 返回 `NAVIGATION_CANCELLED`，新目标立即接管并重新计时；
- 完成：Action 返回 `OK`。

## 4. 启动

在 `SpaceHero` 容器内、工作区 `/workspace/autonomy`。先停掉 Atlas 等其它定位进程。

```sh
# 1. autosim（nav 模式：/map + map->odom）
cd /workspace/autonomy
CONFIG=src/autonomy/autosim/config/default.yaml \
  src/autonomy/autosim/scripts/run.sh sim
# 最快冒烟可用 fake_nav.yaml；Isaac 用 isaac_nav.yaml。
```

另开终端启动 Autoviz（工具栏 **2D Goal Pose**，Topic `/goal_pose`）。宿主机 `python3 docker/run_autonomy.py` 已起的 Autoviz 可直接用：

```sh
source /workspace/autonomy/install/setup.bash
autoviz --config /workspace/autonomy/src/autonomy/autoviz/config/default.autoviz
# atlas.autoviz 同样带 NavGoal；Fixed Frame 仍须为 map。
```

第三终端启动导航三进程。工作目录必须是 `src/autonomy/autonomy`，因为 DAG 里写的是相对路径 `navigation/dag/*.dag`：

```sh
source /workspace/autonomy/install/setup.bash
cd /workspace/autonomy/src/autonomy/autonomy
export PATH=/workspace/autonomy/install/autonomy/bin:/workspace/autonomy/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=/workspace/autonomy/.deps/onnxruntime-linux-x64-1.23.2/lib:/workspace/autonomy/install/autonomy/lib:${LD_LIBRARY_PATH:-}
export AUTOLINK_LAUNCH_PATH=$PWD/navigation/launch
export NAVIGATION_CONF_DIR=$PWD/navigation
autolink launch start navigation.launch
```

结束：

```sh
autolink launch stop navigation.launch
```

### Autoviz 发送 2D Goal Pose

1. Fixed Frame 设为 `map`，确认 `/map`、RobotModel、TF 已显示。
2. 工具栏选择 **2D Goal Pose**（快捷键 `g`）。工具属性 Topic 保持 `/goal_pose`。
3. 在 3D 视图地面单击位置，拖拽设置 yaw 后松开。
4. 状态栏应出现 `Setting goal pose: Frame:map Position(...) Angle:... → /goal_pose`。
5. 机器人应开始朝目标运动；`/navigation/status` 的 `goal.goal_id` 为 `goal_pose`，`/cmd_vel` 为 `TwistStamped`。

不要改成 `/navigation/goal`：该通道类型是 `GoalSpec`，与工具发布的 `PoseStamped` 不兼容。

Autoviz 是 Qt 客户端，无法从浏览器自动化点击。无 GUI 时发同一条 `PoseStamped` 到 `/goal_pose` 即可复现。

脚本等价（调试用，不是 Autoviz）：

```sh
navigation_goal_sender 2.0 1.0 0.0   # x y yaw，写 /navigation/goal
navigation_goal_sender              # 探索
```

默认配置 `conf/navigation.pb.txt`：

```textproto
policy { name: "simulation" ... }
initial_goal { type: TYPE_EXPLORATION }
```

未点选目标前会按探索策略运动；2D Goal Pose 会抢占为点到点。

## 5. 组件行为



### ObservationBuilderComponent

- depth 消息触发；
- 读最新 camera_info/odom/imu/map；
- 发布 `/navigation/observation`。



### NavigationComponent

- 启动前 `RegisterSimulationPlugins()`；
- 读 `/navigation/goal`（`GoalSpec`）与 `/goal_pose`（Autoviz `PoseStamped`），调用 `Runner::SetGoal`；
- 提供 `/navigate_to_pose` Action；
- 发布 `/navigation/cmd_vel_raw`、status、heartbeat；
- Runner 8 s 无进度上报状态，接收 Behavior 请求后执行局部绕行；耗尽终止 Action。



### SafetyComponent

- 读 raw cmd、depth、odom、camera_info；
- 采样 5 列深度生成障碍点；
- 通过 depth stamp 进展识别新帧，以 steady 接收时间计算 200 ms 租约；
- 输出 `TwistStamped` 与 `/safety/heartbeat`。



### BehaviorComponent

- 加载默认恢复 XML，注册内建节点、写入 node_context，消费 NavigationStatus；
- 请求恢复或终止当前目标，并发布 `/behavior/heartbeat`；详见 [19](19-behavior-recovery-validation.md)。



### GuardianComponent

- 聚合 nav/safety/behavior 心跳；
- 输出 `/guardian/degradation`。



## 6. 仿真策略

`SimulationPolicy`：

- `TYPE_POSE`：相对位姿比例控制，前进 + 转向；
- `TYPE_EXPLORATION`：有效 `Observation.map` 优先，选择与机器人连通的自由 frontier 成员，沿带墙体净空的四邻接路径前进；
- 无 map 时，启用的 FrontierNet 提供深度候选；空候选按 odom 累计扫描约一圈；
- 无前沿：已知地图扫描后等待更新；无地图则仅在前方深度 ≥1 m 时以 0.2 m/s 前进；
- 目标 8 s 无进展或 45 s 总预算耗尽后换目标，记忆默认最多 64 项/120 s；无盲倒车；
- 返回 4 步动作块，供 `ChunkBlender` 消费。

地图路径与姿态控制不依赖模型；`policy.frontiernet.enabled=true` 仍需成功加载 ONNX。
已有 DeFM adapter 制品尚未训练，不代表学习式探索质量已验收。

## 7. 已知限制

| 项 | 说明 |
| --- | --- |
| 路径 | pose 目标仍为直接 goal seek；地图探索增加已知自由栅格 BFS 路径，不引入全局 costmap 服务 |
| 障碍 | 只采样 depth 中心行 5 列，非完整点云 |
| 恢复 | 默认 XML 局部绕行/有限重试；复杂场景仍需验收 |
| 探索 | 全已知地图无 frontier 时扫描后等待；局部深度地图、帧对齐及覆盖率须单独验收 |
| 时钟 | 输出 header 用 depth stamp；Safety 新鲜度/心跳与 Runner dt 用 steady clock |
| Behavior | 默认恢复树已闭环；门/电梯/重定位任务树待联调 |
| 坐标系      | 2D Goal 在 `map`。Observation 用 `/tf` 的 `map→odom` 把 `/odom` 变到 map，再和点击目标比较。`atlas.yaml`（`mode: slam`）不要与本 launch 同时跑 |

## 8. 排障

| 现象 | 检查 |
| --- | --- |
| 不动 | `/navigation/observation` 的 odom `frame_id` 应为 `map`；status.stage 长时间 `flank` 说明误进恢复 |
| 立即零速 | safety 时间戳是否与 depth 同源；`/cmd_vel` 是否 TwistStamped |
| guardian 降级 | nav/safety/behavior heartbeat 是否存在 |
| 不探索 | `/map` 是否发布；frontier 是否为空；看 `[exploration] mode=scan/map_target/model_target/model_direct/no_frontier/abandoned` |
| 目标不生效 | `/navigation/goal` 的 type/pose/goal_id；sequence |
| Autoviz 点了没动 | 工具 Topic 是否 `/goal_pose`；Fixed Frame 是否 `map`；是否误用 atlas slam 配置 |
| 无 2D Goal 工具 | 使用 `default.autoviz`；`atlas.autoviz` 已加入 NavGoal，需重载配置 |

地图frontier的实现边界、真实深度栅格输入、600 s探索阶段与最终合并回归证据见
[20-exploration-validation.md](20-exploration-validation.md)。
