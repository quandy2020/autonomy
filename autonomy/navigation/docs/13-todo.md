# 13 · TODO 与闭环验收

> 目标：把“能编译”推进到“仿真闭环可验收”，再推进到 VNTPA/BT 完整能力。
> 每个 TODO 必须有可执行的验收命令或观测指标，不做“只写代码不闭环”。

## 0. 闭环分级

| 级别 | 闭环 | 验收 |
| --- | --- | --- |
| L0 | 构建/单测 | Linux 上全量 build + CTest 通过 |
| L1 | 传感器→执行器 | autosim 发布 depth/odom，navigation 发布 `/cmd_vel`，机器人动 |
| L2 | 点到点 | `/navigate_to_pose` 目标，距目标 < 0.3 m |
| L3 | 避障 | 正前方障碍，机器人减速/绕行/停车，无碰撞 |
| L4 | 恢复 | 目标被挡 > 8 s，自动清目标重发并脱困 |
| L5 | 探索 | `TYPE_EXPLORATION` 下持续探索，无 frontier 时前进+转向 |
| L6 | 抢占 | 新 Action/topic 目标抢占旧目标，旧 Goal 返回 CANCELLED |
| L7 | 守护 | kill navigation/safety，guardian 输出降级 |
| L8 | VNTPA | Engine 加载 DeFM+VNTPA，输出动作块 |
| L9 | 任务 | following/coverage/door/elevator/docking 接目标与组件 |

**“仿真闭环完成”定义为 L1–L7 全绿；L8 需要模型权重；L9 是后续能力。**

## 1. P0：构建与运行时阻塞

### T-00 Linux 构建闭环

- [ ] `tools/build_linux.sh` 在 autolink + automsgs 环境构建通过
- [ ] 根工程 `autolink` 下三个 Component + Observation/Behavior 组件链接通过
- [ ] `ctest` 69/69（或更新后的数量）通过
- [ ] `cmake --install` 成功

验收：

```sh
tools/build_linux.sh /path/to/automsgs/build build-linux
ctest --test-dir build-linux --output-on-failure
cmake --install build-linux --prefix /opt/navigation
```

### T-01 修复重构后的编译/链接

近期改动的重点回归：

- [ ] `Pose2D`/`Twist2D` automsgs 别名全量编译
- [ ] `math_utils` 删除后 `autonomy::common::Clamp`/`wrap_angle` 调用
- [ ] `time_utils` 删除后 `ToSeconds`/`std::chrono` 调用
- [ ] `config.proto` 导入 `navigation.proto` 无循环
- [ ] `conf/models/vntpa.pb.txt` 重命名后的 Engine 路径
- [ ] `Observation` 新增 `map` 字段生成
- [ ] `configuration_validator` 只校验保留配置

### T-02 ObservationBuilderComponent 闭环

- [ ] DAG 能实例化 `navigation::perception::ObservationBuilderComponent`
- [ ] `/camera/depth` 触发，`/navigation/observation` 有数据
- [ ] camera_info/odom/imu/map 缺省不崩溃
- [ ] validity 位正确

验收：

```sh
autolink topic echo /navigation/observation --once
```

### T-03 Action server 闭环

- [ ] `/navigate_to_pose` action server ready
- [ ] Goal → `Runner.SetGoal` → 机器人动
- [ ] Feedback 有 current_pose/navigation_time/distance_remaining
- [ ] Cancel → `NAVIGATION_CANCELLED`
- [ ] 新 Action goal 抢占旧 Action
- [ ] `/navigation/goal` 抢占 Action
- [ ] Action 抢占 topic goal

### T-04 执行器类型闭环

- [ ] safety 输出 `TwistStamped`，autosim reader 接收
- [ ] 停止/零速时仍持续发布
- [ ] frame_id=`base_link`，stamp 与 depth 同源

### T-05 仿真时钟闭环

- [ ] Safety 使用 depth `Header.stamp` 做新鲜度
- [ ] depth 停发 > 200 ms 触发零速
- [ ] `/clock` 与 depth 时间不矛盾

### T-06 心跳/Guardian 闭环

- [ ] `/navigation/heartbeat`、`/safety/heartbeat`、`/behavior/heartbeat` 有数据
- [ ] kill navigation → guardian `REFLEX_ONLY`
- [ ] kill safety → guardian `SAFE_STOP`
- [ ] kill behavior → 预期降级

## 2. P1：仿真 L2–L7

### T-07 点到点 L2

- [ ] 发送目标 `(2,1,0)`，10 s 内到达
- [ ] 终点误差 < 0.3 m
- [ ] 无碰撞、无长时间零速

### T-08 避障 L3

- [ ] autosim 放置正前方障碍
- [ ] 反射层触发，速度下降/转向
- [ ] CBF/AEB 不输出不可行速度
- [ ] 障碍移除后可继续

### T-09 恢复 L4

- [ ] 目标前放不可穿越障碍
- [ ] 8 s 无进度触发 ClearGoal + SetGoal
- [ ] 恢复后不原地永久等待
- [ ] Action 反馈 `number_of_recoveries`（当前未填，TODO）

### T-10 探索 L5

- [ ] `initial_goal TYPE_EXPLORATION` 生效
- [ ] `/map` 有 frontier 时向最近 frontier
- [ ] 无 frontier 时前进+转向
- [ ] 不撞墙、不长期卡死

### T-11 抢占 L6

- [ ] Action A 执行中发 Action B，A 返回 CANCELLED，B 接管
- [ ] Action A 执行中发 topic goal，A 返回 CANCELLED
- [ ] topic goal 后发 Action，Action 接管
- [ ] 抢占后 feedback 重新计时

### T-12 Guardian L7

- [ ] 正常三心跳 → NORMAL
- [ ] 停导航心跳 → REFLEX_ONLY/SAFE_STOP
- [ ] 停安全心跳 → SAFE_STOP
- [ ] 降级消息有 from/to/reason

## 3. P2：模型与 VNTPA

### T-13 DeFM 推理闭环

- [ ] Engine 加载 `defm_vit_s14.onnx`
- [ ] 输入 `[1,3,224,224]`，输出 `[1,256,384]`/`[1,384]`
- [ ] 与 onnxruntime 误差 < 1e-4
- [ ] `manifest.pb` sha256 校验

### T-14 VNTPA 权重

- [ ] 训练/获取 `vntpa.onnx`
- [ ] 更新 `conf/models/vntpa.pb.txt` sha256
- [ ] `RegisterEnginePlugins()` 在部署入口调用
- [ ] `Runner::Create` policy `vntpa` 成功
- [ ] 输出 8 步动作块

### T-15 VNTPA 闭环 L8

- [ ] 目标点导航 SR/SPL 与论文指标一致
- [ ] 无目标时行为定义（探索/待机）
- [ ] 长距离 1000 m² 场景记忆条目受控

## 4. P3：任务与行为

### T-16 Behavior 闭环

- [x] 7 个节点改为 BT.CPP hpp/cpp + `BT_REGISTER_NODES`
- [x] 节点基类 `NavigationCondition` / `NavigationSyncAction`
- [ ] `RegisterBuiltinBtNodes` 在 BehaviorComponent 调用
- [ ] `node_context` blackboard 由 Runner/Behavior 写入
- [ ] BehaviorComponent 不只发心跳
- [ ] 绑定 `navigate_with_recovery.xml`
- [ ] `RecoveryNode`/`PipelineSequence` 实际执行
- [ ] 恢复动作接入 `Runner`

### T-17 场景任务

- [ ] following：目标输入 + 距离保持 [1,3] m
- [ ] coverage：Region 输入 + 覆盖率 >95%
- [ ] door/elevator：检测 + 状态机 + 仿真场景
- [ ] docking：目标 + reachability adjuster

### T-18 语义/交互

- [ ] click → 3D 目标
- [ ] language → GoalSpec
- [ ] semantic search 接场景图
- [ ] 无解时澄清请求

## 5. P4：工程化

### T-19 安全/可靠性

- [ ] TSan 全量通过
- [ ] ASan 全量通过
- [ ] 六类故障注入自动化
- [ ] 心跳/诊断覆盖每个模块

### T-20 性能

- [ ] N-01 反射 P99 < 30 ms
- [ ] N-02 安全链 < 50 ms
- [ ] N-03 关键帧 < 2 ms
- [ ] N-04 CBF < 1 ms
- [ ] N-05 Blender < 100 μs
- [ ] N-07 端到端 < 100 ms

### T-21 配置/文档

- [x] 配置模块化：3 主配置 + 4 平台 + 2 manifest
- [x] 文档编号 `00`–`13`
- [ ] DAG/launch 与文档一致
- [ ] 安装包包含 docs/conf/models

## 6. 建议执行顺序

```text
T-00/01 构建
  -> T-02/03/04/05/06 输入输出/时钟/心跳
  -> T-07/08/09/10/11/12 仿真 L2–L7
  -> T-13/14/15 VNTPA
  -> T-16/17/18 任务
  -> T-19/20/21 工程化
```

## 7. 当前已知不做

- SLAM / 地图构建
- 底层电机控制
- 多机协同
- 训练框架进 C++ 构建
- Nav2 通用全局规划器替代

## 8. 闭环完成标准

- `docs/08-acceptance.md` 中 L1–L7 全部 ✅；
- `01-requirements.md` F-01~F-25 中仿真范围项全部 ✅ 或有明确外部依赖；
- 任意目标/取消/抢占都有 Action Result；
- 传感器中断、心跳丢失、安全失败都进入 fail-safe；
- 所有 TODO 有对应测试或日志证据。
