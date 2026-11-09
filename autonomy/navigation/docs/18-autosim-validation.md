# 18 · 真实 autosim 多进程验证记录

> 本文保留基础验收历史；默认行为树自主脱困的后续实现与证据见 [19](19-behavior-recovery-validation.md)。

验证日期：2026-11-09。环境为 `quandy@192.168.66.124` 的 `SpaceHero`，
workspace `/workspace/autonomy`，源码 `/workspace/autonomy/src/autonomy`。

## 场景与运行方式

`tools/run_autosim_acceptance.sh` 启动 opt-in 验收，不加入快速 CTest。
验收使用生产 autosim Runner、Autolink bridge、差速 Robot plant 和 Habitat RGB-D、
odom、IMU；生成一个有边墙的 OBJ 房间，在 Habitat 中插入/移除真实可渲染障碍。
RGB-D 为 160×120、20 Hz，关闭 chase/map，禁用 IMU 噪声，仿真时钟从 0 开始。

Navigation+Observation、Behavior、Safety、Guardian 分为四个独立 mainboard 进程，
Python Action 客户端和 autosim 各自独立运行。DAG 从生产文件提取，配置在启动时
快照到输出目录，模型/平台资源通过 `NAVIGATION_CONF_DIR` 解析。
运行于独立 IPC/挂载命名空间、独立 `/dev/shm` 和 `/tmp`，退出只清理本次创建的进程。

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy
bash src/autonomy/autonomy/navigation/tools/run_autosim_acceptance.sh \
  /workspace/autonomy/data/navigation-autosim-new
```

输出目录每次使用新名字。只跑部分检查时追加 `--checks clock,guardian`。
依赖当前环境的 Habitat Python `/opt/venv/bin/python3`；也可设置 `AUTOSIM_PYTHON`。
验收要求容器允许 `unshare --ipc --mount` 和 tmpfs 挂载。

## 已通过的真实场景检查

完整运行输出：`/workspace/autonomy/data/navigation-autosim-full2`，
控制台日志 `/tmp/navigation-autosim-full2.log`。七项全部通过，退出码 0。

- 传感器与执行器：Observation validity=31，包含 depth、camera_info、odom、IMU、TF；
  initial_goal 已产生真实前进位移，autosim 实际收到并执行 TwistStamped。
- 点 `(2,1,0)`：Action 在 9.0313 s 返回成功，真实位置误差 0.04650 m，
  每步几何检查无穿透，最小几何余量 3.5982 m。
- 正前方障碍：机器人停在 x=0.51687 m，半径 0.3 m 的 footprint 对障碍
  最小余量 0.18313 m；停车保持，移除障碍后继续前进。
- 深度暂停/epoch 重置：暂停后 0.36290 s 实际 plant 停车；Safety 持续发布零速，
  深度恢复并将 epoch 重置为 0 后继续；输出 frame=`base_link`、时间跟随 depth。
- 抢占/取消：真实 Python→C++ Action A→B、Action→topic→Action、旧目标终态、
  feedback 计时重置和取消零 raw command；取消不会重新执行缓存 topic。
- 无进度重试：障碍前停止后 6.8065 s 观察到 `number_of_recoveries=1`，
  对应距离不再改善满 8 s 后 clear/resubmit；计时起点不同于“首次检测到低速”。
  障碍移除后继续前进。此项仅验证基本目标重试与 feedback。
- 进程故障：kill Behavior/Navigation 后 Guardian 为 REFLEX_ONLY，kill Safety 后为
  SAFE_STOP；各进程重建后恢复 NORMAL。Navigation 被 kill 后 0.48634 s 实际停车，
  Safety 被 kill 后 0.60831 s 实际停车，分别包含 raw 指令租约和 plant watchdog/减速。

停车时间包含进程退出、消息/场景采样和执行器减速，不是 Guardian 检查周期或
200 ms 租约的精确性能测量。Guardian 检查周期仍为 500 ms。
补充运行 `navigation-autosim-stream` 检查暂停期间每条最终 linear/lateral/angular
指令都为零，以及 Navigation 被 kill 后 Safety 输出确实归零，三项检查全部通过。
该次深度暂停/Navigation kill/Safety kill 实际停车分别为 0.38334/0.43459/0.58771 s。

## 真实联调发现并修复的问题

Autolink Action 的六种 RPC envelope 只有字符串序列化方法，而服务发现要求
字符串与数组序列化四个方法齐全。增加带长度检查的 array serializer 后，服务
端点能够跨进程发现；请求/结果 envelope 使用 protobuf canonical type name，
使 C++ protobuf Action 与 Python bytes Action 的 wire type 一致。
组件回归包含六种 envelope 的 serializer trait 和两种 Action 类型的 wire type 检查。

Safety 仅约束加速度变化时，会把旧加速度带过目标速度，零指令后角速度仍持续振荡。
共享的平台速度/几何限幅先得到完整目标；可行性滤波预留 jerk 制动余量，满足
`a*dt + a²/(2*j) <= |target-measured|` 后再限制加速度变化。
闭环回归覆盖前进、反向、停车及 0/20/60 ms 测量延迟，并逐步检查速度、加速度、jerk
与最后连续 2 s 的收敛。原有单次加速度限幅断言继续保留。

Safety 每次收到 `/navigation/cmd_vel_raw` 都更新 steady clock 接收租约；超过
200 ms 未收到新指令即进入紧急零速，避免导航进程死亡后继续执行缓存速度。
无进度计时同样改用 steady clock，适用于 Action 和 topic pose 目标；新目标重置
计时/距离/恢复次数，Action feedback 填充次数。

完整构建后的组件回归还发现夹具会把取消后的减速位移误判为探索运动，
在新探索 raw 指令出现前提前结束等待。夹具现在先确认取消后的实际速度收敛，
再同时要求新探索正向 raw/final 指令与 >0.02 m 位移；保留探索 2 s 门限。

## 回归与证据文件

用户指定的 `colcon build --packages-up-to autonomy` 全量构建成功，耗时
3 min 45 s；夹具修复后的最后一次增量 colcon 成功，1.35 s。
最终导航 CTest **74/74 通过，0 失败，21.24 s**，包含 FrontierNet 新增回归。
更新记录后再次安装，并逐文件 SHA-256 比较配置、模型、DAG、launch、树和文档。

针对修复的三项 CTest 全通过：`velocity_feasibility_filter_test`、
`safety_shield_implementation_test`、`component_integration`，日志
`/tmp/navigation-feasibility-ctest.log`。完整构建/全量回归日志为
`/tmp/navigation-autosim-final-colcon.log`、`/tmp/navigation-autosim-final-ctest.log`；
完整重编译日志保留在 `/tmp/navigation-autosim-full-colcon.log`，安装资源比较日志
为 `/tmp/navigation-autosim-install-check.log`。Shell/Python 语法与主仓/Autolink
`git diff --check` 也通过。

每次场景输出保存配置快照、拆分 DAG、各进程日志、`summary.json`、
`scene_state.json`、每步采样轨迹 `trajectory.jsonl`、Habitat OBJ 和
`depth.npy`/`rgb.png`。失败运行保存 `failure.json`，包含最近 raw/final 指令和反馈。
`navigation-autosim-full1` 保存了修复前“raw 已零、最终角速度仍振荡”的证据，
`navigation-autosim-full2` 保存修复后的完整通过结果。

## 本次验收的边界

Habitat 的机器人通过运动学 set_pose 移动；每个 plant step 独立计算半径 0.3 m 的
footprint 与墙/障碍的几何余量。零碰撞表示该检查无几何穿透，**不代表物理接触
响应、刚体动力学碰撞或真实底盘制动认证**。本次只覆盖一个静态障碍场景。

此历史验收时点，L4 默认 BehaviorTree XML/恢复脱困、L5 长期探索/map frontier 闭环尚未完成，
不能把七个脚本检查称为完整 L1–L7 验收。FrontierNet 实验及模型限制见
[17-frontiernet.md](17-frontiernet.md)，VNTPA planner 权重仍缺。
ASan/TSan、CUDA/TensorRT 与目标硬件实时性能未在此运行中验证。
后续 T-09/T-16 默认树、保留障碍自主绕行、取消/重建与耗尽终态的实测更新见
[19](19-behavior-recovery-validation.md)。长期探索仍待推进，详见 [13-todo.md](13-todo.md)。

## FrontierNet 驱动的探索闭环

地图 frontier 优先、地图无候选回退 DeFM 的策略在真实 Habitat 场景
（`config/default.yaml`，17DRP5sb8fy）连续跑通，`/navigation/goal` 发
`TYPE_EXPLORATION` 后由 DeFM 输出驱动 `/cmd_vel`：

| 观察窗口 | 路径长度 | 净位移 | 转角 | 平移占空比 | `infeasible` |
| --- | --- | --- | --- | --- | --- |
| 120 s | 8.56 m | 3.69 m | 1.97 rad | 42% | 0 |
| 180 s | 10.97 m | 1.06 m | 4.53 rad | 39% | 0 |

约 5 min 的模式计数：`model_target` 19、`model_direct` 19、`reached` 4、
`abandoned` 14、`scan` 5；ONNX 单帧推理 30–100 ms
（`[frontiernet] pixels=2147 clusters=5 inference=31.1ms depth_ok=1 model_ready=1`）。

闭环里发现并修复的两个问题：

1. `FrontierDetectorTask::PathTo` 要求起点格 `distance > ceil(clearance/res)`；
   0.4 m / 0.5 m 分辨率下膨胀 1 格，机器人离墙 0.86 m 时起点格就不安全，
   所有路径为空。DeFM 目标改为在地图无路径时直接沿深度射线转向
   （`mode=model_direct`），地图目标仍要求路径。
2. 前向净空只检查画面中央带，漏掉桌腿等低矮障碍，机器人贴到 0.22 m 后
   安全层连续 298 次 `infeasible` 永久锁死。新增 `LowClearance`
   （画面 55–95% 行、25–75% 列，阈值 0.45 m）并入 `PathBlocked`，
   修复后同样的 180 s 窗口 `infeasible` 为 0。

复现：

```bash
# 1) 拉起 autosim（Habitat，default.yaml）与导航
# 2) 发探索目标
docker exec SpaceHero bash -lc "cd /workspace/autonomy && \
  AUTOLINK_DOMAIN_ID=82 LD_LIBRARY_PATH=/workspace/autonomy/build/autonomy/lib \
  ./build/autonomy/bin/navigation_goal_sender"
# 3) 观察
grep -o "mode=[a-z_]*" /tmp/nav.log | sort | uniq -c
grep -c infeasible /tmp/nav.log
```

边界：直行控制器只有前向/低位净空门限，没有局部绕障，杂物密集区会频繁
`abandoned`（14/19 次目标切换），覆盖率验收仍需局部规划器。
