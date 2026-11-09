# 13 · TODO 与闭环验收

> 2026-11-09：已接入默认行为树，九项真实 Habitat 检查通过，包含保留障碍自主绕行、恢复取消/重建与耗尽终态。
> 基础证据见 [18](18-autosim-validation.md)，本轮证据见 [19](19-behavior-recovery-validation.md)。复选框只覆盖明确写出的范围。

## 0. 闭环分级

L0 为 Linux 构建/测试/安装；L1–L7 依次为真实传感器执行链路、点到点、避障、
恢复、探索、抢占、守护；L8 为 VNTPA 策略；L9 为场景任务。
完整仿真验收要求 **真实 autosim L1–L7 全绿**。组件夹具验证是前置证据，
不替代真实场景、独立进程和碰撞验收。

## 1. P0：构建与组件

### T-00 Linux 构建

- [x] umbrella `colcon build --packages-up-to autonomy`。
- [x] Navigation/Safety/Guardian/Observation/Behavior 五组件实际编译链接。
- [x] 独立开关 `NAVIGATION_BUILD_TESTS` 与配置校验。
- [x] 最终全量导航 CTest 76/76（21.92 s）；安装资源验证见 [19](19-behavior-recovery-validation.md)。

```sh
cd /workspace/autonomy
src/autonomy/autonomy/navigation/tools/build_linux.sh /workspace/autonomy 8
```

### T-01 重构回归

- [x] Pose2D/Twist2D automsgs 别名及 common math/time 调用实际编译。
- [x] navigation proto 生成，无重复 namespace/import 编译错误。
- [x] map 字段生成、两份 manifest 与保留配置解析通过。
- [x] 修正 float 精度、离散动作类别顺序和避障方向的旧测试断言。

### T-02 ObservationBuilderComponent

- [x] 三 DAG 中 ObservationBuilder 实例化和 depth→Observation 组件验收。
- [x] camera/odom 缺省及加入后的 validity；IMU/map 缺省不崩溃。
- [x] 真实 autosim IMU、camera、odom、TF 输入与 Observation validity=31。
- [ ] map 输入与消费链路验收。
- [x] 真实 autosim `/camera/depth/image_raw`→`/navigation/observation`。

### T-03 Action server

- [x] 组件夹具的 acceptance→Runner→运动、feedback/result/cancel。
- [x] Action↔topic、Action A→B 抢占与反馈计时重置。
- [x] 真实独立 Python 客户端→C++ `/navigate_to_pose` 联调及 RPC 服务发现/类型一致性。

### T-04 执行器边界

- [x] 组件输出 TwistStamped，零速持续发布，frame=`base_link`、depth header 同源。
- [x] autosim reader 实际接收并执行。
- [x] Safety jerk 制动余量与速度闭环收敛回归，避免零指令后的角速度振荡。

### T-05 仿真时钟与断流

- [x] depth Header.stamp 的进展用于识别新帧；重复缓存 stamp 不续租。
- [x] 新鲜度使用接收端 steady clock，租约 200 ms，不拿仿真 epoch 与 steady clock 相减。
- [x] depth freshness 单测覆盖超时、stamp=0、模拟重启及时间倒退。
- [x] 组件停发 depth 超过 200 ms 停车、恢复后可继续。
- [x] 真实 autosim `/clock`、depth header 与暂停/epoch=0 重置验收。

### T-06 心跳/Guardian

- [x] 三模块心跳与 Guardian 判断使用 steady clock。
- [x] 辅助 reader 每周期 Observe，避免永远拿不到缓存消息。
- [x] 组件 NORMAL、导航/行为失效 REFLEX_ONLY、安全失效 SAFE_STOP；Behavior 重建恢复。
- [x] 真实进程 kill navigation/safety/behavior、降级及重启恢复验收。

## 2. P1：真实 autosim L2–L7

### T-07 点到点

- [x] 真实场景目标 `(2,1,0)`，8.9384 s 到达、终点误差 0.04349 m。
- [x] 此点导航无几何穿透、无长期零速；物理接触响应与更多场景仍待测。

### T-08 避障

- [x] Habitat 正前方静态障碍停车，无几何穿透，最小 footprint 余量 0.18329 m。
- [x] 障碍移除后继续；更多障碍/动态场景及物理接触碰撞验收仍待测。

### T-09 恢复

- [x] 静态障碍持续保留，steady 8 s 无进度由默认 XML 触发局部绕行；一次恢复后到达目标，无几何穿透。
- [x] Action/topic pose 目标的无进度检测、旧推理失效、恢复与反馈计数接入 Runner。
- [x] 恢复期间取消、新目标拒绝旧请求、Behavior 重建沿用预算与不可绕过墙体耗尽终态。
- [ ] 更多尺寸/动态障碍、复杂地图与刚体物理接触响应。

### T-10 探索

- [x] 真实 initial_goal 产生前进位移并经 Safety 到达执行器。
- [ ] 无 frontier 回退/转向及长期探索场景。
- [ ] map frontier 目标选择、无撞墙/长期卡死。

### T-11 抢占

- [x] Runner 取消/抢占清空 pending 工作并使旧异步结果失效。
- [x] 回归覆盖旧结果正在推理/已完成 × 直接抢占/先取消，以及目标到达零速。
- [x] Action/topic 目标所有权同步，独立记忆已见 topic，防止旧 topic 复活。
- [x] 组件级 Action/topic 抢占、取消与 feedback 重新计时。
- [x] 独立 Python/C++ 进程 Action/topic 抢占、取消、计时重置及缓存目标不复活。

### T-12 Guardian

- [x] 组件停用后 from/to/reason、REFLEX_ONLY/SAFE_STOP 与 Behavior 重建恢复验证。
- [x] 真实 kill 与执行器 fail-safe 联动：导航 raw 接收租约 200 ms；Safety 退出由 plant watchdog 停车。
- [x] 实际 plant 停车：Navigation kill 0.41938 s、Safety kill 0.61538 s，包含减速/采样延迟。

## 3. P2：模型

### T-13 DeFM

- [x] Engine 加载 `conf/models/defm_vit_s14.onnx`。
- [x] float32 输入 `[1,3,224,224]`，输出 `[1,256,384]`/`[1,384]`。
- [x] 三组 CPU C++/Python ORT 最大绝对误差为 0，满足 `<1e-4`。
- [x] `conf/models/defm.pb.txt` 哈希、名字/dtype/形状严格校验。
- [x] 错误输入/NaN/hash/名字拒绝，失败加载不保留旧模型。
- [ ] CUDA/TensorRT 及端到端深度预处理对齐。

PyTorch/ORT 的独立跨框架阈值为 5e-4，ramp tokens 实测 3.5154819e-4，
详见 [06-models.md](06-models.md)。

### T-14 VNTPA 权重（外部阻塞）

- [x] C++ Habitat 位姿、coverage gate、当前帧 context（无权重时仍不能当 planner 验收）。
- [ ] HM3D tokens + 30k 步论文训练，导出单 planner `vntpa.onnx`。
- [ ] 更新 manifest hash，严格校验 IO 和制品。
- [ ] 部署入口注册 Engine 插件，Runner::Create(policy=vntpa) 与 8 步 logits 验收。

### T-15 VNTPA L8

- [ ] 同场景 SR/SPL、无目标行为、长期场景记忆受控。
- [ ] K=1/32/64 padding/mask 与 Python 一致。

### T-22 FrontierNet 式探索 + DeFM 编码器

- [x] `python/frontiernet/` 骨架：官方 `TwoHeadUnet` 复刻、DeFM 五级 adapter、权重移植、`smoke/baseline/align`
- [x] 官方 `decoder` + 两个头可从 `rgbd_11cls.pth` 严格迁移（单测覆盖）
- [x] 用 autosim 的 depth 帧跑 `baseline`，固化官方输出 npz/json 作为对齐基线
- [x] 训练脚本：`frontiernet.collect` / `label` / `train`，冻结 DeFM backbone，训 adapter + decoder + 两个头
- [x] 重训后 region IoU 0.743、df MAE 0.085 m（240 帧 autosim，2040 步 / 135 s）
- [x] 导出 ONNX（`--load full`）并接入 C++：`conf/models/frontiernet_defm.onnx` sha256 `9f298a9f…`
- [x] 闭环里让模型分支生效：地图无候选时回退 DeFM，`PathTo` 失败时 `model_direct` 直行；autosim 120 s 位移 8.56 m、0 次 `infeasible`
- [ ] 换场景/楼层复采数据，验证跨房间泛化

验收：

```bash
cd autonomy/navigation/python
python3 -m frontiernet.cli smoke --backbone synthetic
python3 -m pytest tests/frontiernet -q
python3 -m frontiernet.cli align --weights rgbd_11cls.pth \
  --defm-weights ../conf/models/defm_vit_s14.pth \
  --depth frame.npy --rgb frame.png
```

## 4. P3：行为与任务

### T-16 Behavior 闭环

- [x] BT.CPP 节点与核心测试实际构建。
- [x] BehaviorComponent 注册内建节点、写入 node_context 并执行默认 XML；缺失 XML 初始化失败。
- [x] RecoveryNode/PipelineSequence 与 Runner 恢复接入；会话/目标/次数/新鲜度保护与持久预算。
- [x] 真实默认树自主绕行、取消/重建、耗尽 Action 终态与零速通过。
- [ ] door/relocalization 示例树的完整任务编排与场景验收（T-17）。

### T-17 场景任务

- [ ] following 目标与 [1,3] m 距离保持。
- [ ] coverage Region 与 >95% 覆盖率。
- [ ] door/elevator 检测、状态机和仿真场景。
- [ ] docking 目标与 reachability adjuster。

### T-18 语义/交互

- [ ] click→3D Goal、language→GoalSpec、场景图 semantic search、无解澄清。

## 5. P4：工程化

### T-19 可靠性

- [ ] 独立构建目录 ASan/TSan。
- [ ] 六类故障注入到真实进程、各模块诊断与执行器联动。

### T-20 性能

- [ ] 目标硬件 N-01 反射 P99 <30 ms、N-02 安全链 <50 ms。
- [ ] N-03 关键帧 <2 ms、N-04 CBF <1 ms、N-05 Blender <100 μs。
- [ ] N-07 端到端 <100 ms。

### T-21 配置/文档

- [x] 三主配置、四平台、两份 manifest 解析。
- [x] 构建/模型/VNTPA 文档统一到当前实际接口，删除旧连续头和强制三阶段描述。
- [x] DAG/launch 与安装后的 docs/conf/models 实际资源校验。

## 6. 后续顺序

默认 T-09/16 局部恢复闭环已完成，优先推进 T-10 map frontier 与长期探索，
再推进 T-14/15 planner 权重及数值对齐、场景任务和目标硬件可靠性/性能。
SLAM、底层电机控制、多机协同和训练框架加入 C++ 构建不在本模块范围。
