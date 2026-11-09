# 15 · VNT-PA 对齐与优化清单

> 2026-11-09：DeFM CPU Engine 已通过数值验收。VNTPA 的 C++ 输入、关键帧 patch tokens、
> 离散动作解码与 ActionPrimitive 已接入；planner checkpoint/ONNX 仍缺失。
> 当前接口可测试，但尚无证据证明它与论文模型数值等价。

## 1. 当前采用的方案

采用 **一个 DeFM ONNX + 一个 VNTPA planner ONNX**，固定 `K=64`、`T=1`、8 步动作块。
planner 内部完成 summarizer、map attention、reader 和 decoder。
三阶段拆分属于后续性能候选，当前未测得延迟，不据此宣称单模型已满足 5 Hz 预算。

实际 C++ FloatTensorMap 与 `conf/models/vntpa.pb.txt` 的外部接口一致：

| 名称 | 形状 | 外部 dtype |
| --- | --- | --- |
| `kf_tokens` | `[64,256,384]` | float32 |
| `kf_pose` | `[64,3]` | float32 |
| `kf_valid` | `[64]` | float32，0/1 mask |
| `query_pose` | `[1,3]` | float32 |
| `goal` | `[1,2]` | float32 |
| `logits` | `[1,8,4]` | float32 |

manifest 的 `precision: fp16` 不改变外部 IO dtype。导出器应在图内将 mask 转为 bool，
需要内部 fp16 时显式转换。不能把 `kf_tokens` 外部声明为 f16 或 `kf_valid` 声明为 bool。

关键源码：`planning/plugins/policy/vntpa/{context_builder,query_builder,preprocessor,engine_model}.cpp`。
Preprocessor 从 memory context 读取关键帧 tokens；当前 depth 用于编码/插帧，
不作为额外 `depth_feature` 直接传入 planner。不足 64 帧补零并使用 `kf_valid`。

## 2. 动作契约

模型输出 4 类 logits，顺序固定：**STOP / FORWARD / LEFT / RIGHT**。
`ActionDecoder` 用 argmax 解码，按配置的步长与 `action_dt_s` 换算为连续速度；
`Postprocessor` 输出带动作原语的动作块，最终 Safety 对外发布连续 `TwistStamped`。

```text
FORWARD -> vx = forward_step / action_dt_s
LEFT    -> omega = turn_step_rad / action_dt_s
RIGHT   -> omega = -turn_step_rad / action_dt_s
STOP    -> zero
```

默认 chunk 长度 8、execute_length 4、action_dt_s 0.5 s，必须与训练/评估配置一致。
解码器仍保留旧连续 `[chunk,3]` 的兼容路径，但当前 VNTPA 导出目标是 `[1,8,4]`。
不需要把分类头改成连续回归头。数值对齐应同时比较 logits 与 argmax/动作原语，
不能只比较最终被安全层裁剪过的速度。

## 3. 已有能力与仍存在的差异

- memory 已保存 patch tokens，ContextBuilder 可构造固定 K 的输入；
  不是旧文档中的“只有 pooled embedding”。
- C++ QueryBuilder / ContextBuilder 把 ROS 平面位姿转成 Habitat `(x, z, yaw)`
  再写入 `query_pose` / `kf_pose` / `goal`。RoPE 仍在 ONNX 内。
- 有深度时 PoseIndexedMemory 使用论文 coverage gate（η=10，16×16 patch）；
  无 coverage keys 的测试路径仍走 cosine novelty。
- Query 默认把当前 DeFM tokens 写入 context slot 0（论文 `eval_accum`）；
  `vntpa.disable_live_view` 可关掉。
- 上游大 context 的行为和内存预算尚未验证。当前固定 K=64，不能宣称 K=768 可用。
- `tools/vntpa` 保留训练/导出脚手架；未用真实 checkpoint 完成上述接口的导出验收，
  不能作为论文模型等价制品交付。
- Engine 插件存在，但 NavigationComponent 当前注册仿真插件。真实模型入口、
  VNTPA manifest 哈希/元数据校验仍需与权重接入一起验收。

## 4. 下一步：权重、导出与对齐

需要训练后的 planner checkpoint 或匹配上述接口的 ONNX；也可在明确准备好
HM3D/habitat-sim 数据与训练机时后训练。随机权重不能计入导航验收。

1. 记录 checkpoint 来源、训练配置、许可证和 SHA-256。
2. 固定 K=64、T=1、chunk=8，展开 GRU 解码步骤，以 opset 17/18 导出单 planner。
3. 用同一组 keyframe/query/goal 比较 PyTorch、Python ORT、C++ Engine 的 logits。
4. 验证 1/32/64 个有效关键帧、padding/mask、STOP 与所有转向类别。
5. 更新 `conf/models/vntpa.pb.txt`，在入口注册 Engine 插件并验证 Runner::Create。
6. 在同一仿真场景/起终点集合中比较成功率、SPL、stop rate 与 detour ratio。

无目标时的待机/探索必须单独定义；当前仿真探索策略不能代替 VNTPA 的无目标策略。

## 5. 何时考虑拆阶段

先测目标硬件上的 Query P50/P99、CPU/GPU 内存和完整链路延迟。
当 K 增大、Query 超预算或目标切换导致重复计算明显时，再评估：

```text
summarizer：关键帧插入时缓存 summary
map：goal/context 变化时缓存 goal-dependent memory
reader/decoder：每次 query
```

拆分后必须为 goal/context epoch 建立缓存失效规则，并逐阶段和端到端验收。
CPU 部署是否需要拆分由实测决定。

## 6. 验收清单

- [x] C++ memory context 包含关键帧 patch tokens。
- [x] Query/Context/ActionDecoder 单元测试通过。
- [x] DeFM C++ Engine 对 Python ORT 最大误差 <1e-4。
- [ ] 真实 VNTPA checkpoint 与静态单 planner ONNX。
- [ ] 模型 hash/名称/dtype/形状严格匹配并拒绝错误制品。
- [ ] PyTorch/ORT/C++ logits 数值对齐，argmax 一致率 ≥99%。
- [ ] 有效 context K=1/32/64 的 padding/mask 对齐。
- [x] ROS↔Habitat 平面位姿写入 planner 输入。
- [x] 当前帧入库（live view）与 coverage gate C++ 实现。
- [ ] coverage gate 与 Python `keyframe_gate` 标签一致率 ≥99%。
- [ ] 同场景 SR/SPL 与 Python eval 可比较。
- [ ] 目标硬件端到端 P99 满足 N-07。

执行优先级与剩余闭环项见 [13-todo.md](13-todo.md)。
