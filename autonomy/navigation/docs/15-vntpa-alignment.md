# 15 · VNT-PA 对齐与优化清单

> 参考实现：[VNT-PA](../../../../ros2/ros-planning/VNT-PA)（Python）。
> 当前 C++ 实现：`planning/plugins/policy/vntpa/`。
> 结论：**当前 C++ 不是 VNT-PA 的等价实现，而是一个简化代理模型。**
>
> 当前状态（2026-11-08）：单 planner 的 **C++ 接口、memory patch tokens、离散动作解码、
> ActionPrimitive 输出** 已落地；`vntpa.onnx` 权重和 Python 导出仍是外部阻塞。

## 1. 核心差异

| 维度 | VNT-PA（Python） | 当前 C++ |
| --- | --- | --- |
| 模型结构 | DeFM patch summarizer + 6 层 map self-attention + 4 层 reader cross-attention + GRU chunk decoder | `VntpaPlanner` 简化版：拼接 depth/query/context + 4 层 TransformerEncoder |
| 输入 | 关键帧 patch tokens `(K,256,384)`、关键帧 pose `(K,3)`、query pose `(T,3)`、goal `(2)` | 当前帧 depth tokens `(1,256,384)`、query Fourier 64 维、context embedding、context pose 64 维 |
| 当前观测 | **不使用当前视角** | 使用当前帧 depth_feature |
| 位置编码 | 3D RoPE：x/z/yaw 分三份，yaw 可 yaw-free | `PoseAttention` 的 sin/tanh 手工编码，不是 RoPE |
| Goal | goal token 加入 map；query 输入 distance/bearing | 仅相对位姿编码，无 goal token/特征 |
| 动作 | 离散 4 类：STOP / FORWARD / LEFT / RIGHT | 连续 `Twist2D{x,y,theta}`；**目标契约仍是连续** |
| 解码 | 8 步 GRU，自回归，`exec_horizon=4` | ONNX 输出 `[T,8,3]` 连续动作，`execute_length` 可配 |
| Context | 最多 768 关键帧，coverage gate 选帧 | `kMaxContextEntries=64`，cosine novelty + 时间间隔 |
| 训练 | HM3D + navmesh expert，离散 chunk 交叉熵 | 无训练闭环；`tools/vntpa` 是玩具实现 |

## 2. P0：必须先解决

### 2.1 动作输出：保持连续

本项目的控制契约是连续的：

```proto
ActionChunk.actions: repeated Twist2D{x, y, theta}
```

因此不引入离散 `ActionPrimitive`，VNT-PA 侧也必须改成连续回归头。

需要修改：

1. Python planner 最后一层从 `N_ACTIONS=4` 分类改成 3 维回归：
   ```python
   self.head = nn.Linear(dim, 3)          # vx, vy, omega
   output: [T, chunk, 3]                  # 连续动作块
   ```
2. 训练 loss 从交叉熵改为 `SmoothL1` / `MSE`：
   ```python
   loss = F.smooth_l1_loss(pred, target_actions)
   ```
3. 专家标签由离散动作轨迹转成连续速度/位移：
   - `FORWARD` → `(v, 0, 0)`；
   - `TURN_LEFT/RIGHT` → `(0, 0, ±omega)`；
   - `STOP` → `(0, 0, 0)`；
   - 按 `action_dt_s` 归一化并受底盘限幅。
4. C++ `ActionDecoder` 保持连续 `std::vector<float>`；
5. `Postprocessor` 保持 `Twist2D` 限幅；
6. 不新增 `DiscreteActionChunk` / `ActionPrimitive`。

关键约束：

- ONNX 输出固定为 `[T, 8, 3]`；
- `action_dt_s` 必须与训练标签一致；
- `execute_length` 与 VNT-PA `exec_horizon` 对齐，建议默认 4；
- STOP 用全零速度表示，不占用离散类别。

### 2.2 ONNX 输入输出不匹配

当前 manifest：

```text
depth_feature [1,256,384]
query_token   [1,64]
context       [1,32,384]
context_pose  [1,32,64]
context_mask  [1,32]
action_chunk  [1,8,3]
```

VNT-PA 需要：

```text
kf_tokens   [K,256,384]   # 或已 summarize 的 [K,384]
kf_pose     [K,3]
query_pose  [T,3]
goal        [2]
actions     [T,8,3]        # 连续 (vx, vy, omega)
```

**不一定三阶段。** 一个 ONNX 也可以，取决于是否接受固定 context、重复计算和导出复杂度。

| 方案 | ONNX 数量 | 优点 | 代价 |
| --- | ---: | --- | --- |
| 全量单模型（DeFM + summarizer + map + read） | 1 | 一个制品、调用简单 | 输入固定 K×3×224×224；图巨大；无法缓存；导出/调试困难 |
| 单 planner（含 summarizer，输入 patch tokens） | 1 + DeFM | 一个 VNTPA 制品；C++ 简单 | 每次 query 重算所有 keyframe summarizer/map；K 必须固定 |
| planner（输入 summary entries）+ 独立 summarizer | 2 + DeFM | 关键帧 summary 可缓存；planner 较小 | 两个 VNTPA 模型/manifest；C++ 需要串联 |
| summarizer / map / read 三阶段 | 3 + DeFM | 可缓存 summary 和 goal memory；query 最便宜 | 模型管理、状态缓存最复杂 |

建议：

- **当前最优：单 planner ONNX**（输入 patch tokens），K 固定 64；
- 实测 query 延迟和显存后，再决定是否拆出 summarizer；
- 如果 K 接近 768 或 P99 超过 50 ms，再拆 `summarizer` / `map` / `read`；
- 不建议把 DeFM 和 planner 合成一个全量 ONNX。

```text
# 单 planner（推荐先落地）
inputs:  kf_tokens[K,256,384], kf_pose[K,3], query_pose[1,3], goal[2]
output:  logits[1,8,4]
```

```text
# 延迟优化后
defm_summarizer.onnx : [K,256,384] -> [K,384]
vntpa_map.onnx       : entries[K,384], kf_pose[K,3], goal[2] -> memory[K+1,768]
vntpa_read.onnx      : memory, query_pose[T,3], goal[2] -> logits[T,8,4]
```

如果只允许一个 VNTPA ONNX，优先选**单 planner + patch tokens**；三阶段不是硬要求。

### 2.3 Context 没有 patch tokens

`memory::Keyframe` 只有：

```cpp
std::array<float, 512> embedding;
```

VNT-PA 需要每个关键帧的 DeFM patch tokens 或 summarizer 输出。

建议二选一：

- **推荐**：插帧时运行 `defm_summarizer.onnx`，存 `summary[384]`；内存小，推理快；
- 保真：存 `tokens[256,384]`（fp16 约 196 KB/帧；768 帧约 150 MB）。

同时把 `kMaxContextEntries` 从 64 提升到可配置的 256–768，并加预算/子采样。

### 2.4 当前帧不应进入策略

VNT-PA 原文：

> no observation of the current view.

当前 `Policy::Query` 把当前 `depth_feature` 送入模型，行为不一致。

修复：

- 当前 depth 只用于**关键帧插入**和 `Observation`；
- 策略输入只来自 memory context；
- `Preprocessor` 删除 `depth_feature` 输入，改为 `kf_tokens`。

### 2.5 RoPE 与 Goal token

当前 `PoseAttention`：

```cpp
values = {x, y, sin(theta), cos(theta)}
frequency = 2^(band/4)
```

这不是 VNT-PA 的 3D RoPE。

建议：

- 在 ONNX 内实现 RoPE（推荐），C++ 只传原始 `(x,z,yaw)`；
- 或严格移植 `RoPE`：x/z 共用一个 metric frequency 表，yaw 用整数谐波，head_dim 必须能被 6 整除；
- goal token 加入 map，yaw 自由度置零；
- query 增加 `(distance, cos(bearing), sin(bearing))`。

## 3. P1：感知/记忆对齐

### 3.1 Coverage gate

Python `keyframe_gate.py`：

- 每个 DeFM patch 对应深度 cell；
- patch 足迹随 depth 增大；
- 多尺度 cell + bearing bin 去重；
- 保留“至少 eta 个 patch 未见”的帧。

C++ 当前 `KeyframeFilter` 使用 embedding cosine + 时间间隔，不是 coverage。

建议：

- 移植 `patch_depth` + `world_points` + cell keys；
- 用 patch token 对应的 16×16 深度 cell 计算 novelty；
- 配置 `keep_eta`、`levels`、`bearing_bins`；
- 单测与 Python `gate_labels` 对比。

### 3.2 关键帧选择

Python：

- `max_kf=768`；
- 按 coverage gate 选帧，超出均匀子采样；
- `eval` 用固定 seed 的 pose jitter。

C++：

- nearest 64；
- 无子采样策略；
- 无 pose jitter。

建议：最近邻 + 均匀覆盖混合；限制每米/每区域数量；支持 `max_kf` 配置。

## 4. P2：模型导出与训练

### 4.1 导出

Python `model.py` 的特点让 ONNX 导出不直接：

- `scaled_dot_product_attention`；
- 3D RoPE；
- bool mask；
- 自回归 GRU（8 步循环）；
- packed tokens + `scatter_packed`。

需要：

- 固定 `K=768`、`T=1`、`chunk=8` 的导出形状；
- 把 GRU 循环展开为 8 步；
- 用 `torch.onnx.export(opset=17/18)` 并用 onnxruntime 对比 logits；
- 导出 summarizer/map/read 三阶段。

当前 `tools/vntpa/export_onnx.py` 只导出玩具 `VntpaPlanner`，不能用于 VNT-PA checkpoint。

### 4.2 训练数据

Python 依赖：

- HM3D + habitat-sim；
- frontier exploration 采 tour；
- DeFM tokens；
- coverage gate labels；
- navmesh shortest-path expert；
- 离散 action chunk 交叉熵。

C++ 侧不需要重写训练，但需要：

- 导出前的 PyTorch 推理脚本与 C++ 对齐；
- 训练配置/checkpoint 版本记录；
- 模型 SHA/opset/IO 写入 manifest。

## 5. P3：运行时优化

### 5.1 缓存

Python eval 明确缓存：

```text
summarize_tour -> goal-independent summaries
encode_for_goal -> goal-dependent memory
```

C++ 当前每次 `Query` 重建全部 context，建议：

- keyframe 插入时算 summary；
- `SetGoal` 时算 map memory；
- 每个 query 只跑 reader/decoder；
- memory 以 `goal_epoch` 失效。

### 5.2 动作执行

- `exec_horizon=4`；
- STOP 立即结束；
- 动作原语按底盘速度/时间展开；
- 与 safety 的 `TwistStamped` 边界适配；
- 记录动作队列，避免每个 tick 重查。

### 5.3 指标

Python 评估：

- success（0.2 m geodesic）；
- SPL；
- stop rate；
- detour ratio 分桶。

C++ 目前没有闭环指标；需要 sim harness 输出同样字段。

## 6. 建议实施顺序

```text
P0-1 连续动作回归头（不使用离散）
P0-2 三阶段 ONNX + manifest
P0-3 memory 存 summary/tokens
P0-4 Preprocessor 去掉当前 depth，加 goal/RoPE
P1-1 coverage gate
P1-2 context 预算
P2-1 导出/对齐
P3-1 缓存
P3-2 闭环 SR/SPL
```

## 7. 不建议的做法

- 不要继续扩展当前 `PoseAttention` 假装是 RoPE；
- 不要保留 VNT-PA 的 4 类离散输出；本工程必须输出连续 `Twist2D`；
- 不要让 C++ 重新实现整个 transformer，优先 ONNX 黑盒；
- 不要把 768 个关键帧的 patch tokens 全量放进 64 槽内存。

## 8. 当前阻塞

参考目录没有 VNT-PA checkpoint：

```text
VNT-PA/third_party/defm/weights/   # 仅 .gitkeep
VNT-PA/ 无 *.pt / *.pth / *.onnx
```

因此：

- 可以改 C++/Python 接口与连续头；
- 无法验证与论文权重的数值一致性；
- 无法产出可比 SR/SPL 的闭环结果。

需要：

1. VNT-PA 训练好的 `final.pt`；或
2. HM3D 数据 + 训练机时；或
3. 已导出的 VNT-PA ONNX + manifest。

## 9. 验收

- [ ] 同一 checkpoint，PyTorch 与 C++/ONNX 输出 argmax 一致率 ≥ 99%；
- [ ] 连续动作块与 Python 回归输出一致（atol=1e-4）；
- [ ] context K=1/32/768 三种规模可运行；
- [ ] coverage gate 与 Python 标签一致率 ≥ 99%；
- [ ] 仿真 SR/SPL 与 Python eval 同量级；
- [ ] 端到端 P99 满足 N-07。

## 9. 最佳方案（本项目）

### 9.1 结论

当前代码的目标是 **K ≤ 64、5 Hz、单机器人、已有 DeFM ONNX**：

> **最佳方案 = 一个 VNTPA planner ONNX + 已有 DeFM ONNX，不做三阶段。**

```text
defm_vit_s14.onnx          # 已有，关键帧/建图编码
vntpa_planner.onnx         # 新增，VNT-PA 全图
    输入：kf_tokens [64,256,384] f16
          kf_pose   [64,3] f32
          kf_valid  [64] bool
          query_pose[1,3] f32
          goal      [2] f32
    输出：logits [1,8,4] f32
```

summarizer、6 层 map、4 层 reader、GRU 解码全部放在同一个 ONNX 图内。

### 9.2 为什么这是当前最优

| 维度 | 单 planner | 三阶段 |
| --- | --- | --- |
| C++ Engine 接口 | 直接复用现有单模型 `Run` | 需要三个模型和缓存状态 |
| 模型/manifest | 1 个 VNTPA | 3 个 VNTPA |
| 正确性 | 全图一次导出，端到端对齐 | 阶段边界误差/状态错误 |
| 当前 K=64 | 重算成本可接受 | 缓存收益有限 |
| 部署故障面 | 小 | 大 |
| 后续优化 | 可拆 | 已拆 |

K=64 时，summarizer + map 的重复计算在 GPU 上远低于 5 Hz 预算；先用单模型把
**数值正确性**做对，比提前拆阶段更重要。

### 9.3 什么时候切换三阶段

满足任一条件再拆：

- `K > 128` 或接近 VNT-PA 的 768；
- P99 `Query` > 50 ms；
- 同一 context 下频繁换 goal，map 需要跨 query 复用；
- 需要 CPU-only 部署。

切换后：

```text
defm_summarizer.onnx  # 插帧时
vntpa_map.onnx        # goal/context 变化时
vntpa_read.onnx       # 每 query
```

这是性能优化，不是正确性前置条件。

### 9.4 单 planner 的硬约束

- `K=64` 固定；不足补零，`kf_valid` 屏蔽；
- `T=1`；需要批量 query 时再加；
- GRU 8 步展开；
- 输出 `logits[1,8,4]`，不是 `[1,8,3]`；
- 输入必须是关键帧 patch tokens，不是当前帧；
- 动作解码为 STOP/FORWARD/LEFT/RIGHT。

### 9.5 不推荐

- 一个 ONNX 包含 DeFM + planner：图过大、无法复用 DeFM；
- 三阶段作为第一版：复杂度高，收益在 K=64 时有限；
- 单模型输入 summary：需要额外 summarizer，实际变成两个 VNTPA 模型。

**最终选择：单 VNTPA planner ONNX，K=64；性能不达标再拆三阶段。**
