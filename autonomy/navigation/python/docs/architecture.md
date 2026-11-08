# 架构

`vntpa` 采用 flat layout，包目录 `vntpa/` 直接位于工程根。

```text
configs/
   │
   ▼
vntpa.cli  ──────────────┐
   │                     │
   ▼                     ▼
training / evaluation   runtime / models
   │                     │
   ▼                     ▼
data  ◄──────────────  onnx / autosim
```

## 数据流

```text
Habitat/HM3D
  -> frontier exploration
  -> tours (poses, depth)
  -> DeFM tokens HDF5
  -> planner bank (query, goal, action chunk)

autosim depth
  -> DeFM ONNX
  -> keyframe context
  -> VNT-PA ONNX
  -> discrete action chunk
  -> TwistStamped /cmd_vel
```

## 六个顶层子包

| 子包 | 职责 |
| --- | --- |
| `data` | dataset、transforms、sampler、collate、collection |
| `models` | DeFM encoder、VNT-PA planner、query/context/action/head |
| `training` | trainer、losses、optimizer、checkpoint、callbacks、evaluation |
| `runtime` | deployment、inference、sim/autolink |
| `cli` | collect、train、eval、export、validate |
| `utils` | config、errors、logging、seed、geometry、math、timer、visualization |

## 模型接口

DeFM：

```text
depth [1,3,224,224]
  -> tokens [1,256,384]
  -> pooled [1,384]
```

VNT-PA：

```text
kf_tokens  [64,256,384]
kf_pose    [64,3]
kf_valid   [64]
query_pose [1,3]
goal       [1,2]
  -> logits [1,8,4]
```

`kf_tokens` 是 padding 后的 K 帧；导出 wrapper 内部按 `kf_valid` 打包成上游要求的
`(n_valid, P, D)`。

## 对齐上游

训练/数据代码 vendored 自
[VNT-PA](https://github.com/beimingli0626/VNT-PA)：

```text
pan/collection/explore.py -> tours
pan/collection/tokens.py  -> frozen DeFM token stores
pan/collection/bank.py    -> evaluation bank
pan/planner/train.py      -> DDP imitation training
pan/planner/eval.py       -> closed-loop evaluation
```

本项目只增加：

- 本地 DeFM 权重加载，不访问 Hugging Face；
- 单 planner ONNX 导出/接口校验；
- autosim/autolink bridge；
- TensorBoard 日志。
