# 11 · VNTPA 离线训练

与 `planning/plugins/policy/vntpa/` 的推理侧一一对应：

| 文件 | 作用 |
| --- | --- |
| `../tools/vntpa/tokens.py` | 位姿傅里叶编码、动作归一化（与 C++ `PoseAttention` 对齐） |
| `../tools/vntpa/model.py` | Transformer planner：depth tokens + query + context |
| `../tools/vntpa/bank.py` | 位姿索引关键帧库（K 近邻检索） |
| `../tools/vntpa/expert.py` | 仿真专家：A* + 速度平滑生成动作标签 |
| `../tools/vntpa/train.py` / `eval.py` / `explore.py` | 训练、评估、在线探索 |
| `../tools/vntpa/export_onnx.py` | 导出 opset 17，校验 I/O 名称与形状 |

## 流程

```sh
cd ../tools/vntpa && pip install -r requirements.txt
python expert.py --config config/train_vntpa.yaml      # 生成专家数据
python train.py --config config/train_vntpa.yaml
python eval.py --config config/eval_vntpa.yaml
python export_onnx.py --checkpoint checkpoints/best.pt --out ../../conf/models/vntpa.onnx
```

导出后把 sha256、I/O 形状写入 `../conf/models/vntpa.pb.txt`；推理侧
`ModelLoader` 会校验名称与形状。`checkpoints/` 只保留本地权重，不入库。

## 数据契约

- 每条样本：`depth_feature [N,D]`、`query [D]`、`context [K,D]`、
  `context_pose [K,P]`、`context_mask [K]`、`actions [T,3]`。
- 动作 `(vx, vy, ω)`，差分底盘 `vy=0`。
- `context` 由位姿索引库检索，训练与推理使用同一 `../tools/vntpa/tokens.py` 编码。
